#!/usr/bin/env python3
"""
Quaternion State Manager / مدير حالة الرباعي

Manages two distinct quaternion states:
1. Recovery (`_last_valid_quaternion`): fallback when norm→0 or NaN
2. Sign Continuity (`_sign_continuity_reference`): keeps q/-q consistent across frames

يدير حالتين منفصلتين للرباعي:
1. الاستعادة: احتياطي عند تدهور الرباعي (norm→0 أو NaN)
2. استمرارية الإشارة: يحافظ على اتساق q/-q بين الإطارات

Do NOT merge: recovery = simulation concern, sign continuity = math concern.
States sync on valid quaternion update or recovery.
لا تدمجهما: الاستعادة شأن محاكاة، استمرارية الإشارة شأن رياضي.
"""

import numpy as np
from typing import Optional, Tuple, Dict, Any
from dataclasses import dataclass
import logging

from dynamics.quaternion_utils import (
    QUAT_NORM_THRESHOLD,
    ensure_quaternion_sign_continuity,
    normalize_quaternion,
)

logger = logging.getLogger(__name__)


@dataclass
class QuaternionStateInfo:
    """Quaternion state snapshot for debugging. / لقطة حالة الرباعي للتصحيح."""
    last_valid_quaternion: Optional[np.ndarray]
    sign_continuity_reference: Optional[np.ndarray]
    intermediate_quaternion_reference: Optional[np.ndarray]
    recovery_count: int
    max_recoveries: int
    is_synchronized: bool
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for logging/serialization."""
        return {
            'last_valid_quaternion': self.last_valid_quaternion.tolist() if self.last_valid_quaternion is not None else None,
            'sign_continuity_reference': self.sign_continuity_reference.tolist() if self.sign_continuity_reference is not None else None,
            'intermediate_quaternion_reference': self.intermediate_quaternion_reference.tolist() if self.intermediate_quaternion_reference is not None else None,
            'recovery_count': self.recovery_count,
            'max_recoveries': self.max_recoveries,
            'is_synchronized': self.is_synchronized,
        }


class QuaternionStateManager:
    """
    Unified quaternion state manager: recovery + sign continuity + intermediate.
    مدير موحد لحالة الرباعي: استعادة + استمرارية إشارة + رباعي وسيط.

    IMPORTANT: This is the SINGLE sign continuity application point for stateful ops.
    Do NOT also use internal sign continuity in stateless functions to avoid double application.
    مهم: هذا هو نقطة التطبيق الوحيدة لاستمرارية الإشارة. لا تستخدمها مع الدوال بدون حالة.

    See quaternion_utils.py for architecture details.
    """
    
    def __init__(
        self,
        max_recoveries: int = 3,
        quat_norm_threshold: float = QUAT_NORM_THRESHOLD
    ):
        """Args: max_recoveries, quat_norm_threshold."""
        self._max_recoveries = max_recoveries
        self._quat_norm_threshold = quat_norm_threshold

        # Recovery: fallback for degenerate quaternions (warn_and_recover mode)
        # استعادة: احتياطي للرباعيات المتدهورة
        self._last_valid_quaternion: Optional[np.ndarray] = None
        self._recovery_count: int = 0

        # Sign continuity: keeps q/-q consistent across frame transforms
        # استمرارية الإشارة: يحافظ على اتساق q/-q بين الإطارات
        self._sign_continuity_reference: Optional[np.ndarray] = None

        # Intermediate sign continuity: separate ref for frame-to-frame quaternions
        # (distinct rotation from body-to-frame, so needs its own reference)
        # استمرارية إشارة وسيطة: مرجع منفصل لرباعيات إطار-إلى-إطار
        self._intermediate_quaternion_reference: Optional[np.ndarray] = None
    
    @property
    def max_recoveries(self) -> int:
        """Get maximum allowed recoveries."""
        return self._max_recoveries
    
    @property
    def recovery_count(self) -> int:
        """Get current recovery count."""
        return self._recovery_count
    
    @property
    def quat_norm_threshold(self) -> float:
        """Get quaternion norm threshold."""
        return self._quat_norm_threshold
    
    def is_quaternion_valid(self, quaternion: np.ndarray) -> bool:
        """
        Check if a quaternion is valid (finite and non-degenerate).
        التحقق مما إذا كان الرباعي صالحاً (منتهي وغير متدهور).
        
        Args:
            quaternion: Quaternion to check [q0, q1, q2, q3]
            
        Returns:
            True if quaternion is valid, False otherwise
        """
        if not np.all(np.isfinite(quaternion)):
            return False
        quat_norm = np.linalg.norm(quaternion)
        return quat_norm > self._quat_norm_threshold
    
    def update_valid_quaternion(self, quaternion: np.ndarray) -> bool:
        """Update recovery + sign continuity states with a valid quaternion. Returns True if updated.
        تحديث حالتي الاستعادة واستمرارية الإشارة برباعي صالح."""
        if not self.is_quaternion_valid(quaternion):
            return False
        
        # Normalize the quaternion
        quat_norm = np.linalg.norm(quaternion)
        normalized_quat = quaternion / quat_norm
        
        # Update both states
        self._last_valid_quaternion = normalized_quat.copy()
        self._sign_continuity_reference = normalized_quat.copy()
        
        return True
    
    def get_recovery_quaternion(self) -> Optional[np.ndarray]:
        """Return copy of last valid quaternion, or None. / آخر رباعي صالح أو None."""
        if self._last_valid_quaternion is not None:
            return self._last_valid_quaternion.copy()
        return None
    
    def get_sign_continuity_reference(self) -> Optional[np.ndarray]:
        """Return copy of sign continuity reference, or None. / مرجع استمرارية الإشارة أو None."""
        if self._sign_continuity_reference is not None:
            return self._sign_continuity_reference.copy()
        return None
    
    def set_sign_continuity_reference(self, quaternion: np.ndarray) -> None:
        """Set sign continuity reference (for recovery or external init). / تعيين مرجع استمرارية الإشارة."""
        if self.is_quaternion_valid(quaternion):
            quat_norm = np.linalg.norm(quaternion)
            self._sign_continuity_reference = (quaternion / quat_norm).copy()
    
    def reset_sign_continuity_reference(self) -> None:
        """Reset sign continuity ref to None; next quaternion becomes new ref. / إعادة تعيين المرجع."""
        self._sign_continuity_reference = None
    
    def apply_sign_continuity(self, quaternion: np.ndarray) -> np.ndarray:
        """Apply sign continuity; if no ref exists, quaternion becomes the ref.
        تطبيق استمرارية الإشارة؛ إذا لم يوجد مرجع، الرباعي يصبح المرجع."""
        if self._sign_continuity_reference is None:
            # First quaternion becomes the reference
            if self.is_quaternion_valid(quaternion):
                quat_norm = np.linalg.norm(quaternion)
                self._sign_continuity_reference = (quaternion / quat_norm).copy()
            return quaternion.copy()
        
        # Apply sign continuity
        result = ensure_quaternion_sign_continuity(
            quaternion, self._sign_continuity_reference
        )
        
        # Update reference
        if self.is_quaternion_valid(result):
            quat_norm = np.linalg.norm(result)
            self._sign_continuity_reference = (result / quat_norm).copy()
        
        return result
    
    # --- Intermediate Quaternion Methods (separate sign continuity for frame-to-frame) ---
    # --- طرق الرباعي الوسيط (استمرارية إشارة منفصلة لتحويلات الإطارات) ---
    
    def get_intermediate_quaternion_reference(self) -> Optional[np.ndarray]:
        """Return copy of intermediate quaternion reference, or None. / مرجع الرباعي الوسيط أو None."""
        if self._intermediate_quaternion_reference is not None:
            return self._intermediate_quaternion_reference.copy()
        return None
    
    def set_intermediate_quaternion_reference(self, quaternion: np.ndarray) -> None:
        """Set intermediate quaternion ref (for init or recovery sync). / تعيين مرجع الرباعي الوسيط."""
        if self.is_quaternion_valid(quaternion):
            quat_norm = np.linalg.norm(quaternion)
            self._intermediate_quaternion_reference = (quaternion / quat_norm).copy()
    
    def reset_intermediate_quaternion_reference(self) -> None:
        """Reset intermediate ref to None; next one becomes new ref. / إعادة تعيين مرجع الرباعي الوسيط."""
        self._intermediate_quaternion_reference = None
    
    def apply_intermediate_sign_continuity(self, quaternion: np.ndarray) -> np.ndarray:
        """Apply sign continuity to intermediate quaternion (separate ref from body).
        تطبيق استمرارية الإشارة على رباعي وسيط (مرجع منفصل عن الجسم)."""
        if self._intermediate_quaternion_reference is None:
            # First intermediate quaternion becomes the reference
            if self.is_quaternion_valid(quaternion):
                quat_norm = np.linalg.norm(quaternion)
                self._intermediate_quaternion_reference = (quaternion / quat_norm).copy()
            return quaternion.copy()
        
        # Apply sign continuity
        result = ensure_quaternion_sign_continuity(
            quaternion, self._intermediate_quaternion_reference
        )
        
        # Update reference
        if self.is_quaternion_valid(result):
            quat_norm = np.linalg.norm(result)
            self._intermediate_quaternion_reference = (result / quat_norm).copy()
        
        return result
    
    def record_recovery(self) -> Tuple[bool, int]:
        """Deprecated: use perform_recovery() instead (auto-syncs all refs).
        مهمل: استخدم perform_recovery() بدلاً من ذلك.
        Returns (is_allowed, current_count)."""
        import warnings
        warnings.warn(
            "record_recovery() is deprecated. Use perform_recovery() instead, "
            "which automatically synchronizes all quaternion references after recovery. "
            "record_recovery() مهمل. استخدم perform_recovery() بدلاً من ذلك.",
            DeprecationWarning,
            stacklevel=2
        )
        self._recovery_count += 1
        is_allowed = self._recovery_count <= self._max_recoveries
        return is_allowed, self._recovery_count
    
    def perform_recovery(self, identity_fallback: bool = True) -> Tuple[bool, int, np.ndarray]:
        """Atomic recovery: record attempt + get recovery quat + sync all refs.
        استعادة ذرية: تسجيل المحاولة + رباعي الاستعادة + مزامنة جميع المراجع.

        Args:
            identity_fallback: use [1,0,0,0] if no history (else ValueError)
        Returns: (is_allowed, recovery_count, recovery_quaternion)
        Raises: ValueError if identity_fallback=False and no history
        """
        # Step 1: Record recovery attempt (without deprecation warning)
        self._recovery_count += 1
        is_allowed = self._recovery_count <= self._max_recoveries
        
        # Step 2: Get recovery quaternion
        recovery_quat = self.get_recovery_quaternion()
        if recovery_quat is None:
            if identity_fallback:
                recovery_quat = np.array([1.0, 0.0, 0.0, 0.0])
            else:
                raise ValueError(
                    "No valid quaternion history available for recovery and "
                    "identity_fallback=False. "
                    "لا يتوفر تاريخ رباعي صالح للاستعادة و identity_fallback=False."
                )
        
        # Step 3: Synchronize all references (ALWAYS, even if recovery not allowed)
        # This ensures consistent state even when we're about to raise an error
        # هذا يضمن حالة متسقة حتى عندما نكون على وشك رفع خطأ
        self.synchronize_from_recovery(recovery_quat)
        
        return is_allowed, self._recovery_count, recovery_quat
    
    def reset_recovery_count(self) -> None:
        """Reset recovery count to zero. / إعادة تعيين عداد الاستعادة."""
        self._recovery_count = 0
    
    def get_state_info(self) -> QuaternionStateInfo:
        """Get QuaternionStateInfo snapshot for debugging. / لقطة حالة الرباعي للتصحيح."""
        # Check if states are synchronized (both None or both equal)
        if self._last_valid_quaternion is None and self._sign_continuity_reference is None:
            is_synchronized = True
        elif self._last_valid_quaternion is None or self._sign_continuity_reference is None:
            is_synchronized = False
        else:
            is_synchronized = np.allclose(
                self._last_valid_quaternion,
                self._sign_continuity_reference,
                atol=QUAT_NORM_THRESHOLD
            )
        
        return QuaternionStateInfo(
            last_valid_quaternion=self._last_valid_quaternion.copy() if self._last_valid_quaternion is not None else None,
            sign_continuity_reference=self._sign_continuity_reference.copy() if self._sign_continuity_reference is not None else None,
            intermediate_quaternion_reference=self._intermediate_quaternion_reference.copy() if self._intermediate_quaternion_reference is not None else None,
            recovery_count=self._recovery_count,
            max_recoveries=self._max_recoveries,
            is_synchronized=is_synchronized,
        )
    
    def reset(self) -> None:
        """
        Reset all state to initial values.
        إعادة تعيين كل الحالة إلى القيم الأولية.
        """
        self._last_valid_quaternion = None
        self._sign_continuity_reference = None
        self._intermediate_quaternion_reference = None
        self._recovery_count = 0
    
    def synchronize_from_recovery(self, recovery_quaternion: np.ndarray) -> None:
        """Sync all refs (sign continuity + last valid + intermediate) after recovery.
        مزامنة جميع المراجع بعد الاستعادة لمنع انقلابات الإشارة."""
        if self.is_quaternion_valid(recovery_quaternion):
            quat_norm = np.linalg.norm(recovery_quaternion)
            normalized = recovery_quaternion / quat_norm
            self._sign_continuity_reference = normalized.copy()
            # Also update last valid since this is now our reference
            self._last_valid_quaternion = normalized.copy()
            # Also sync intermediate ref to prevent sign flips after recovery
            # مزامنة مرجع الرباعي الوسيط لمنع انقلابات الإشارة بعد الاستعادة
            self._intermediate_quaternion_reference = normalized.copy()
    
    def log_state(self, context: str = "") -> None:
        """
        Log current state for debugging.
        تسجيل الحالة الحالية للتصحيح.
        
        Args:
            context: Optional context string for the log message
        """
        info = self.get_state_info()
        prefix = f"[{context}] " if context else ""
        
        logger.debug(
            "%sQuaternionStateManager: recovery_count=%d/%d, synchronized=%s, "
            "last_valid=%s, sign_ref=%s, intermediate_ref=%s",
            prefix,
            info.recovery_count,
            info.max_recoveries,
            info.is_synchronized,
            info.last_valid_quaternion,
            info.sign_continuity_reference,
            info.intermediate_quaternion_reference,
        )
