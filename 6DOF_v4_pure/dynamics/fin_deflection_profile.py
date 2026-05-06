#!/usr/bin/env python3
"""
تحميل واستيفاء بيانات انحراف الزعانف من ملفات DAT/CSV خارجية.
"""

import numpy as np
from scipy import interpolate
from pathlib import Path
from typing import Optional, Tuple, List
import logging

logger = logging.getLogger(__name__)


class FinDeflectionProfile:
    """تحميل واستيفاء انحراف الزعانف من ملف (أعمدة: time, dac1-dac4)."""
    
    def __init__(self, file_path: str, units: str = 'degrees',
                 time_offset: float = 0.0):
        """تهيئة من ملف (file_path, units='degrees'/'radians', time_offset)."""
        self.file_path = Path(file_path)
        self.units = units.lower()
        self.time_offset = time_offset
        
        if not self.file_path.exists():
            raise FileNotFoundError(
                f"Fin deflection file not found: {file_path}\n"
                f"ملف انحراف الزعانف غير موجود: {file_path}"
            )
        
        if self.units not in ['degrees', 'radians']:
            raise ValueError(
                f"Invalid units '{units}'. Must be 'degrees' or 'radians'.\n"
                f"وحدات غير صالحة '{units}'. يجب أن تكون 'degrees' أو 'radians'."
            )
        
        # Load data from file
        self._load_data()
        
        # Create interpolators for each fin
        self._create_interpolators()
        
        logger.info(
            f"Loaded fin deflection profile from {file_path}: "
            f"{len(self.time_points)} points, "
            f"time range [{self.time_points[0]:.3f}, {self.time_points[-1]:.3f}] s"
        )
    
    def _load_data(self):
        """
        Load fin deflection data from file.
        """
        # Detect file format based on extension and content
        suffix = self.file_path.suffix.lower()
        
        try:
            # Try to read as tab-separated first (DAT format)
            with open(self.file_path, 'r', encoding='utf-8') as f:
                first_line = f.readline().strip()
            
            # Check if first line is a header
            has_header = not first_line[0].isdigit() and not first_line[0] == '-'
            
            # Determine delimiter
            if '\t' in first_line:
                delimiter = '\t'
            elif ',' in first_line:
                delimiter = ','
            else:
                # Try whitespace
                delimiter = None
            
            # Load data
            if has_header:
                data = np.loadtxt(self.file_path, delimiter=delimiter, skiprows=1)
            else:
                data = np.loadtxt(self.file_path, delimiter=delimiter)
            
            # Validate shape
            if data.ndim != 2 or data.shape[1] < 5:
                raise ValueError(
                    f"File must have at least 5 columns (time + 4 fins). "
                    f"Got shape {data.shape}.\n"
                    f"يجب أن يحتوي الملف على 5 أعمدة على الأقل (الزمن + 4 زعانف). "
                    f"الشكل الحالي {data.shape}."
                )
            
            # Extract time and fin deflections
            self.time_points = data[:, 0] + self.time_offset
            self.fin_deflections = data[:, 1:5]  # Take only first 4 fin columns
            
            # Convert to radians if needed
            if self.units == 'degrees':
                self.fin_deflections = np.radians(self.fin_deflections)
            
            # Validate time is monotonically increasing
            if not np.all(np.diff(self.time_points) > 0):
                # Try to handle duplicate times by keeping first occurrence
                unique_mask = np.concatenate([[True], np.diff(self.time_points) > 0])
                if np.sum(unique_mask) < 2:
                    raise ValueError(
                        "Time points must be monotonically increasing.\n"
                        "يجب أن تكون نقاط الزمن متزايدة بشكل رتيب."
                    )
                logger.warning(
                    f"Found {len(self.time_points) - np.sum(unique_mask)} duplicate/non-increasing "
                    f"time points. Keeping first occurrences only."
                )
                self.time_points = self.time_points[unique_mask]
                self.fin_deflections = self.fin_deflections[unique_mask]
            
            # Store statistics
            self.num_points = len(self.time_points)
            self.time_start = self.time_points[0]
            self.time_end = self.time_points[-1]
            self.dt_mean = np.mean(np.diff(self.time_points))
            self.dt_min = np.min(np.diff(self.time_points))
            self.dt_max = np.max(np.diff(self.time_points))
            
        except Exception as e:
            raise ValueError(
                f"Failed to load fin deflection file: {e}\n"
                f"فشل تحميل ملف انحراف الزعانف: {e}"
            )
    
    def _create_interpolators(self):
        """
        Create interpolators for each fin channel.
        """
        self.interpolators = []
        
        for i in range(4):
            interp = interpolate.interp1d(
                self.time_points,
                self.fin_deflections[:, i],
                kind='linear',
                bounds_error=False,
                fill_value=(self.fin_deflections[0, i], self.fin_deflections[-1, i])
            )
            self.interpolators.append(interp)
    
    def get_deflections(self, t: float) -> np.ndarray:
        """الحصول على انحرافات الزعانف الأربع بالراديان عند الزمن t."""
        deflections = np.array([
            self.interpolators[i](t) for i in range(4)
        ])
        return deflections
    
    def get_deflections_array(self, times: np.ndarray) -> np.ndarray:
        """الحصول على انحرافات الزعانف لمصفوفة أوقات. الناتج: (len(times), 4)."""
        deflections = np.zeros((len(times), 4))
        for i in range(4):
            deflections[:, i] = self.interpolators[i](times)
        return deflections
    
    def get_info(self) -> dict:
        """معلومات حول الملف المحمّل."""
        return {
            'file_path': str(self.file_path),
            'num_points': self.num_points,
            'time_start': self.time_start,
            'time_end': self.time_end,
            'duration': self.time_end - self.time_start,
            'dt_mean': self.dt_mean,
            'dt_min': self.dt_min,
            'dt_max': self.dt_max,
            'frequency_hz': 1.0 / self.dt_mean if self.dt_mean > 0 else 0,
            'units_original': self.units,
            'time_offset': self.time_offset,
            'fin_deflection_range_rad': {
                'fin1': (float(np.min(self.fin_deflections[:, 0])), 
                        float(np.max(self.fin_deflections[:, 0]))),
                'fin2': (float(np.min(self.fin_deflections[:, 1])), 
                        float(np.max(self.fin_deflections[:, 1]))),
                'fin3': (float(np.min(self.fin_deflections[:, 2])), 
                        float(np.max(self.fin_deflections[:, 2]))),
                'fin4': (float(np.min(self.fin_deflections[:, 3])), 
                        float(np.max(self.fin_deflections[:, 3]))),
            }
        }
    
    def __repr__(self) -> str:
        return (
            f"FinDeflectionProfile(file='{self.file_path.name}', "
            f"points={self.num_points}, "
            f"time=[{self.time_start:.3f}, {self.time_end:.3f}]s)"
        )


def load_fin_deflection_profile(config: dict, base_path: Optional[Path] = None) -> Optional[FinDeflectionProfile]:
    """تحميل ملف انحراف الزعانف من التكوين. يرجع None إذا لم يتم التكوين."""
    sim_config = config.get('simulation', {})
    
    if not sim_config.get('use_fin_deflection_file', False):
        return None
    
    file_path = sim_config.get('fin_deflection_file')
    if not file_path:
        logger.warning(
            "use_fin_deflection_file is True but fin_deflection_file is not specified. "
            "use_fin_deflection_file مفعّل لكن fin_deflection_file غير محدد."
        )
        return None
    
    # Resolve relative path
    file_path = Path(file_path)
    if not file_path.is_absolute() and base_path is not None:
        file_path = base_path / file_path
    
    units = sim_config.get('fin_deflection_units', 'degrees')
    time_offset = sim_config.get('fin_deflection_time_offset', 0.0)
    
    return FinDeflectionProfile(
        file_path=str(file_path),
        units=units,
        time_offset=time_offset
    )
