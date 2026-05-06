#!/usr/bin/env python3
"""
MSIS Atmosphere Table Generator
مولد جداول الغلاف الجوي MSIS

This tool generates atmosphere tables using the NRLMSISE-00/MSIS2.1 empirical
atmospheric model. The tables are compatible with the 6DOF rocket simulation.

هذه الأداة تولد جداول الغلاف الجوي باستخدام نموذج MSIS التجريبي.
الجداول متوافقة مع محاكاة الصواريخ 6DOF.

"""

import os
import sys
import copy
import argparse
import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any

import numpy as np
import pandas as pd
import yaml

try:
    import pymsis
    PYMSIS_AVAILABLE = True
    # Try to import Variable enum for version-safe column indexing
    # محاولة استيراد Variable enum للفهرسة الآمنة بين الإصدارات
    try:
        from pymsis import Variable as MSISVariable
        PYMSIS_VARIABLE_AVAILABLE = True
    except ImportError:
        MSISVariable = None
        PYMSIS_VARIABLE_AVAILABLE = False
        print("Warning: pymsis.Variable not available. Using hardcoded column indices.")
except ImportError:
    PYMSIS_AVAILABLE = False
    PYMSIS_VARIABLE_AVAILABLE = False
    MSISVariable = None
    print("Warning: pymsis not installed. Install with: pip install pymsis")

# Try to import HWM14 (Horizontal Wind Model 2014)
# محاولة استيراد نموذج الرياح الأفقية HWM14
PYHWM14_AVAILABLE = False
HWM14_IMPORT_ERROR = None
try:
    from pyhwm2014 import HWM14
    PYHWM14_AVAILABLE = True
except ImportError as e:
    HWM14_IMPORT_ERROR = str(e)


# Physical constants
# الثوابت الفيزيائية
R_GAS = 287.05  # Specific gas constant for dry air (J/kg/K) - used as fallback
GAMMA_DIATOMIC = 1.4     # Ratio of specific heats for diatomic gases (N2, O2)
GAMMA_MONATOMIC = 5.0/3.0  # Ratio of specific heats for monatomic gases (O, He, H, Ar, N)
M_AIR = 28.9644e-3  # Molar mass of air (kg/mol)
R_UNIVERSAL = 8.314462  # Universal gas constant (J/mol/K)
K_BOLTZMANN = 1.380649e-23  # Boltzmann constant (J/K)

# Species molecular masses (kg/mol)
# الكتل الجزيئية للأنواع (كجم/مول)
# MSIS output order: [Total mass density, N2, O2, O, He, H, Ar, N, Anomalous O, NO, Temperature]
SPECIES_MOLAR_MASS = {
    'N2': 28.0134e-3,   # Nitrogen molecule
    'O2': 31.9988e-3,   # Oxygen molecule
    'O': 15.9994e-3,    # Atomic oxygen
    'He': 4.0026e-3,    # Helium
    'H': 1.00794e-3,    # Hydrogen
    'Ar': 39.948e-3,    # Argon
    'N': 14.0067e-3,    # Atomic nitrogen
    'O_anom': 15.9994e-3,  # Anomalous oxygen (same as O)
    'NO': 30.0061e-3,   # Nitric oxide
}

# Species gamma values (Cp/Cv) at standard temperature
# قيم جاما للأنواع (Cp/Cv) عند درجة الحرارة القياسية
# Note: For diatomic molecules, gamma decreases at high temperatures due to
# vibrational excitation. Use get_temperature_dependent_gamma() for accurate
# values at high temperatures (>500 K).
# ملاحظة: للجزيئات ثنائية الذرة، تنخفض جاما عند درجات الحرارة العالية بسبب
# الإثارة الاهتزازية. استخدم get_temperature_dependent_gamma() للقيم الدقيقة
# عند درجات الحرارة العالية (>500 K).
SPECIES_GAMMA = {
    'N2': 1.4,      # Diatomic (at ~300 K)
    'O2': 1.4,      # Diatomic (at ~300 K)
    'O': 5.0/3.0,   # Monatomic (constant)
    'He': 5.0/3.0,  # Monatomic (constant)
    'H': 5.0/3.0,   # Monatomic (constant)
    'Ar': 5.0/3.0,  # Monatomic (constant)
    'N': 5.0/3.0,   # Monatomic (constant)
    'O_anom': 5.0/3.0,  # Monatomic (constant)
    'NO': 1.4,      # Diatomic (at ~300 K)
}

# Characteristic vibrational temperatures (K) for diatomic molecules
# درجات الحرارة الاهتزازية المميزة (K) للجزيئات ثنائية الذرة
# Used for temperature-dependent gamma calculation
# تُستخدم لحساب جاما المعتمد على درجة الحرارة
THETA_VIB = {
    'N2': 3374.0,   # Nitrogen vibrational temperature
    'O2': 2256.0,   # Oxygen vibrational temperature
    'NO': 2719.0,   # Nitric oxide vibrational temperature
}


def get_temperature_dependent_gamma(species: str, temperature: np.ndarray) -> np.ndarray:
    """
    Calculate temperature-dependent gamma (Cp/Cv) for a species.
    حساب جاما المعتمد على درجة الحرارة (Cp/Cv) لنوع معين.
    
    For monatomic gases, gamma is constant at 5/3.
    For diatomic gases, gamma decreases at high temperatures due to vibrational
    excitation. This uses the harmonic oscillator model:
    
    Cv_vib/R = (θ_v/T)² * exp(θ_v/T) / (exp(θ_v/T) - 1)²
    
    where θ_v is the characteristic vibrational temperature.
    
    للغازات أحادية الذرة، جاما ثابتة عند 5/3.
    للغازات ثنائية الذرة، تنخفض جاما عند درجات الحرارة العالية بسبب الإثارة الاهتزازية.
    
    Args:
        species: Species name (e.g., 'N2', 'O2', 'O', 'He')
        temperature: Array of temperatures in Kelvin
        
    Returns:
        Array of gamma values for each temperature
    """
    # Monatomic gases have constant gamma = 5/3
    # الغازات أحادية الذرة لها جاما ثابتة = 5/3
    if species in ['O', 'He', 'H', 'Ar', 'N', 'O_anom']:
        return np.full_like(temperature, 5.0/3.0)
    
    # Diatomic gases: calculate temperature-dependent gamma
    # الغازات ثنائية الذرة: حساب جاما المعتمد على درجة الحرارة
    if species not in THETA_VIB:
        # Unknown species, return standard diatomic gamma
        return np.full_like(temperature, 1.4)
    
    theta_v = THETA_VIB[species]
    
    # Avoid numerical issues at very low temperatures
    # تجنب المشاكل العددية عند درجات الحرارة المنخفضة جداً
    T_safe = np.maximum(temperature, 100.0)
    
    # Calculate x = θ_v / T
    x = theta_v / T_safe
    
    # Calculate vibrational contribution to Cv/R using harmonic oscillator model
    # حساب المساهمة الاهتزازية في Cv/R باستخدام نموذج المذبذب التوافقي
    # Cv_vib/R = x² * exp(x) / (exp(x) - 1)²
    # For numerical stability, use different formulas for large and small x
    # للاستقرار العددي، استخدم صيغ مختلفة لـ x الكبير والصغير
    
    exp_x = np.exp(np.minimum(x, 700))  # Prevent overflow
    exp_neg_x = np.exp(-np.minimum(x, 700))
    
    # Cv_vib/R = x² * exp(-x) / (1 - exp(-x))² for numerical stability
    denominator = (1.0 - exp_neg_x) ** 2
    denominator = np.maximum(denominator, 1e-30)  # Avoid division by zero
    cv_vib_over_R = x**2 * exp_neg_x / denominator
    
    # Total Cv/R for diatomic: translation (3/2) + rotation (1) + vibration
    # Cv/R الكلي للثنائية الذرة: انتقال (3/2) + دوران (1) + اهتزاز
    cv_over_R = 2.5 + cv_vib_over_R  # 3/2 + 2/2 + vib
    
    # Cp/R = Cv/R + 1 (ideal gas)
    cp_over_R = cv_over_R + 1.0
    
    # gamma = Cp/Cv
    gamma = cp_over_R / cv_over_R
    
    return gamma

# MSIS output column indices - hardcoded fallback values
# فهارس أعمدة مخرجات MSIS - قيم احتياطية ثابتة
# These indices are stable across pymsis versions and match the documented output order:
# هذه الفهارس ثابتة عبر إصدارات pymsis وتطابق ترتيب المخرجات الموثق:
# [Total mass density, N2, O2, O, He, H, Ar, N, Anomalous O, NO, Temperature]
_MSIS_COL_INDICES = {
    'MASS_DENSITY': 0,      # Total mass density (kg/m³)
    'N2': 1,                # N2 number density (m⁻³)
    'O2': 2,                # O2 number density (m⁻³)
    'O': 3,                 # O number density (m⁻³)
    'HE': 4,                # He number density (m⁻³)
    'H': 5,                 # H number density (m⁻³)
    'AR': 6,                # Ar number density (m⁻³)
    'N': 7,                 # N number density (m⁻³)
    'ANOMALOUS_O': 8,       # Anomalous O number density (m⁻³)
    'NO': 9,                # NO number density (m⁻³)
    'TEMPERATURE': 10,      # Temperature (K)
}

# Mapping from our internal keys to pymsis.Variable enum names
# تعيين من مفاتيحنا الداخلية إلى أسماء pymsis.Variable enum
# pymsis.Variable uses names like "Total mass density", "N2", "Temperature"
# pymsis.Variable يستخدم أسماء مثل "Total mass density"، "N2"، "Temperature"
_PYMSIS_VARIABLE_NAMES = {
    'MASS_DENSITY': 'Total mass density',
    'N2': 'N2',
    'O2': 'O2',
    'O': 'O',
    'HE': 'He',
    'H': 'H',
    'AR': 'Ar',
    'N': 'N',
    'ANOMALOUS_O': 'Anomalous oxygen',
    'NO': 'NO',
    'TEMPERATURE': 'Temperature',
}


def _get_msis_index(var_name: str) -> int:
    """
    Get the MSIS output column index for a variable.
    الحصول على فهرس عمود مخرجات MSIS لمتغير.
    
    This function first tries to use pymsis.Variable enum for version-safe indexing,
    then falls back to hardcoded indices if the enum is not available.
    
    هذه الدالة تحاول أولاً استخدام pymsis.Variable enum للفهرسة الآمنة بين الإصدارات،
    ثم ترجع إلى الفهارس الثابتة إذا لم يكن enum متاحاً.
    
    Args:
        var_name: Internal variable name (e.g., 'MASS_DENSITY', 'N2', 'TEMPERATURE')
        
    Returns:
        Column index for the variable
        
    Raises:
        KeyError: If var_name is not a known MSIS variable
    """
    # Validate the variable name first
    # التحقق من اسم المتغير أولاً
    if var_name not in _MSIS_COL_INDICES:
        raise KeyError(
            f"Unknown MSIS variable: '{var_name}'. "
            f"Valid variables are: {list(_MSIS_COL_INDICES.keys())}. "
            f"متغير MSIS غير معروف: '{var_name}'."
        )
    
    # Try to use pymsis.Variable enum for version-safe indexing
    # محاولة استخدام pymsis.Variable enum للفهرسة الآمنة بين الإصدارات
    if PYMSIS_VARIABLE_AVAILABLE and MSISVariable is not None:
        pymsis_name = _PYMSIS_VARIABLE_NAMES.get(var_name)
        if pymsis_name:
            try:
                # pymsis.Variable is an IntEnum, so we can use it directly as an index
                # pymsis.Variable هو IntEnum، لذلك يمكننا استخدامه مباشرة كفهرس
                return int(MSISVariable[pymsis_name])
            except (KeyError, ValueError, TypeError):
                # Variable not found in enum, or enum doesn't support int() conversion
                # المتغير غير موجود في enum، أو enum لا يدعم تحويل int()
                pass
    
    # Fall back to hardcoded indices
    # الرجوع إلى الفهارس الثابتة
    return _MSIS_COL_INDICES[var_name]


class MSISAtmosphereGenerator:
    """
    Generator for atmosphere tables using MSIS empirical model.
    مولد جداول الغلاف الجوي باستخدام نموذج MSIS التجريبي.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize the generator with configuration.
        تهيئة المولد بالتكوين.
        
        Args:
            config: Configuration dictionary from YAML file
        """
        self.config = config
        self.location = config.get('location', {})
        self.datetime_config = config.get('datetime', {})
        self.space_weather = config.get('space_weather', {})
        self.altitude_config = config.get('altitude', {})
        self.wind_config = config.get('wind', {})
        self.output_config = config.get('output', {})
        self.msis_config = config.get('msis', {})
        
    def generate_altitude_grid(self) -> np.ndarray:
        """
        Generate altitude grid based on configuration.
        توليد شبكة الارتفاع بناءً على التكوين.
        
        Returns:
            Array of altitudes in meters
        """
        # Read altitude settings from config
        # قراءة إعدادات الارتفاع من التكوين
        alt_min = self.altitude_config.get('min_m', 0.0)
        alt_max = self.altitude_config.get('max_m', 400000.0)
        num_points = self.altitude_config.get('num_points', 1000)
        spacing = self.altitude_config.get('spacing', 'logarithmic')
        
        # Debug: Print altitude configuration being used
        # تصحيح: طباعة تكوين الارتفاع المستخدم
        print(f"  Altitude config: min_m={alt_min}, max_m={alt_max}, num_points={num_points}, spacing={spacing}")
        
        # Note: We start from alt_min (default 0) regardless of ground_altitude_m
        # This allows the atmosphere table to cover all altitudes from sea level
        # ملاحظة: نبدأ من alt_min (الافتراضي 0) بغض النظر عن ارتفاع الأرض
        # هذا يسمح لجدول الغلاف الجوي بتغطية جميع الارتفاعات من مستوى سطح البحر
        
        if spacing == 'logarithmic':
            # Use logarithmic spacing for better resolution at low altitudes
            # استخدام تباعد لوغاريتمي لدقة أفضل في الارتفاعات المنخفضة
            alt_min_log = max(alt_min, 1.0)  # Avoid log(0)
            altitudes = np.logspace(
                np.log10(alt_min_log),
                np.log10(alt_max),
                num_points
            )
            # Ensure ground altitude is included
            if alt_min < alt_min_log:
                altitudes = np.insert(altitudes, 0, alt_min)
        else:
            # Linear spacing
            altitudes = np.linspace(alt_min, alt_max, num_points)
            
        return altitudes
    
    def get_datetime(self) -> Tuple[datetime.datetime, int]:
        """
        Get datetime and day of year from configuration.
        الحصول على التاريخ والوقت ورقم اليوم من التكوين.
        
        Returns:
            Tuple of (datetime object, day of year)
        """
        year = self.datetime_config.get('year', 2024)
        month = self.datetime_config.get('month', 6)
        day = self.datetime_config.get('day', 21)
        hour = self.datetime_config.get('hour', 12)
        
        dt = datetime.datetime(year, month, day, hour, 0, 0)
        doy = dt.timetuple().tm_yday
        
        return dt, doy
    
    def calculate_msis_atmosphere(self, altitudes: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Calculate atmospheric properties using MSIS model with composition-aware
        pressure and speed of sound calculations.
        
        حساب خصائص الغلاف الجوي باستخدام نموذج MSIS مع حسابات الضغط وسرعة الصوت
        المدركة للتركيب الكيميائي.
        
        Args:
            altitudes: Array of altitudes in meters
            
        Returns:
            Dictionary with atmospheric properties including composition-aware
            pressure and speed of sound that account for varying molecular mass
            and gamma at high altitudes.
        """
        if not PYMSIS_AVAILABLE:
            raise ImportError("pymsis is required for MSIS calculations. Install with: pip install pymsis")
        
        lat = self.location.get('latitude', 0.0)
        lon = self.location.get('longitude', 0.0)
        
        dt, doy = self.get_datetime()
        
        f107_val = self.space_weather.get('f107', 150.0)
        f107a_val = self.space_weather.get('f107a', 150.0)
        ap_val = self.space_weather.get('ap', 15.0)
        
        # Convert altitudes to km for MSIS
        altitudes_km = altitudes / 1000.0
        
        # Create input arrays for pymsis (all arrays must have same length)
        n_points = len(altitudes)
        dates = np.array([dt] * n_points)
        lons = np.full(n_points, lon)
        lats = np.full(n_points, lat)
        f107s = np.full(n_points, f107_val)
        f107as = np.full(n_points, f107a_val)
        
        # Build Ap array: (n_points, 7) for the 7 Ap indices
        # مصفوفة Ap: (n_points, 7) لـ 7 مؤشرات Ap
        # Indices: [Daily Ap, 3hr current, 3hr-3h, 3hr-6h, 3hr-9h, avg 12-33h, avg 36-57h]
        # المؤشرات: [Ap اليومي، 3 ساعات الحالي، 3 ساعات-3س، 3 ساعات-6س، 3 ساعات-9س، متوسط 12-33س، متوسط 36-57س]
        storm_time_config = self.space_weather.get('storm_time_ap', {})
        storm_time_enabled = storm_time_config.get('enabled', False)
        
        if storm_time_enabled:
            # Use detailed Ap history for storm-time calculations
            # استخدام تاريخ Ap المفصل لحسابات وقت العاصفة
            ap_history = storm_time_config.get('ap_history', [ap_val] * 7)
            if len(ap_history) != 7:
                print(f"  Warning: ap_history should have 7 values, got {len(ap_history)}. Padding/truncating.")
                print(f"  تحذير: ap_history يجب أن يحتوي على 7 قيم، حصلنا على {len(ap_history)}. جاري التعبئة/الاقتطاع.")
                ap_history = (ap_history + [ap_val] * 7)[:7]
            aps = np.tile(np.array(ap_history), (n_points, 1))
            print(f"  Storm-time Ap mode enabled with history: {ap_history}")
            print(f"  وضع العاصفة Ap مفعّل مع التاريخ: {ap_history}")
        else:
            # Default: fill all 7 indices with the same Ap value
            # الافتراضي: ملء جميع المؤشرات السبعة بنفس قيمة Ap
            aps = np.full((n_points, 7), ap_val)
        
        # Select MSIS version
        version = self.msis_config.get('version', 'msis2.1')
        
        # Call MSIS using pymsis.calculate() - the official API that works with pymsis.Variable
        # استدعاء MSIS باستخدام pymsis.calculate() - الواجهة الرسمية التي تعمل مع pymsis.Variable
        # Output shape: (n_points, 11) for MSIS2.1
        # Columns: [Total mass density (kg/m³), N2, O2, O, He, H, Ar, N, Anomalous O, NO, Temperature (K)]
        # Number densities are in m⁻³
        # 
        # Note on geomagnetic_activity option:
        # ملاحظة حول خيار geomagnetic_activity:
        # - geomagnetic_activity=1 (default): Uses daily Ap (aps[0]) only
        # - geomagnetic_activity=-1: Uses full 7-element Ap history for storm-time calculations
        # - geomagnetic_activity=1 (الافتراضي): يستخدم Ap اليومي (aps[0]) فقط
        # - geomagnetic_activity=-1: يستخدم تاريخ Ap الكامل ذو 7 عناصر لحسابات وقت العاصفة
        try:
            if version == 'nrlmsise00':
                if storm_time_enabled:
                    output = pymsis.calculate(
                        dates, lons, lats, altitudes_km,
                        f107s=f107s, f107as=f107as, aps=aps,
                        version=0,  # NRLMSISE-00 is version 0
                        geomagnetic_activity=-1  # Enable storm-time Ap mode
                    )
                else:
                    output = pymsis.calculate(
                        dates, lons, lats, altitudes_km,
                        f107s=f107s, f107as=f107as, aps=aps,
                        version=0  # NRLMSISE-00 is version 0
                    )
            else:
                ver_num = 2.1 if version == 'msis2.1' else 2.0
                if storm_time_enabled:
                    output = pymsis.calculate(
                        dates, lons, lats, altitudes_km,
                        f107s=f107s, f107as=f107as, aps=aps,
                        version=ver_num,
                        geomagnetic_activity=-1  # Enable storm-time Ap mode
                    )
                else:
                    output = pymsis.calculate(
                        dates, lons, lats, altitudes_km,
                        f107s=f107s, f107as=f107as, aps=aps,
                        version=ver_num
                    )
        except Exception as e:
            print(f"Error calling pymsis.calculate: {e}")
            print("Falling back to NRLMSISE-00...")
            output = pymsis.calculate(
                dates, lons, lats, altitudes_km,
                f107s=f107s, f107as=f107as, aps=aps,
                version=0
            )
        
        # Log which indexing method is being used
        if PYMSIS_VARIABLE_AVAILABLE:
            print("  Using pymsis.Variable enum for column indexing (version-safe)")
            print("  استخدام pymsis.Variable enum للفهرسة (آمن بين الإصدارات)")
        else:
            print("  Using fallback hardcoded column indices")
            print("  استخدام فهارس الأعمدة الثابتة كاحتياطي")
        
        # Extract density and temperature using version-safe indexing
        # استخراج الكثافة ودرجة الحرارة باستخدام فهرسة آمنة بين الإصدارات
        density = output[..., _get_msis_index('MASS_DENSITY')]  # Total mass density (kg/m³)
        temperature = output[..., _get_msis_index('TEMPERATURE')]  # Temperature (K)
        
        # Extract species number densities (m⁻³) using version-safe indexing
        # استخراج كثافات الأنواع (م⁻³) باستخدام فهرسة آمنة بين الإصدارات
        n_N2 = output[..., _get_msis_index('N2')]
        n_O2 = output[..., _get_msis_index('O2')]
        n_O = output[..., _get_msis_index('O')]
        n_He = output[..., _get_msis_index('HE')]
        n_H = output[..., _get_msis_index('H')]
        n_Ar = output[..., _get_msis_index('AR')]
        n_N = output[..., _get_msis_index('N')]
        n_O_anom = output[..., _get_msis_index('ANOMALOUS_O')]
        n_NO = output[..., _get_msis_index('NO')]
        
        # Handle any NaN or invalid values in density and temperature
        nan_count_density = np.sum(np.isnan(density))
        nan_count_temp = np.sum(np.isnan(temperature))
        if nan_count_density > 0 or nan_count_temp > 0:
            print(f"  Warning: Found {nan_count_density} NaN density values and {nan_count_temp} NaN temperature values. Replacing with defaults.")
            print(f"  تحذير: تم العثور على {nan_count_density} قيم كثافة NaN و {nan_count_temp} قيم درجة حرارة NaN. استبدالها بالقيم الافتراضية.")
        
        density = np.nan_to_num(density, nan=1e-20, posinf=1e-20, neginf=1e-20)
        temperature = np.nan_to_num(temperature, nan=200.0, posinf=2000.0, neginf=200.0)
        
        # Handle NaN in species number densities with warning
        # معالجة قيم NaN في كثافات الأنواع مع تحذير
        species_nan_counts = {
            'N2': np.sum(np.isnan(n_N2)),
            'O2': np.sum(np.isnan(n_O2)),
            'O': np.sum(np.isnan(n_O)),
            'He': np.sum(np.isnan(n_He)),
            'H': np.sum(np.isnan(n_H)),
            'Ar': np.sum(np.isnan(n_Ar)),
            'N': np.sum(np.isnan(n_N)),
            'O_anom': np.sum(np.isnan(n_O_anom)),
            'NO': np.sum(np.isnan(n_NO)),
        }
        total_species_nan = sum(species_nan_counts.values())
        if total_species_nan > 0:
            nan_details = ', '.join(f"{k}:{v}" for k, v in species_nan_counts.items() if v > 0)
            print(f"  Warning: Found {total_species_nan} NaN values in species number densities ({nan_details}). Replacing with 0.0.")
            print(f"  تحذير: تم العثور على {total_species_nan} قيم NaN في كثافات الأنواع ({nan_details}). استبدالها بـ 0.0.")
        
        n_N2 = np.nan_to_num(n_N2, nan=0.0, posinf=0.0, neginf=0.0)
        n_O2 = np.nan_to_num(n_O2, nan=0.0, posinf=0.0, neginf=0.0)
        n_O = np.nan_to_num(n_O, nan=0.0, posinf=0.0, neginf=0.0)
        n_He = np.nan_to_num(n_He, nan=0.0, posinf=0.0, neginf=0.0)
        n_H = np.nan_to_num(n_H, nan=0.0, posinf=0.0, neginf=0.0)
        n_Ar = np.nan_to_num(n_Ar, nan=0.0, posinf=0.0, neginf=0.0)
        n_N = np.nan_to_num(n_N, nan=0.0, posinf=0.0, neginf=0.0)
        n_O_anom = np.nan_to_num(n_O_anom, nan=0.0, posinf=0.0, neginf=0.0)
        n_NO = np.nan_to_num(n_NO, nan=0.0, posinf=0.0, neginf=0.0)
        
        # Ensure minimum values
        # Use 1e-20 kg/m³ as minimum to support extreme altitudes (>1000 km)
        # MSIS is valid up to ~2500 km where density can be ~1e-17 kg/m³
        # استخدام 1e-20 كجم/م³ كحد أدنى لدعم الارتفاعات القصوى (>1000 كم)
        density = np.maximum(density, 1e-20)
        temperature = np.maximum(temperature, 100.0)
        
        # Calculate total number density (m⁻³)
        # N_total = sum of all species number densities
        n_total = n_N2 + n_O2 + n_O + n_He + n_H + n_Ar + n_N + n_O_anom + n_NO
        
        # Calculate pressure using ideal gas law: P = N * k_B * T
        # where N is total number density (m⁻³), k_B is Boltzmann constant (J/K)
        # This is physically correct regardless of composition
        # حساب الضغط باستخدام قانون الغاز المثالي: P = N * k_B * T
        pressure = n_total * K_BOLTZMANN * temperature
        
        # For very low altitudes where n_total might be zero (shouldn't happen but safety check),
        # fall back to using density with composition-aware R_specific
        # للارتفاعات المنخفضة جداً حيث قد يكون n_total صفراً، نرجع لاستخدام الكثافة مع R_specific المدرك للتركيب
        # Note: We calculate M_mean first (below) and use it for the fallback
        # This is more accurate than using fixed R_GAS for dry air
        # ملاحظة: نحسب M_mean أولاً (أدناه) ونستخدمه للرجوع
        # هذا أكثر دقة من استخدام R_GAS الثابت للهواء الجاف
        low_n_mask = n_total < 1e10  # Very low number density threshold
        
        # Calculate mean molecular mass for each altitude point
        # M_mean = (sum of n_i * M_i) / N_total = rho / N_total * N_A
        # Or equivalently: M_mean = rho * N_A / N_total where N_A is Avogadro's number
        # حساب الكتلة الجزيئية المتوسطة لكل نقطة ارتفاع
        N_AVOGADRO = 6.02214076e23  # Avogadro's number (mol⁻¹)
        
        # Calculate mean molecular mass (kg/mol) from composition
        # M_mean = sum(n_i * M_i) / sum(n_i)
        mass_weighted_sum = (
            n_N2 * SPECIES_MOLAR_MASS['N2'] +
            n_O2 * SPECIES_MOLAR_MASS['O2'] +
            n_O * SPECIES_MOLAR_MASS['O'] +
            n_He * SPECIES_MOLAR_MASS['He'] +
            n_H * SPECIES_MOLAR_MASS['H'] +
            n_Ar * SPECIES_MOLAR_MASS['Ar'] +
            n_N * SPECIES_MOLAR_MASS['N'] +
            n_O_anom * SPECIES_MOLAR_MASS['O_anom'] +
            n_NO * SPECIES_MOLAR_MASS['NO']
        )
        
        # Mean molecular mass (kg/mol)
        M_mean = np.where(n_total > 1e10, mass_weighted_sum / n_total, M_AIR)
        
        # Calculate composition-aware gamma (Cp/Cv) using heat capacity mixing
        # The correct method is to compute cp_mix and cv_mix from mass fractions,
        # then gamma_mix = cp_mix / cv_mix
        # حساب جاما المدرك للتركيب (Cp/Cv) باستخدام خلط السعات الحرارية
        # الطريقة الصحيحة هي حساب cp_mix و cv_mix من الكسور الكتلية
        
        # Calculate mass density for each species: rho_i = n_i * M_i / N_A
        # حساب الكثافة الكتلية لكل نوع: rho_i = n_i * M_i / N_A
        # N_AVOGADRO already defined above
        
        rho_N2 = n_N2 * SPECIES_MOLAR_MASS['N2'] / N_AVOGADRO
        rho_O2 = n_O2 * SPECIES_MOLAR_MASS['O2'] / N_AVOGADRO
        rho_O = n_O * SPECIES_MOLAR_MASS['O'] / N_AVOGADRO
        rho_He = n_He * SPECIES_MOLAR_MASS['He'] / N_AVOGADRO
        rho_H = n_H * SPECIES_MOLAR_MASS['H'] / N_AVOGADRO
        rho_Ar = n_Ar * SPECIES_MOLAR_MASS['Ar'] / N_AVOGADRO
        rho_N = n_N * SPECIES_MOLAR_MASS['N'] / N_AVOGADRO
        rho_O_anom = n_O_anom * SPECIES_MOLAR_MASS['O_anom'] / N_AVOGADRO
        rho_NO = n_NO * SPECIES_MOLAR_MASS['NO'] / N_AVOGADRO
        
        # Total mass density from species (for mass fraction calculation)
        # الكثافة الكتلية الكلية من الأنواع (لحساب الكسر الكتلي)
        rho_total_species = rho_N2 + rho_O2 + rho_O + rho_He + rho_H + rho_Ar + rho_N + rho_O_anom + rho_NO
        
        # Avoid division by zero
        rho_total_species = np.maximum(rho_total_species, 1e-30)
        
        # Calculate specific gas constant for each species: R_i = R_universal / M_i
        # حساب ثابت الغاز النوعي لكل نوع: R_i = R_universal / M_i
        R_N2 = R_UNIVERSAL / SPECIES_MOLAR_MASS['N2']
        R_O2 = R_UNIVERSAL / SPECIES_MOLAR_MASS['O2']
        R_O = R_UNIVERSAL / SPECIES_MOLAR_MASS['O']
        R_He = R_UNIVERSAL / SPECIES_MOLAR_MASS['He']
        R_H = R_UNIVERSAL / SPECIES_MOLAR_MASS['H']
        R_Ar = R_UNIVERSAL / SPECIES_MOLAR_MASS['Ar']
        R_N = R_UNIVERSAL / SPECIES_MOLAR_MASS['N']
        R_O_anom = R_UNIVERSAL / SPECIES_MOLAR_MASS['O_anom']
        R_NO = R_UNIVERSAL / SPECIES_MOLAR_MASS['NO']
        
        # Calculate specific heat capacities for each species (per unit mass)
        # cp_i = gamma_i / (gamma_i - 1) * R_i
        # cv_i = 1 / (gamma_i - 1) * R_i
        # حساب السعات الحرارية النوعية لكل نوع (لكل وحدة كتلة)
        # 
        # Note: For diatomic molecules (N2, O2, NO), we use temperature-dependent
        # gamma to account for vibrational excitation at high temperatures (>500 K).
        # This improves accuracy in the thermosphere where T can exceed 1000 K.
        # ملاحظة: للجزيئات ثنائية الذرة (N2, O2, NO)، نستخدم جاما المعتمد على
        # درجة الحرارة لمراعاة الإثارة الاهتزازية عند درجات الحرارة العالية (>500 K).
        # هذا يحسن الدقة في الثرموسفير حيث يمكن أن تتجاوز T 1000 K.
        def calc_cp_cv(gamma, R):
            cp = gamma / (gamma - 1) * R
            cv = 1 / (gamma - 1) * R
            return cp, cv
        
        # Get temperature-dependent gamma for diatomic species
        # الحصول على جاما المعتمد على درجة الحرارة للأنواع ثنائية الذرة
        gamma_N2 = get_temperature_dependent_gamma('N2', temperature)
        gamma_O2 = get_temperature_dependent_gamma('O2', temperature)
        gamma_NO = get_temperature_dependent_gamma('NO', temperature)
        
        # Monatomic species have constant gamma = 5/3
        # الأنواع أحادية الذرة لها جاما ثابتة = 5/3
        gamma_monatomic = 5.0 / 3.0
        
        # Calculate cp and cv for each species
        # حساب cp و cv لكل نوع
        cp_N2 = gamma_N2 / (gamma_N2 - 1) * R_N2
        cv_N2 = 1 / (gamma_N2 - 1) * R_N2
        cp_O2 = gamma_O2 / (gamma_O2 - 1) * R_O2
        cv_O2 = 1 / (gamma_O2 - 1) * R_O2
        cp_NO = gamma_NO / (gamma_NO - 1) * R_NO
        cv_NO = 1 / (gamma_NO - 1) * R_NO
        
        # Monatomic species (constant gamma)
        cp_O, cv_O = calc_cp_cv(gamma_monatomic, R_O)
        cp_He, cv_He = calc_cp_cv(gamma_monatomic, R_He)
        cp_H, cv_H = calc_cp_cv(gamma_monatomic, R_H)
        cp_Ar, cv_Ar = calc_cp_cv(gamma_monatomic, R_Ar)
        cp_N, cv_N = calc_cp_cv(gamma_monatomic, R_N)
        cp_O_anom, cv_O_anom = calc_cp_cv(gamma_monatomic, R_O_anom)
        
        # Calculate mixture heat capacities using mass fraction weighting
        # cp_mix = sum(w_i * cp_i) where w_i = rho_i / rho_total
        # حساب السعات الحرارية للخليط باستخدام ترجيح الكسر الكتلي
        cp_mix = (
            rho_N2 * cp_N2 + rho_O2 * cp_O2 + rho_O * cp_O +
            rho_He * cp_He + rho_H * cp_H + rho_Ar * cp_Ar +
            rho_N * cp_N + rho_O_anom * cp_O_anom + rho_NO * cp_NO
        ) / rho_total_species
        
        cv_mix = (
            rho_N2 * cv_N2 + rho_O2 * cv_O2 + rho_O * cv_O +
            rho_He * cv_He + rho_H * cv_H + rho_Ar * cv_Ar +
            rho_N * cv_N + rho_O_anom * cv_O_anom + rho_NO * cv_NO
        ) / rho_total_species
        
        # Calculate gamma_mix = cp_mix / cv_mix
        # حساب gamma_mix = cp_mix / cv_mix
        gamma_mean = np.where(n_total > 1e10, cp_mix / cv_mix, GAMMA_DIATOMIC)
        
        # Calculate specific gas constant: R_specific = R_universal / M_mean
        # حساب ثابت الغاز النوعي: R_specific = R_universal / M_mean
        R_specific = R_UNIVERSAL / M_mean
        
        # Apply composition-aware fallback for pressure where n_total is very low
        # تطبيق الرجوع المدرك للتركيب للضغط حيث n_total منخفض جداً
        # Now that we have R_specific, use it instead of fixed R_GAS
        # الآن بعد أن لدينا R_specific، نستخدمه بدلاً من R_GAS الثابت
        if np.any(low_n_mask):
            # Use composition-aware R_specific for fallback (more accurate than fixed R_GAS)
            # استخدام R_specific المدرك للتركيب للرجوع (أكثر دقة من R_GAS الثابت)
            pressure[low_n_mask] = density[low_n_mask] * R_specific[low_n_mask] * temperature[low_n_mask]
            print(f"  Note: Applied composition-aware pressure fallback for {np.sum(low_n_mask)} points with low number density")
            print(f"  ملاحظة: تم تطبيق الرجوع المدرك للتركيب للضغط لـ {np.sum(low_n_mask)} نقطة ذات كثافة أعداد منخفضة")
        
        # Internal consistency check: verify MSIS total density matches species sum
        # فحص التناسق الداخلي: التحقق من أن كثافة MSIS الكلية تطابق مجموع الأنواع
        density_ratio = np.where(density > 1e-25, rho_total_species / density, 1.0)
        mean_ratio = np.mean(density_ratio[density > 1e-25]) if np.any(density > 1e-25) else 1.0
        max_deviation = np.max(np.abs(density_ratio - 1.0)) if len(density_ratio) > 0 else 0.0
        
        if max_deviation > 0.1:  # More than 10% deviation
            print(f"  Warning: MSIS total density differs from species sum by up to {max_deviation*100:.1f}%")
            print(f"  تحذير: كثافة MSIS الكلية تختلف عن مجموع الأنواع بنسبة تصل إلى {max_deviation*100:.1f}%")
            print(f"  Mean ratio (species/MSIS): {mean_ratio:.4f}")
            print(f"  متوسط النسبة (الأنواع/MSIS): {mean_ratio:.4f}")
        else:
            print(f"  Internal consistency check passed: species density matches MSIS total (mean ratio: {mean_ratio:.4f})")
            print(f"  فحص التناسق الداخلي ناجح: كثافة الأنواع تطابق MSIS الكلية (متوسط النسبة: {mean_ratio:.4f})")
        
        # Calculate speed of sound: a = sqrt(gamma * R_specific * T)
        # حساب سرعة الصوت: a = sqrt(gamma * R_specific * T)
        # 
        # IMPORTANT PHYSICAL LIMITATION / قيد فيزيائي مهم:
        # ================================================
        # The concept of "speed of sound" loses its physical meaning in rarefied flow
        # (Knudsen number Kn > 0.01) which occurs above approximately 100 km altitude.
        # In the continuum regime (Kn < 0.01), sound propagates as pressure waves through
        # molecular collisions. In rarefied flow, the mean free path becomes comparable
        # to or larger than the characteristic length scale, and the continuum assumption
        # breaks down.
        # 
        # مفهوم "سرعة الصوت" يفقد معناه الفيزيائي في التدفق المخلخل
        # (عدد كنودسن Kn > 0.01) الذي يحدث فوق ارتفاع ~100 كم تقريباً.
        # في نظام الاستمرارية (Kn < 0.01)، ينتشر الصوت كموجات ضغط عبر
        # تصادمات الجزيئات. في التدفق المخلخل، يصبح المسار الحر المتوسط مماثلاً
        # أو أكبر من مقياس الطول المميز، وينهار افتراض الاستمرارية.
        # 
        # The values calculated here above ~100 km are provided for:
        # - Continuity of the data table (no discontinuities)
        # - Reference calculations (e.g., thermal velocity scaling)
        # - Simulation codes that may use this value for other purposes
        # 
        # القيم المحسوبة هنا فوق ~100 كم مقدمة لـ:
        # - استمرارية جدول البيانات (بدون انقطاعات)
        # - حسابات مرجعية (مثل تدريج السرعة الحرارية)
        # - أكواد المحاكاة التي قد تستخدم هذه القيمة لأغراض أخرى
        # 
        # For aerodynamic calculations above 100 km, use free molecular flow models
        # instead of continuum aerodynamics.
        # للحسابات الديناميكية الهوائية فوق 100 كم، استخدم نماذج التدفق الجزيئي الحر
        # بدلاً من الديناميكا الهوائية الاستمرارية.
        speed_of_sound = np.sqrt(gamma_mean * R_specific * temperature)
        
        # Log composition statistics at key altitudes
        print(f"  Composition-aware calculations enabled:")
        print(f"  حسابات مدركة للتركيب مفعّلة:")
        
        # Find indices for key altitudes (0, 100km, 200km, 400km)
        key_alts = [0, 100000, 200000, 400000]
        for alt in key_alts:
            idx = np.argmin(np.abs(altitudes - alt))
            if altitudes[idx] <= altitudes[-1]:
                print(f"    At {altitudes[idx]/1000:.0f} km: M_mean={M_mean[idx]*1000:.2f} g/mol, "
                      f"gamma={gamma_mean[idx]:.3f}, R_specific={R_specific[idx]:.1f} J/kg/K")
        
        return {
            'altitude_m': altitudes,
            'pressure_Pa': pressure,
            'temperature_K': temperature,
            'density_kg_m3': density,
            'speed_of_sound_m_s': speed_of_sound
        }
    
    def calculate_wind(self, altitudes: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Calculate wind components based on configuration.
        حساب مكونات الرياح بناءً على التكوين.
        
        Args:
            altitudes: Array of altitudes in meters
            
        Returns:
            Dictionary with wind components (north, east, down)
        """
        n_points = len(altitudes)
        
        if not self.wind_config.get('enabled', True):
            return {
                'wind_north_m_s': np.zeros(n_points),
                'wind_east_m_s': np.zeros(n_points),
                'wind_down_m_s': np.zeros(n_points)
            }
        
        model = self.wind_config.get('model', 'simple')
        
        if model == 'none':
            return {
                'wind_north_m_s': np.zeros(n_points),
                'wind_east_m_s': np.zeros(n_points),
                'wind_down_m_s': np.zeros(n_points)
            }
        
        elif model == 'simple':
            return self._calculate_simple_wind(altitudes)
        
        elif model == 'hwm14':
            return self._calculate_hwm14_wind(altitudes)
        
        else:
            print(f"Warning: Unknown wind model '{model}', using simple")
            return self._calculate_simple_wind(altitudes)
    
    def _calculate_simple_wind(self, altitudes: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Calculate wind using simple altitude-dependent profile.
        حساب الرياح باستخدام ملف تعريف بسيط يعتمد على الارتفاع.
        
        This model includes:
        - Surface boundary layer wind (logarithmic profile)
        - Jet stream at ~12 km
        - Exponential decay above jet stream
        - Negligible wind above 100 km
        
        IMPORTANT LIMITATIONS / قيود مهمة:
        =========================================
        This is a HEURISTIC model, NOT based on validated physical data.
        هذا نموذج تقريبي، وليس مبنياً على بيانات فيزيائية محققة.
        
        - The wind profile is a simplified approximation and does NOT represent
          actual meteorological conditions at any specific location or time.
        - ملف الرياح هو تقريب مبسط ولا يمثل الظروف الجوية الفعلية
          في أي موقع أو وقت محدد.
        
        - For accurate wind modeling, use the 'hwm14' model which is based on
          empirical satellite and ground-based measurements.
        - للحصول على نمذجة رياح دقيقة، استخدم نموذج 'hwm14' المبني على
          قياسات تجريبية من الأقمار الصناعية والأرض.
        
        - This model is suitable for:
          * Initial trajectory estimates where wind effects are secondary
          * Sensitivity studies to understand wind impact
          * Cases where "reasonable" wind is needed, not meteorological accuracy
        - هذا النموذج مناسب لـ:
          * تقديرات المسار الأولية حيث تأثيرات الرياح ثانوية
          * دراسات الحساسية لفهم تأثير الرياح
          * الحالات التي تحتاج "رياح معقولة"، وليس دقة أرصاد جوية
        
        Args:
            altitudes: Array of altitudes in meters
            
        Returns:
            Dictionary with wind components
        """
        simple_config = self.wind_config.get('simple', {})
        
        surface_speed = simple_config.get('surface_speed', 5.0)
        surface_dir = np.radians(simple_config.get('surface_direction', 45.0))
        jet_alt = simple_config.get('jet_stream_altitude', 12000.0)
        jet_speed = simple_config.get('jet_stream_speed', 30.0)
        cutoff_alt = simple_config.get('wind_cutoff_altitude', 100000.0)
        
        n_points = len(altitudes)
        wind_speed = np.zeros(n_points)
        wind_dir = np.full(n_points, surface_dir)
        
        for i, alt in enumerate(altitudes):
            if alt < 1000:
                # Boundary layer: logarithmic profile
                wind_speed[i] = surface_speed * (1 + 0.5 * np.log1p(alt / 100))
            elif alt < jet_alt:
                # Increase to jet stream
                frac = (alt - 1000) / (jet_alt - 1000)
                wind_speed[i] = surface_speed * 2 + frac * (jet_speed - surface_speed * 2)
                # Wind direction rotates with altitude (Ekman spiral effect)
                wind_dir[i] = surface_dir + frac * np.radians(30)
            elif alt < cutoff_alt:
                # Exponential decay above jet stream
                scale_height = 20000.0  # 20 km scale height
                wind_speed[i] = jet_speed * np.exp(-(alt - jet_alt) / scale_height)
                wind_dir[i] = surface_dir + np.radians(30)
            else:
                # Negligible wind in thermosphere/exosphere
                wind_speed[i] = 0.0
        
        # Convert to north/east components
        wind_north = wind_speed * np.cos(wind_dir)
        wind_east = wind_speed * np.sin(wind_dir)
        wind_down = np.zeros(n_points)  # Vertical wind typically negligible
        
        return {
            'wind_north_m_s': wind_north,
            'wind_east_m_s': wind_east,
            'wind_down_m_s': wind_down
        }
    
    def _calculate_hwm14_wind(self, altitudes: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Calculate wind using HWM14 (Horizontal Wind Model 2014).
        حساب الرياح باستخدام نموذج الرياح الأفقية HWM14.
        
        HWM14 is an empirical model developed by the Naval Research Laboratory
        based on satellite and ground-based wind measurements. It provides
        realistic horizontal wind profiles from 0-500 km altitude.
        
        HWM14 هو نموذج تجريبي طورته مختبرات البحث البحرية
        بناءً على قياسات الرياح من الأقمار الصناعية والأرض.
        يوفر ملفات رياح أفقية واقعية من 0-500 كم ارتفاع.
        
        Args:
            altitudes: Array of altitudes in meters
            
        Returns:
            Dictionary with wind components (north, east, down)
            
        Raises:
            ImportError: If pyhwm2014 is not installed and fallback is disabled
        """
        n_points = len(altitudes)
        hwm14_config = self.wind_config.get('hwm14', {})
        fallback_to_simple = hwm14_config.get('fallback_to_simple', True)
        
        # Check if HWM14 is available
        # التحقق من توفر HWM14
        if not PYHWM14_AVAILABLE:
            if fallback_to_simple:
                print(f"Warning: pyhwm2014 not available ({HWM14_IMPORT_ERROR})")
                print("  Falling back to simple wind model.")
                print("  To use HWM14, install with: pip install pyhwm2014")
                print("  Note: pyhwm2014 requires a Fortran compiler (gfortran)")
                print("  تحذير: pyhwm2014 غير متاح. الرجوع إلى نموذج الرياح البسيط.")
                return self._calculate_simple_wind(altitudes)
            else:
                raise ImportError(
                    "pyhwm2014 is required for HWM14 wind model but not installed. "
                    "Install with: pip install pyhwm2014 (requires Fortran compiler). "
                    "Or set wind.hwm14.fallback_to_simple: true in config. "
                    "pyhwm2014 مطلوب لنموذج HWM14 لكنه غير مثبت."
                )
        
        # Get datetime and location parameters
        # الحصول على معاملات التاريخ والموقع
        dt, doy = self.get_datetime()
        lat = self.location.get('latitude', 0.0)
        lon = self.location.get('longitude', 0.0)
        
        # Get Ap index from space weather config
        # الحصول على مؤشر Ap من تكوين الطقس الفضائي
        ap_val = self.space_weather.get('ap', 15.0)
        
        # HWM14 altitude limits (0-500 km)
        # حدود ارتفاع HWM14 (0-500 كم)
        HWM14_MAX_ALT_KM = 500.0
        
        # Convert altitudes to km for HWM14
        # تحويل الارتفاعات إلى كم لـ HWM14
        altitudes_km = altitudes / 1000.0
        
        # Calculate UT (Universal Time) in hours
        # حساب التوقيت العالمي بالساعات
        ut_hours = dt.hour + dt.minute / 60.0 + dt.second / 3600.0
        
        # Initialize output arrays
        # تهيئة مصفوفات المخرجات
        wind_meridional = np.zeros(n_points)  # North-South (positive = northward)
        wind_zonal = np.zeros(n_points)       # East-West (positive = eastward)
        
        print(f"  Calculating HWM14 winds for {n_points} altitude points...")
        print(f"  Location: {lat:.4f}°N, {lon:.4f}°E")
        print(f"  Date: {dt.strftime('%Y-%m-%d')}, UT: {ut_hours:.2f} hours")
        print(f"  Ap index: {ap_val}")
        
        # Calculate winds for each altitude
        # حساب الرياح لكل ارتفاع
        # Note: HWM14 doesn't support vectorized calls, so we loop
        # ملاحظة: HWM14 لا يدعم الاستدعاءات المتجهة، لذلك نستخدم حلقة
        for i, alt_km in enumerate(altitudes_km):
            # Clamp altitude to HWM14 valid range
            # تقييد الارتفاع إلى النطاق الصالح لـ HWM14
            if alt_km > HWM14_MAX_ALT_KM:
                # Above HWM14 range, wind is negligible
                # فوق نطاق HWM14، الرياح مهملة
                wind_meridional[i] = 0.0
                wind_zonal[i] = 0.0
                continue
            
            try:
                # Create HWM14 object for this altitude
                # إنشاء كائن HWM14 لهذا الارتفاع
                # HWM14 parameters:
                # - alt: altitude in km
                # - glat: geographic latitude in degrees
                # - glon: geographic longitude in degrees  
                # - year: year
                # - day: day of year (1-366)
                # - ut: universal time in hours
                # - ap: Ap index array [-1, ap_value] for disturbed winds
                hwm14_obj = HWM14(
                    alt=float(alt_km),
                    glat=lat,
                    glon=lon,
                    year=dt.year,
                    day=doy,
                    ut=ut_hours,
                    ap=[-1, ap_val],  # -1 enables disturbed wind calculation
                    option=1,  # Height profile mode
                    verbose=False
                )
                
                # Extract total winds (quiet + disturbed)
                # استخراج الرياح الكلية (هادئة + مضطربة)
                # HWM14 outputs: Merid (meridional/north-south), Zonal (east-west)
                wind_meridional[i] = hwm14_obj.Wmerid.item() if hasattr(hwm14_obj.Wmerid, 'item') else float(hwm14_obj.Wmerid)
                wind_zonal[i] = hwm14_obj.Wzonal.item() if hasattr(hwm14_obj.Wzonal, 'item') else float(hwm14_obj.Wzonal)
                
            except Exception as e:
                if i == 0:  # Only warn once
                    print(f"  Warning: HWM14 calculation failed at {alt_km:.1f} km: {e}")
                    print(f"  Using zero wind for problematic altitudes.")
                wind_meridional[i] = 0.0
                wind_zonal[i] = 0.0
        
        # HWM14 convention: meridional = positive northward, zonal = positive eastward
        # This matches our wind_north_m_s and wind_east_m_s convention
        # اتفاقية HWM14: الميريديوني = موجب شمالاً، الزوني = موجب شرقاً
        # هذا يطابق اتفاقية wind_north_m_s و wind_east_m_s
        wind_north = wind_meridional
        wind_east = wind_zonal
        wind_down = np.zeros(n_points)  # HWM14 only provides horizontal winds
        
        # Sanity check: typical thermospheric winds are ±300 m/s
        # فحص صحة: الرياح الثرموسفيرية النموذجية ±300 م/ث
        max_wind = max(np.abs(wind_north).max(), np.abs(wind_east).max())
        if max_wind > 500:
            print(f"  Warning: Maximum wind speed ({max_wind:.1f} m/s) exceeds typical range.")
        
        print(f"  HWM14 wind range: meridional [{wind_north.min():.1f}, {wind_north.max():.1f}] m/s, "
              f"zonal [{wind_east.min():.1f}, {wind_east.max():.1f}] m/s")
        
        return {
            'wind_north_m_s': wind_north,
            'wind_east_m_s': wind_east,
            'wind_down_m_s': wind_down
        }
    
    def generate_table(self) -> pd.DataFrame:
        """
        Generate complete atmosphere table.
        توليد جدول الغلاف الجوي الكامل.
        
        Returns:
            DataFrame with atmosphere data
        """
        print("Generating altitude grid...")
        altitudes = self.generate_altitude_grid()
        print(f"  Generated {len(altitudes)} altitude points from {altitudes[0]:.1f} m to {altitudes[-1]:.1f} m")
        
        print("Calculating MSIS atmosphere...")
        atm_data = self.calculate_msis_atmosphere(altitudes)
        print(f"  Temperature range: {atm_data['temperature_K'].min():.1f} K to {atm_data['temperature_K'].max():.1f} K")
        print(f"  Density range: {atm_data['density_kg_m3'].min():.2e} to {atm_data['density_kg_m3'].max():.2e} kg/m^3")
        
        print("Calculating wind profile...")
        wind_data = self.calculate_wind(altitudes)
        max_wind = np.sqrt(wind_data['wind_north_m_s']**2 + wind_data['wind_east_m_s']**2).max()
        print(f"  Maximum wind speed: {max_wind:.1f} m/s")
        
        # Combine all data
        data = {
            'altitude_m': atm_data['altitude_m'],
            'pressure_Pa': atm_data['pressure_Pa'],
            'temperature_K': atm_data['temperature_K'],
            'density_kg_m3': atm_data['density_kg_m3'],
            'speed_of_sound_m_s': atm_data['speed_of_sound_m_s'],
            'wind_north_m_s': wind_data['wind_north_m_s'],
            'wind_east_m_s': wind_data['wind_east_m_s'],
            'wind_down_m_s': wind_data['wind_down_m_s']
        }
        
        df = pd.DataFrame(data)
        
        return df
    
    def save_table(self, df: pd.DataFrame, output_path: Path) -> None:
        """
        Save atmosphere table to CSV file.
        حفظ جدول الغلاف الجوي في ملف CSV.
        
        Args:
            df: DataFrame with atmosphere data
            output_path: Path to output file
        """
        precision = self.output_config.get('precision', {})
        
        # Determine altitude precision that ensures uniqueness
        # تحديد دقة الارتفاع التي تضمن التفرد
        # Altitude is the interpolation key, so it must be unique after formatting
        # الارتفاع هو مفتاح الاستيفاء، لذلك يجب أن يكون فريداً بعد التنسيق
        altitude_precision = precision.get('altitude', 1)
        max_altitude_precision = 6  # Maximum precision to try
        
        if 'altitude_m' in df.columns:
            altitudes = df['altitude_m'].values
            while altitude_precision <= max_altitude_precision:
                # Format altitudes and check for duplicates
                # تنسيق الارتفاعات والتحقق من التكرارات
                formatted_alts = [f"{x:.{altitude_precision}f}" for x in altitudes]
                if len(formatted_alts) == len(set(formatted_alts)):
                    # No duplicates, this precision is sufficient
                    # لا توجد تكرارات، هذه الدقة كافية
                    break
                altitude_precision += 1
            
            if altitude_precision > precision.get('altitude', 1):
                print(f"  Note: Increased altitude precision from {precision.get('altitude', 1)} to {altitude_precision} "
                      f"to ensure unique values (logarithmic spacing creates close values at low altitudes).")
                print(f"  ملاحظة: تم زيادة دقة الارتفاع من {precision.get('altitude', 1)} إلى {altitude_precision} "
                      f"لضمان قيم فريدة.")
        
        # Define formatters for each column
        # تعريف المنسقات لكل عمود
        formatters = {
            'altitude_m': lambda x: f"{x:.{altitude_precision}f}",
            'pressure_Pa': lambda x: f"{x:.{precision.get('pressure', 6)}f}",
            'temperature_K': lambda x: f"{x:.{precision.get('temperature', 4)}f}",
            'density_kg_m3': lambda x: f"{x:.{precision.get('density', 12)}e}",
            'speed_of_sound_m_s': lambda x: f"{x:.{precision.get('speed_of_sound', 4)}f}",
            'wind_north_m_s': lambda x: f"{x:.{precision.get('wind', 4)}f}",
            'wind_east_m_s': lambda x: f"{x:.{precision.get('wind', 4)}f}",
            'wind_down_m_s': lambda x: f"{x:.{precision.get('wind', 4)}f}"
        }
        
        # Apply formatters to create a copy with formatted string values
        # تطبيق المنسقات لإنشاء نسخة بقيم نصية منسقة
        # Note: pandas to_csv() doesn't support 'formatters' parameter,
        # so we format columns before saving
        # ملاحظة: pandas to_csv() لا يدعم معامل 'formatters'،
        # لذلك ننسق الأعمدة قبل الحفظ
        df_formatted = df.copy()
        for col, fmt_func in formatters.items():
            if col in df_formatted.columns:
                df_formatted[col] = df_formatted[col].apply(fmt_func)
        
        # Create output directory if needed
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Save to CSV
        # حفظ إلى CSV
        df_formatted.to_csv(output_path, index=False)
        
        print(f"Saved atmosphere table to: {output_path}")
        print(f"  File size: {output_path.stat().st_size / 1024:.1f} KB")


def find_rocket_models(base_dir: Path) -> List[Path]:
    """
    Find all rocket model directories (both single-stage and multi-stage).
    البحث عن جميع مجلدات نماذج الصواريخ (أحادية المرحلة ومتعددة المراحل).
    
    This function finds all directories containing rocket_properties.yaml,
    which includes:
    - Single-stage rockets: rocket_name/rocket_properties.yaml
    - Multi-stage rockets: rocket_name/stage1/rocket_properties.yaml, 
                          rocket_name/warhead/rocket_properties.yaml, etc.
    
    هذه الدالة تجد جميع المجلدات التي تحتوي على rocket_properties.yaml،
    والتي تشمل:
    - الصواريخ أحادية المرحلة: rocket_name/rocket_properties.yaml
    - الصواريخ متعددة المراحل: rocket_name/stage1/rocket_properties.yaml،
                              rocket_name/warhead/rocket_properties.yaml، إلخ.
    
    Args:
        base_dir: Base directory containing rocket models
        
    Returns:
        List of paths to directories containing rocket_properties.yaml
    """
    rocket_dirs = []
    
    if not base_dir.exists():
        print(f"Warning: Rocket models directory not found: {base_dir}")
        return rocket_dirs
    
    for item in base_dir.iterdir():
        if item.is_dir() and item.name.startswith('rocket_'):
            # Check if it's a single-stage rocket (rocket_properties.yaml in root)
            # التحقق مما إذا كان صاروخاً أحادي المرحلة
            if (item / 'rocket_properties.yaml').exists():
                rocket_dirs.append(item)
            else:
                # Check for multi-stage rocket (rocket_properties.yaml in subdirectories)
                # التحقق من الصاروخ متعدد المراحل
                for subdir in item.iterdir():
                    if subdir.is_dir() and (subdir / 'rocket_properties.yaml').exists():
                        rocket_dirs.append(subdir)
    
    return sorted(rocket_dirs)


def get_rocket_launch_coordinates(rocket_dir: Path, config_dir: Path) -> Dict[str, float]:
    """
    Get launch coordinates from rocket or config files.
    الحصول على إحداثيات الإطلاق من ملفات الصاروخ أو التكوين.
    
    Args:
        rocket_dir: Path to rocket model directory
        config_dir: Path to config directory
        
    Returns:
        Dictionary with latitude, longitude, altitude
    """
    # Default coordinates
    coords = {
        'latitude': 16.457472,
        'longitude': 44.115361,
        'ground_altitude_m': 0.0
    }
    
    # Try to read from config file
    config_file = config_dir / '6dof_config_advanced.yaml'
    if config_file.exists():
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            launch = config.get('launch', {})
            if 'latitude' in launch:
                coords['latitude'] = launch['latitude']
            if 'longitude' in launch:
                coords['longitude'] = launch['longitude']
            if 'altitude' in launch:
                coords['ground_altitude_m'] = launch['altitude']
        except Exception as e:
            print(f"  Warning: Could not read config file: {e}")
    
    return coords


def load_config(config_path: Path) -> Dict[str, Any]:
    """
    Load configuration from YAML file.
    تحميل التكوين من ملف YAML.
    
    Args:
        config_path: Path to configuration file
        
    Returns:
        Configuration dictionary
    """
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def generate_for_all_rockets(config: Dict[str, Any], base_dir: Path) -> None:
    """
    Generate atmosphere tables for all rockets.
    توليد جداول الغلاف الجوي لجميع الصواريخ.
    
    Args:
        config: Configuration dictionary
        base_dir: Base directory of 6dof simulation
    """
    rockets_config = config.get('rockets', {})
    models_dir = base_dir / rockets_config.get('models_dir', 'data/rocket_models')
    config_dir = base_dir / 'config'
    
    # Find rocket models
    specific_rockets = rockets_config.get('specific_rockets')
    if specific_rockets:
        rocket_dirs = [models_dir / name for name in specific_rockets if (models_dir / name).exists()]
    else:
        rocket_dirs = find_rocket_models(models_dir)
    
    if not rocket_dirs:
        print("No rocket models found!")
        return
    
    print(f"\nFound {len(rocket_dirs)} rocket model directories:")
    print(f"تم العثور على {len(rocket_dirs)} مجلد نموذج صاروخ:")
    for rd in rocket_dirs:
        # Show relative path from models_dir for clarity
        # عرض المسار النسبي من models_dir للوضوح
        try:
            rel_path = rd.relative_to(models_dir)
            print(f"  - {rel_path}")
        except ValueError:
            print(f"  - {rd.name}")
    
    # Generate for each rocket
    output_filename = config.get('output', {}).get('filename', 'atmosphere_table.csv')
    
    for rocket_dir in rocket_dirs:
        # Show relative path from models_dir for clarity
        # عرض المسار النسبي من models_dir للوضوح
        try:
            rel_path = rocket_dir.relative_to(models_dir)
        except ValueError:
            rel_path = rocket_dir.name
        
        print(f"\n{'='*60}")
        print(f"Processing: {rel_path}")
        print(f"معالجة: {rel_path}")
        print(f"{'='*60}")
        
        # Get launch coordinates for this rocket
        coords = get_rocket_launch_coordinates(rocket_dir, config_dir)
        
        # Update config with rocket-specific coordinates using deep copy
        # to avoid modifying nested dictionaries in the original config
        # تحديث التكوين بإحداثيات الصاروخ باستخدام نسخ عميق
        # لتجنب تعديل القواميس المتداخلة في التكوين الأصلي
        rocket_config = copy.deepcopy(config)
        rocket_config['location'] = {
            'latitude': coords['latitude'],
            'longitude': coords['longitude'],
            'ground_altitude_m': coords['ground_altitude_m']
        }
        
        print(f"  Location: {coords['latitude']:.4f}N, {coords['longitude']:.4f}E")
        print(f"  Ground altitude: {coords['ground_altitude_m']:.1f} m")
        
        # Generate table
        generator = MSISAtmosphereGenerator(rocket_config)
        df = generator.generate_table()
        
        # Save to rocket directory
        output_path = rocket_dir / output_filename
        generator.save_table(df, output_path)
    
    print(f"\n{'='*60}")
    print(f"Completed! Generated tables for {len(rocket_dirs)} rockets.")
    print(f"{'='*60}")


def main():
    """
    Main entry point.
    نقطة الدخول الرئيسية.
    """
    parser = argparse.ArgumentParser(
        description='Generate atmosphere tables using MSIS empirical model',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate for all rockets using default config
  python msis_generator.py
  
  # Generate using custom config file
  python msis_generator.py --config my_config.yaml
  
  # Generate for a single rocket
  python msis_generator.py --rocket rocket_tail_single_esmail
  
  # Generate with custom output path
  python msis_generator.py --output /path/to/output.csv
  
  # Generate with custom altitude range (0 to 100 km)
  python msis_generator.py --alt-min 0 --alt-max 100000
  
  # Generate with custom number of points and linear spacing
  python msis_generator.py --num-points 500 --spacing linear
  
  # Generate for specific rocket with custom altitude settings
  python msis_generator.py --rocket rocket_tail_single_esmail --alt-min 1000 --alt-max 200000 --num-points 800
        """
    )
    
    parser.add_argument(
        '--config', '-c',
        type=Path,
        default=Path(__file__).parent / 'config.yaml',
        help='Path to configuration file (default: config.yaml)'
    )
    
    parser.add_argument(
        '--rocket', '-r',
        type=str,
        default=None,
        help='Generate for specific rocket only'
    )
    
    parser.add_argument(
        '--output', '-o',
        type=Path,
        default=None,
        help='Output file path (overrides config)'
    )
    
    parser.add_argument(
        '--base-dir', '-b',
        type=Path,
        default=None,
        help='Base directory of 6dof simulation'
    )
    
    # Altitude grid parameters
    # معاملات شبكة الارتفاع
    parser.add_argument(
        '--alt-min',
        type=float,
        default=None,
        help='Minimum altitude in meters (overrides config)'
    )
    
    parser.add_argument(
        '--alt-max',
        type=float,
        default=None,
        help='Maximum altitude in meters (overrides config)'
    )
    
    parser.add_argument(
        '--num-points',
        type=int,
        default=None,
        help='Number of altitude points in the table (overrides config)'
    )
    
    parser.add_argument(
        '--spacing',
        type=str,
        choices=['linear', 'logarithmic'],
        default=None,
        help='Altitude spacing mode: "linear" or "logarithmic" (overrides config)'
    )
    
    args = parser.parse_args()
    
    # Check pymsis availability
    if not PYMSIS_AVAILABLE:
        print("ERROR: pymsis is required but not installed.")
        print("Install with: pip install pymsis")
        sys.exit(1)
    
    # Load configuration
    if args.config.exists():
        print(f"Loading configuration from: {args.config}")
        config = load_config(args.config)
        # Debug: Print loaded altitude configuration
        # تصحيح: طباعة تكوين الارتفاع المحمّل
        altitude_config = config.get('altitude', {})
        print(f"Loaded altitude config: {altitude_config}")
    else:
        print(f"Warning: Config file not found: {args.config}")
        print("Using default configuration...")
        config = {}
    
    # Determine base directory
    if args.base_dir:
        base_dir = args.base_dir
    else:
        # Assume we're in 6dof/tools/msis_atmosphere_generator/
        base_dir = Path(__file__).parent.parent.parent
    
    print(f"Base directory: {base_dir}")
    
    # Handle specific rocket or output path
    if args.rocket:
        config.setdefault('rockets', {})['specific_rockets'] = [args.rocket]
    
    if args.output:
        config.setdefault('output', {})['filename'] = args.output.name
    
    # Handle altitude grid parameters from CLI
    # معالجة معاملات شبكة الارتفاع من سطر الأوامر
    if args.alt_min is not None or args.alt_max is not None or args.num_points is not None or args.spacing is not None:
        config.setdefault('altitude', {})
        if args.alt_min is not None:
            config['altitude']['min_m'] = args.alt_min
            print(f"Altitude min (from CLI): {args.alt_min} m")
        if args.alt_max is not None:
            config['altitude']['max_m'] = args.alt_max
            print(f"Altitude max (from CLI): {args.alt_max} m")
        if args.num_points is not None:
            config['altitude']['num_points'] = args.num_points
            print(f"Number of points (from CLI): {args.num_points}")
        if args.spacing is not None:
            config['altitude']['spacing'] = args.spacing
            print(f"Spacing mode (from CLI): {args.spacing}")
    
    # Generate tables
    generate_for_all_rockets(config, base_dir)


if __name__ == '__main__':
    main()
