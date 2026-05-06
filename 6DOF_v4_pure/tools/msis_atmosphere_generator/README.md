# MSIS Atmosphere Table Generator
# مولد جداول الغلاف الجوي MSIS

## Overview | نظرة عامة

This tool generates atmosphere tables using the **NRLMSISE-00/MSIS2.1** empirical atmospheric model. These tables are designed for use with the 6DOF rocket trajectory simulation and provide atmospheric properties from ground level up to 400+ km altitude.

هذه الأداة تولد جداول غلاف جوي باستخدام نموذج **NRLMSISE-00/MSIS2.1** التجريبي. هذه الجداول مصممة للاستخدام مع محاكاة مسار الصواريخ 6DOF وتوفر خصائص جوية من مستوى الأرض حتى ارتفاع 400+ كم.

> **Important Note on MSIS Model Limitations | ملاحظة مهمة حول قيود نموذج MSIS:**
>
> MSIS is primarily an **upper atmosphere climatological model** optimized for altitudes above 80-100 km. At lower altitudes (<20-30 km), MSIS provides **climatological averages** rather than actual meteorological conditions. For applications requiring high accuracy at low altitudes, consider using:
> - International Standard Atmosphere (ISA) for engineering approximations
> - Local radiosonde/weather data for actual conditions
> - MSIS values as reasonable approximations for trajectory simulations
>
> نموذج MSIS هو في الأساس **نموذج مناخي للغلاف الجوي العلوي** محسّن للارتفاعات فوق 80-100 كم. في الارتفاعات المنخفضة (<20-30 كم)، يوفر MSIS **متوسطات مناخية** وليس ظروفاً أرصادية فعلية. للتطبيقات التي تتطلب دقة عالية في الارتفاعات المنخفضة، يُنصح باستخدام:
> - الغلاف الجوي القياسي الدولي (ISA) للتقريبات الهندسية
> - بيانات الراديوزوند/الطقس المحلية للظروف الفعلية
> - قيم MSIS كتقريبات معقولة لمحاكاة المسارات

## Features | الميزات

- **Real Atmospheric Data**: Uses MSIS empirical model based on satellite and ground measurements
- **High Altitude Support**: Generates tables up to 400 km (or higher) for ballistic missile simulations
- **Configurable Parameters**: All inputs are configurable via YAML file
- **Space Weather Effects**: Accounts for solar activity (F10.7) and geomagnetic conditions (Ap)
- **Wind Profiles**: Includes realistic wind profiles with jet stream and boundary layer effects
- **Batch Generation**: Automatically generates tables for all rockets in the repository
- **Arabic Documentation**: Full bilingual documentation (English/Arabic)

- **بيانات جوية حقيقية**: يستخدم نموذج MSIS التجريبي المبني على قياسات الأقمار الصناعية والأرضية
- **دعم الارتفاعات العالية**: يولد جداول حتى 400 كم (أو أعلى) لمحاكاة الصواريخ الباليستية
- **معاملات قابلة للتكوين**: جميع المدخلات قابلة للتكوين عبر ملف YAML
- **تأثيرات الطقس الفضائي**: يأخذ في الاعتبار النشاط الشمسي (F10.7) والظروف الجيومغناطيسية (Ap)
- **ملفات الرياح**: يتضمن ملفات رياح واقعية مع التيار النفاث وتأثيرات الطبقة الحدودية
- **التوليد الدفعي**: يولد الجداول تلقائياً لجميع الصواريخ في المستودع
- **توثيق عربي**: توثيق ثنائي اللغة كامل (إنجليزي/عربي)

## Installation | التثبيت

### Prerequisites | المتطلبات الأساسية

```bash
# Install required Python packages
# تثبيت حزم Python المطلوبة
pip install -r requirements.txt
```

Or install individually:
أو التثبيت بشكل فردي:

```bash
pip install pymsis numpy pandas scipy pyyaml
```

### Verify Installation | التحقق من التثبيت

```bash
python -c "import pymsis; print('pymsis version:', pymsis.__version__)"
```

## Quick Start | البدء السريع

### 1. Generate for All Rockets | التوليد لجميع الصواريخ

```bash
cd 6dof/tools/msis_atmosphere_generator
python msis_generator.py
```

This will:
- Find all rocket models in `data/rocket_models/`
- Generate one `atmosphere_table.csv` for each rocket
- Use default configuration from `config.yaml`

سيقوم هذا بـ:
- البحث عن جميع نماذج الصواريخ في `data/rocket_models/`
- توليد ملف `atmosphere_table.csv` واحد لكل صاروخ
- استخدام التكوين الافتراضي من `config.yaml`

### 2. Generate for Specific Rocket | التوليد لصاروخ محدد

```bash
python msis_generator.py --rocket rocket_tail_single_esmail
```

### 3. Use Custom Configuration | استخدام تكوين مخصص

```bash
python msis_generator.py --config my_custom_config.yaml
```

## Configuration | التكوين

The `config.yaml` file controls all aspects of table generation:

ملف `config.yaml` يتحكم في جميع جوانب توليد الجداول:

### Location Settings | إعدادات الموقع

```yaml
location:
  latitude: 16.457472      # Degrees (positive = North)
  longitude: 44.115361     # Degrees (positive = East)
  ground_altitude_m: 1795.0  # Ground elevation (m)
```

### Date and Time | التاريخ والوقت

```yaml
datetime:
  year: 2024
  month: 6      # 1-12 (affects seasonal variations)
  day: 21
  hour: 12      # UTC (affects diurnal variations)
```

### Space Weather Parameters | معاملات الطقس الفضائي

These parameters significantly affect upper atmosphere density (>100 km):

هذه المعاملات تؤثر بشكل كبير على كثافة الغلاف الجوي العلوي (>100 كم):

```yaml
space_weather:
  f107: 150.0    # Solar radio flux (70-250 SFU)
  f107a: 150.0   # 81-day average of F10.7
  ap: 15.0       # Geomagnetic index (0-400)
```

| Parameter | Solar Minimum | Average | Solar Maximum | Storm |
|-----------|--------------|---------|---------------|-------|
| F10.7     | 70           | 150     | 250           | -     |
| Ap        | 4            | 15      | 30            | 100+  |

| المعامل | الحد الأدنى الشمسي | المتوسط | الحد الأقصى الشمسي | عاصفة |
|---------|-------------------|---------|-------------------|-------|
| F10.7   | 70                | 150     | 250               | -     |
| Ap      | 4                 | 15      | 30                | 100+  |

### Altitude Grid | شبكة الارتفاع

```yaml
altitude:
  min_m: 0.0           # Minimum altitude (m)
  max_m: 400000.0      # Maximum altitude (m)
  num_points: 1000     # Number of grid points
  spacing: "logarithmic"  # "linear" or "logarithmic"
```

**Note**: Logarithmic spacing provides better resolution at lower altitudes where atmospheric gradients are steeper.

**ملاحظة**: التباعد اللوغاريتمي يوفر دقة أفضل في الارتفاعات المنخفضة حيث تدرجات الغلاف الجوي أكثر حدة.

### Wind Model | نموذج الرياح

```yaml
wind:
  enabled: true
  model: "simple"  # "simple", "hwm14", or "none"
  simple:
    surface_speed: 5.0          # Surface wind (m/s)
    surface_direction: 45.0     # Direction (degrees from North)
    jet_stream_altitude: 12000.0  # Jet stream height (m)
    jet_stream_speed: 30.0      # Jet stream speed (m/s)
    wind_cutoff_altitude: 100000.0  # Wind negligible above this (m)
  hwm14:
    fallback_to_simple: true    # Fall back to simple model if HWM14 not available
```

#### HWM14 Wind Model (Optional) | نموذج الرياح HWM14 (اختياري)

The **Horizontal Wind Model 2014 (HWM14)** is an empirical model developed by the Naval Research Laboratory that provides realistic horizontal wind profiles from 0-500 km altitude based on satellite and ground-based measurements.

نموذج **الرياح الأفقية 2014 (HWM14)** هو نموذج تجريبي طورته مختبرات البحث البحرية يوفر ملفات رياح أفقية واقعية من 0-500 كم ارتفاع بناءً على قياسات الأقمار الصناعية والأرض.

**Installation | التثبيت:**

HWM14 requires a Fortran compiler (gfortran) to build. Install with:

HWM14 يتطلب مترجم Fortran (gfortran) للبناء. التثبيت:

```bash
# Install Fortran compiler (Ubuntu/Debian)
# تثبيت مترجم Fortran (Ubuntu/Debian)
sudo apt-get install gfortran

# Install pyhwm2014
# تثبيت pyhwm2014
pip install pyhwm2014
```

Or install from source:
أو التثبيت من المصدر:

```bash
git clone https://github.com/rilma/pyHWM14.git
cd pyHWM14
pip install .
```

**Usage | الاستخدام:**

To use HWM14, set `model: "hwm14"` in your config.yaml. If pyhwm2014 is not installed and `fallback_to_simple: true`, the generator will automatically fall back to the simple wind model.

لاستخدام HWM14، اضبط `model: "hwm14"` في config.yaml. إذا لم يكن pyhwm2014 مثبتاً و `fallback_to_simple: true`، سيرجع المولد تلقائياً إلى نموذج الرياح البسيط.

| Wind Model | Description | Accuracy | Requirements |
|------------|-------------|----------|--------------|
| simple | Altitude-dependent profile | Low | None |
| hwm14 | Empirical model (NRL) | High | pyhwm2014 + gfortran |
| none | No wind (zeros) | N/A | None |

| نموذج الرياح | الوصف | الدقة | المتطلبات |
|--------------|-------|-------|-----------|
| simple | ملف يعتمد على الارتفاع | منخفضة | لا شيء |
| hwm14 | نموذج تجريبي (NRL) | عالية | pyhwm2014 + gfortran |
| none | بدون رياح (أصفار) | غير متاح | لا شيء |

### MSIS Version | إصدار MSIS

```yaml
msis:
  version: "msis2.1"  # "nrlmsise00", "msis2.0", or "msis2.1"
```

| Version | Description | Recommended For |
|---------|-------------|-----------------|
| nrlmsise00 | Classic model (2000) | Legacy compatibility |
| msis2.0 | Updated model (2019) | General use |
| msis2.1 | Latest model (2021) | Best accuracy |

## Output Format | تنسيق المخرجات

The generated CSV file contains the following columns:

ملف CSV المولد يحتوي على الأعمدة التالية:

| Column | Unit | Description (EN) | الوصف (AR) |
|--------|------|------------------|------------|
| altitude_m | m | Altitude above sea level | الارتفاع فوق مستوى سطح البحر |
| pressure_Pa | Pa | Atmospheric pressure | الضغط الجوي |
| temperature_K | K | Temperature | درجة الحرارة |
| density_kg_m3 | kg/m³ | Air density | كثافة الهواء |
| speed_of_sound_m_s | m/s | Speed of sound | سرعة الصوت |
| wind_north_m_s | m/s | Northward wind component | مكون الرياح الشمالي |
| wind_east_m_s | m/s | Eastward wind component | مكون الرياح الشرقي |
| wind_down_m_s | m/s | Downward wind component | مكون الرياح السفلي |

### Example Output | مثال على المخرجات

```csv
altitude_m,pressure_Pa,temperature_K,density_kg_m3,speed_of_sound_m_s,wind_north_m_s,wind_east_m_s,wind_down_m_s
1795.0,81234.567890,281.5432,1.004567890123e+00,336.4521,3.5355,3.5355,0.0000
2000.0,79432.123456,279.8765,9.876543210123e-01,335.2134,3.6789,3.6789,0.0000
...
400000.0,0.000001,1245.6789,2.345678901234e-12,707.1234,0.0000,0.0000,0.0000
```

## Usage in Simulation | الاستخدام في المحاكاة

After generating the atmosphere table, update your rocket's configuration to use it:

بعد توليد جدول الغلاف الجوي، حدّث تكوين صاروخك لاستخدامه:

### Option 1: Direct Path | المسار المباشر

```yaml
# In 6dof_config_advanced.yaml
atmosphere:
  table_file: "data/rocket_models/rocket_tail_single_esmail/atmosphere_table.csv"
```

### Option 2: Relative to Rocket | نسبي للصاروخ

```yaml
# In rocket_properties.yaml
atmosphere:
  table_file: "atmosphere_table.csv"
```

## Advanced Usage | الاستخدام المتقدم

### Command Line Options | خيارات سطر الأوامر

```bash
python msis_generator.py [OPTIONS]

Options:
  -c, --config PATH    Configuration file (default: config.yaml)
  -r, --rocket NAME    Generate for specific rocket only
  -o, --output PATH    Output file path (overrides config)
  -b, --base-dir PATH  Base directory of 6dof simulation
  --alt-min FLOAT      Minimum altitude in meters (overrides config)
  --alt-max FLOAT      Maximum altitude in meters (overrides config)
  --num-points INT     Number of altitude points in the table (overrides config)
  --spacing MODE       Altitude spacing: "linear" or "logarithmic" (overrides config)
  -h, --help           Show help message
```

### Altitude Grid Examples | أمثلة شبكة الارتفاع

```bash
# Generate with custom altitude range (0 to 100 km)
# التوليد بنطاق ارتفاع مخصص (0 إلى 100 كم)
python msis_generator.py --alt-min 0 --alt-max 100000

# Generate with custom number of points and linear spacing
# التوليد بعدد نقاط مخصص وتباعد خطي
python msis_generator.py --num-points 500 --spacing linear

# Generate for specific rocket with custom altitude settings
# التوليد لصاروخ محدد بإعدادات ارتفاع مخصصة
python msis_generator.py --rocket rocket_tail_single_esmail --alt-min 1000 --alt-max 200000 --num-points 800
```

### Programmatic Usage | الاستخدام البرمجي

```python
from msis_generator import MSISAtmosphereGenerator, load_config

# Load configuration
config = load_config('config.yaml')

# Modify configuration programmatically
config['location']['latitude'] = 25.0
config['space_weather']['f107'] = 200.0

# Generate table
generator = MSISAtmosphereGenerator(config)
df = generator.generate_table()

# Save to custom location
from pathlib import Path
generator.save_table(df, Path('my_atmosphere.csv'))
```

### Batch Generation with Custom Settings | التوليد الدفعي بإعدادات مخصصة

```python
from msis_generator import MSISAtmosphereGenerator, find_rocket_models
from pathlib import Path

base_dir = Path('/path/to/6dof')
rockets = find_rocket_models(base_dir / 'data/rocket_models')

# Generate with different space weather for each rocket
for rocket_dir in rockets:
    config = {
        'location': {'latitude': 16.5, 'longitude': 44.1, 'ground_altitude_m': 1800},
        'datetime': {'year': 2024, 'month': 6, 'day': 21, 'hour': 12},
        'space_weather': {'f107': 150, 'f107a': 150, 'ap': 15},
        'altitude': {'min_m': 0, 'max_m': 400000, 'num_points': 1000, 'spacing': 'logarithmic'},
        'wind': {'enabled': True, 'model': 'simple'},
        'msis': {'version': 'msis2.1'}
    }
    
    generator = MSISAtmosphereGenerator(config)
    df = generator.generate_table()
    generator.save_table(df, rocket_dir / 'atmosphere_table.csv')
```

## Scientific Background | الخلفية العلمية

### MSIS Model | نموذج MSIS

The **Mass Spectrometer and Incoherent Scatter** (MSIS) model is an empirical atmospheric model developed by the Naval Research Laboratory (NRL). It provides:

نموذج **مطياف الكتلة والتشتت غير المتماسك** (MSIS) هو نموذج جوي تجريبي طورته مختبرات البحث البحرية (NRL). يوفر:

- **Temperature**: Neutral temperature from 0-1000 km
- **Density**: Total mass density and individual species (N2, O2, O, He, H, Ar, N)
- **Composition**: Number densities of atmospheric constituents

- **درجة الحرارة**: درجة الحرارة المحايدة من 0-1000 كم
- **الكثافة**: الكثافة الكلية والأنواع الفردية (N2, O2, O, He, H, Ar, N)
- **التركيب**: كثافات الأعداد للمكونات الجوية

### Data Sources | مصادر البيانات

MSIS is based on data from:
- Incoherent scatter radars (Millstone Hill, Arecibo, etc.)
- Satellite mass spectrometers (AE-C, AE-E, DE-2, etc.)
- Satellite drag measurements
- Rocket-borne instruments

MSIS مبني على بيانات من:
- رادارات التشتت غير المتماسك (Millstone Hill, Arecibo, إلخ)
- مطيافات الكتلة على الأقمار الصناعية (AE-C, AE-E, DE-2, إلخ)
- قياسات سحب الأقمار الصناعية
- أدوات محمولة على الصواريخ

### Accuracy | الدقة

| Altitude Range | Temperature Accuracy | Density Accuracy |
|----------------|---------------------|------------------|
| 0-100 km       | ±5 K                | ±10%             |
| 100-200 km     | ±20 K               | ±15%             |
| 200-400 km     | ±50 K               | ±20-30%          |
| >400 km        | ±100 K              | ±30-50%          |

## Physical Limitations | القيود الفيزيائية

### Speed of Sound Above 100 km | سرعة الصوت فوق 100 كم

The concept of "speed of sound" loses its physical meaning in **rarefied flow** (Knudsen number Kn > 0.01), which occurs above approximately **100 km altitude**. In the continuum regime (Kn < 0.01), sound propagates as pressure waves through molecular collisions. In rarefied flow, the mean free path becomes comparable to or larger than the characteristic length scale, and the continuum assumption breaks down.

مفهوم "سرعة الصوت" يفقد معناه الفيزيائي في **التدفق المخلخل** (عدد كنودسن Kn > 0.01)، الذي يحدث فوق ارتفاع **~100 كم** تقريباً. في نظام الاستمرارية (Kn < 0.01)، ينتشر الصوت كموجات ضغط عبر تصادمات الجزيئات. في التدفق المخلخل، يصبح المسار الحر المتوسط مماثلاً أو أكبر من مقياس الطول المميز، وينهار افتراض الاستمرارية.

**The values calculated above ~100 km are provided for:**
- Continuity of the data table (no discontinuities)
- Reference calculations (e.g., thermal velocity scaling)
- Simulation codes that may use this value for other purposes

**القيم المحسوبة فوق ~100 كم مقدمة لـ:**
- استمرارية جدول البيانات (بدون انقطاعات)
- حسابات مرجعية (مثل تدريج السرعة الحرارية)
- أكواد المحاكاة التي قد تستخدم هذه القيمة لأغراض أخرى

> **Note**: For aerodynamic calculations above 100 km, use **free molecular flow models** instead of continuum aerodynamics.
>
> **ملاحظة**: للحسابات الديناميكية الهوائية فوق 100 كم، استخدم **نماذج التدفق الجزيئي الحر** بدلاً من الديناميكا الهوائية الاستمرارية.

### Simple Wind Model Limitations | قيود نموذج الرياح البسيط

The `simple` wind model is a **heuristic approximation** and is NOT based on validated physical data or actual meteorological measurements. It provides a reasonable altitude-dependent wind profile for trajectory simulations but does not represent actual conditions at any specific location or time.

نموذج الرياح `simple` هو **تقريب تجريبي** وليس مبنياً على بيانات فيزيائية محققة أو قياسات أرصاد جوية فعلية. يوفر ملف رياح معقول يعتمد على الارتفاع لمحاكاة المسارات لكنه لا يمثل الظروف الفعلية في أي موقع أو وقت محدد.

**The simple wind model is suitable for:**
- Initial trajectory estimates where wind effects are secondary
- Sensitivity studies to understand wind impact
- Cases where "reasonable" wind is needed, not meteorological accuracy

**نموذج الرياح البسيط مناسب لـ:**
- تقديرات المسار الأولية حيث تأثيرات الرياح ثانوية
- دراسات الحساسية لفهم تأثير الرياح
- الحالات التي تحتاج "رياح معقولة"، وليس دقة أرصاد جوية

> **Recommendation**: For accurate wind modeling, use the `hwm14` model which is based on empirical satellite and ground-based measurements from the Naval Research Laboratory.
>
> **توصية**: للحصول على نمذجة رياح دقيقة، استخدم نموذج `hwm14` المبني على قياسات تجريبية من الأقمار الصناعية والأرض من مختبرات البحث البحرية.

## Troubleshooting | استكشاف الأخطاء

### Common Issues | المشاكل الشائعة

**1. pymsis not found**
```
ImportError: No module named 'pymsis'
```
Solution: `pip install pymsis`

**2. Invalid date/time**
```
ValueError: day is out of range for month
```
Solution: Check datetime configuration for valid values

**3. NaN values in output**
```
Warning: NaN values detected in MSIS output
```
Solution: This can occur at very high altitudes (>500 km). The generator automatically handles this by replacing NaN with minimum valid values.

**4. Memory error with large grids**
```
MemoryError: Unable to allocate array
```
Solution: Reduce `num_points` in altitude configuration

## References | المراجع

1. Picone, J. M., et al. (2002). "NRLMSISE-00 empirical model of the atmosphere: Statistical comparisons and scientific issues." Journal of Geophysical Research, 107(A12).

2. Emmert, J. T., et al. (2021). "NRLMSIS 2.0: A whole-atmosphere empirical model of temperature and neutral species densities." Earth and Space Science, 8(3).

3. Drob, D. P., et al. (2015). "An update to the Horizontal Wind Model (HWM): The quiet time thermosphere." Earth and Space Science, 2(7).

## License | الترخيص

This tool is part of the 6DOF rocket simulation project. See the main project license for details.

هذه الأداة جزء من مشروع محاكاة الصواريخ 6DOF. راجع ترخيص المشروع الرئيسي للتفاصيل.

## Contact | التواصل

For issues or questions, please open an issue in the main repository.

للمشاكل أو الأسئلة، يرجى فتح issue في المستودع الرئيسي.
