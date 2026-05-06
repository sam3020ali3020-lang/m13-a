# 07 — التنظيف والإيقاف التلقائي

> **المبدأ:** كل تشغيل يُنظَّف قبل وبعد — لا بيانات محفوظة، لا عمليات زومبي، لا مشاريع UI عالقة.

## 🧹 ما الذي يُنظَّف تلقائياً

### قبل كل تشغيل
| ما يُحذَف | المكان | السبب |
|---|---|---|
| نتائج HITL قديمة | `results/hitl_tcp/run_*` (أقدم من آخر 5) | توفير مساحة + لا لبس |
| ملفات adb forward سابقة | `adb forward --remove-all` | تجنّب تعارض منافذ |
| عمليات Python قديمة | `pkill -f run_hitl_tcp` | تعارض TCP |
| QGC caches | 6 مجلّدات (انظر أدناه) | منع ظهور مشاريع قديمة |

### بعد كل تشغيل (finally + trap)
- التطبيق على الهاتف: `am force-stop com.ardophone.px4v17`
- adb port forwards: `adb forward --remove-all`
- QGC: SIGTERM → SIGKILL بعد 5s
- Python bridge process

## 🗑️ نظافة QGC الشاملة (Nuclear Wipe)

6 مجلّدات تُحذَف قبل كل تشغيل QGC:

```bash
~/.config/QGroundControl.org/
~/.local/share/QGroundControl.org/
~/Documents/QGroundControl/
~/.config/qgroundcontrol/
~/.local/share/qgroundcontrol/
/tmp/QGroundControl-*
```

**السبب:** QGC يحفظ:
- آخر موقع خريطة → يعرضه في المرة التالية (مُربِك)
- مشاريع بلان قديمة → تظهر في قائمة Plans
- إعدادات تيليمتري → قد تتعارض مع tcp:5760
- logs → تتراكم بسرعة

## ⚙️ آلية التنظيف في `run_phone_hitl.sh`

```bash
# في أعلى الملف
trap cleanup_on_exit EXIT INT TERM

cleanup_on_exit() {
    local exit_code=$?
    log_info "═══ Cleanup on exit (code=$exit_code) ═══"
    shutdown_phone_app || true
    kill_qgc || true
    adb forward --remove-all 2>/dev/null || true
    exit $exit_code
}
```

يضمن أنّ حتى لو:
- ضغط المستخدم Ctrl+C
- حدث خطأ بايثون
- انفصل كابل USB
- انتهى الوقت

...الكل يُنظَّف.

## 🧼 حفظ نتائج قليلة فقط

```python
# في run_hitl_tcp.py
def cleanup_old_runs(runs_dir: Path, keep: int = 5):
    runs = sorted(runs_dir.glob("run_*"), key=lambda p: p.stat().st_mtime)
    for old in runs[:-keep]:
        shutil.rmtree(old, ignore_errors=True)
```

**تحكّم يدوي:**
```bash
# احتفظ بآخر 10 نتائج
bash hitl/run_phone_hitl.sh --keep-runs 10

# احتفظ بكل النتائج (للتحليل الإحصائي)
bash hitl/run_phone_hitl.sh --keep-runs 999
```

## 📱 إيقاف التطبيق على الهاتف تلقائياً

```python
def shutdown_phone_app():
    subprocess.run(['adb', 'shell', 'am', 'force-stop', 'com.ardophone.px4v17'],
                   check=False, timeout=10)
    subprocess.run(['adb', 'forward', '--remove-all'],
                   check=False, timeout=5)
```

هذا يضمن:
- التطبيق يُغلق فعلاً (وليس في الخلفية)
- البطارية لا تُستنزَف
- المحاكاة القادمة تبدأ من حالة نظيفة

## 🔄 تنظيف يدوي شامل (إذا شيء تعطّل)

```bash
# 1) اقتل كل شيء متعلّق بـMAVLink
pkill -9 -f "run_hitl|QGroundControl|mavlink"

# 2) إعادة تعيين adb كاملة
adb kill-server
adb start-server
adb shell am force-stop com.ardophone.px4v17

# 3) امسح QGC بالكامل
rm -rf ~/.config/QGroundControl.org \
       ~/.local/share/QGroundControl.org \
       ~/Documents/QGroundControl \
       ~/.config/qgroundcontrol \
       ~/.local/share/qgroundcontrol \
       /tmp/QGroundControl-*

# 4) امسح نتائج HITL كلها
rm -rf results/hitl_tcp/*

# 5) تحقّق أن كل شيء نظيف
ps aux | grep -E "QGround|run_hitl|mavlink" | grep -v grep   # يجب أن يكون فارغ
ls results/hitl_tcp/                                          # يجب أن يكون فارغ
adb shell ps | grep ardophone                                 # يجب أن يكون فارغ
```

## ✅ تأكّد أنّ التنظيف يعمل

شغّل وانظر إلى السطور الأخيرة:

```bash
bash hitl/run_phone_hitl.sh --no-qgc 2>&1 | tail -20
```

يجب أن ترى:
```
[INFO] Cleanup on exit (code=0)
[INFO] Shutting down phone app
[INFO] adb forward --remove-all
[INFO] Pruning old runs: keeping last 5
[INFO] All cleanup completed
```

ثم تأكّد:
```bash
# التطبيق مغلق
adb shell ps | grep ardophone   # لا شيء

# لا python خلفية
pgrep -f run_hitl_tcp           # لا شيء

# لا port forward
adb forward --list              # لا شيء

# عدد النتائج = 5 (الحدّ الأقصى)
ls results/hitl_tcp/ | wc -l    # 5
```
