/**
 * Stress / Race-Detection Tests — SharedSensorData synchronization pattern.
 *
 * **الفكرة**:
 * ننسخ **نفس نمط الـ synchronization** المستخدم في
 *   `AndroidApp/app/src/main/cpp/native_sensor_reader.cpp`
 *   `AndroidApp/app/src/main/cpp/android_uorb_publishers.cpp`
 * ونُجري stress-testing عليه تحت ThreadSanitizer (TSan) و AddressSanitizer (ASan).
 * إذا TSan يرصد data race في هذا النمط ⇒ نفس race موجود في الكود الإنتاجي.
 *
 * **لماذا لا نُدرج native_sensor_reader.cpp مباشرة؟**
 * لأنه يعتمد على Android NDK (`ASensorManager`, `ALooper`) — لا يُبنى على host.
 * لكن بنية `SharedSensorData` (`shared_sensor_data.h`) هي نفسها في كلا الجانبَين،
 * ونمط الـ lock/atomic identical.
 *
 * **الاختبارات الـ5:**
 *   1. Accel_ConcurrentReaderWriter      — 1 writer + 4 readers، كشف torn reads
 *   2. Gyro_ConcurrentReaderWriter        — مطابق للـ accel
 *   3. Baro_Accumulator_SumCountInvariant — sum == count * value (no lost updates)
 *   4. Mag_Accumulator_InvariantHolds     — مطابق للـ baro
 *   5. IndependentMutexes_DoNotSerialize  — accel_mutex و gyro_mutex لا يحجبان بعضهما
 *
 * Build + Run:
 *   cmake -S . -B build_tsan -DSANITIZER=thread  && cmake --build build_tsan && ./build_tsan/stress_sensor_race
 *   cmake -S . -B build_asan -DSANITIZER=address && cmake --build build_asan && ./build_asan/stress_sensor_race
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>
#include <vector>

#include "shared_sensor_data.h"

// Global instance (يقابل `extern SharedSensorData g_sensor_data;` في px4_jni.cpp)
// لا نضع `static` لأن الـ header يُعلنها extern.
SharedSensorData g_sensor_data;

// --------------------------------------------------------------------------
// Helpers — mirror the exact lock/atomic pattern from production code
// --------------------------------------------------------------------------

// نمط writer: مطابق لـ native_sensor_reader.cpp:87-94
static inline void write_accel(float x, float y, float z) {
    const hrt_abstime receipt_us = hrt_absolute_time();
    std::lock_guard<std::mutex> lock(g_sensor_data.accel_mutex);
    g_sensor_data.accel.data[0] = x;
    g_sensor_data.accel.data[1] = y;
    g_sensor_data.accel.data[2] = z;
    g_sensor_data.accel.timestamp_ns = (int64_t)receipt_us * 1000;
    g_sensor_data.accel.hrt_receipt_us = receipt_us;
    g_sensor_data.accel.has_new_data.store(true);
}

static inline void write_gyro(float x, float y, float z) {
    const hrt_abstime receipt_us = hrt_absolute_time();
    std::lock_guard<std::mutex> lock(g_sensor_data.gyro_mutex);
    g_sensor_data.gyro.data[0] = x;
    g_sensor_data.gyro.data[1] = y;
    g_sensor_data.gyro.data[2] = z;
    g_sensor_data.gyro.timestamp_ns = (int64_t)receipt_us * 1000;
    g_sensor_data.gyro.hrt_receipt_us = receipt_us;
    g_sensor_data.gyro.has_new_data.store(true);
}

// نمط reader: مطابق لـ android_uorb_publishers.cpp:115-137
// يُرجع true إذا حصل على بيانات طازجة
static inline bool read_accel_snapshot(float out[3]) {
    if (!g_sensor_data.accel.has_new_data.load()) return false;
    std::lock_guard<std::mutex> lock(g_sensor_data.accel_mutex);
    std::memcpy(out, g_sensor_data.accel.data, sizeof(float) * 3);
    g_sensor_data.accel.has_new_data.store(false);
    return true;
}

static inline bool read_gyro_snapshot(float out[3]) {
    if (!g_sensor_data.gyro.has_new_data.load()) return false;
    std::lock_guard<std::mutex> lock(g_sensor_data.gyro_mutex);
    std::memcpy(out, g_sensor_data.gyro.data, sizeof(float) * 3);
    g_sensor_data.gyro.has_new_data.store(false);
    return true;
}


// ==========================================================================
// Test 1 — Accel: Concurrent Reader/Writer
//
// Writer يكتب بأنماط مُنضبطة: frame i → (i, 2i, 3i). قراءة صحيحة تحترم
// a[1] == 2*a[0] && a[2] == 3*a[0]. أي torn read (مزج مكوّنات من frames
// مختلفة) يكسر هذا الـ invariant.
// ==========================================================================
TEST(SharedSensorData, Accel_ConcurrentReaderWriter) {
    std::atomic<bool> stop{false};
    std::atomic<int> torn_reads{0};
    std::atomic<uint64_t> total_reads{0};

    // 4 readers يتحقّقون من الـ invariant
    std::vector<std::thread> readers;
    for (int i = 0; i < 4; i++) {
        readers.emplace_back([&]{
            float a[3];
            while (!stop.load(std::memory_order_relaxed)) {
                if (read_accel_snapshot(a)) {
                    total_reads.fetch_add(1, std::memory_order_relaxed);
                    // invariant: a[1] == 2*a[0], a[2] == 3*a[0] (ضمن تقريب float)
                    if (std::fabs(a[1] - 2.0f * a[0]) > 1e-3f ||
                        std::fabs(a[2] - 3.0f * a[0]) > 1e-3f) {
                        torn_reads.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }
        });
    }

    // Writer thread
    std::thread writer([&]{
        for (int i = 1; i <= 100000 && !stop.load(); i++) {
            float v = (float)(i % 10000) + 0.5f;  // تجنّب صفر
            write_accel(v, 2.0f * v, 3.0f * v);
        }
    });

    writer.join();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    stop.store(true);
    for (auto& t : readers) t.join();

    // TSan سيُفشل البرنامج قبل أن نصل إلى هنا إذا رصد race
    EXPECT_EQ(torn_reads.load(), 0)
        << "Torn reads detected: readers saw inconsistent frames. "
        << "total_reads=" << total_reads.load();
    EXPECT_GT(total_reads.load(), 1000u)
        << "Too few reads — readers may be starved";
}


// ==========================================================================
// Test 2 — Gyro: Concurrent Reader/Writer (نفس النمط)
// ==========================================================================
TEST(SharedSensorData, Gyro_ConcurrentReaderWriter) {
    std::atomic<bool> stop{false};
    std::atomic<int> torn_reads{0};
    std::atomic<uint64_t> total_reads{0};

    std::vector<std::thread> readers;
    for (int i = 0; i < 4; i++) {
        readers.emplace_back([&]{
            float a[3];
            while (!stop.load(std::memory_order_relaxed)) {
                if (read_gyro_snapshot(a)) {
                    total_reads.fetch_add(1, std::memory_order_relaxed);
                    if (std::fabs(a[1] - 2.0f * a[0]) > 1e-3f ||
                        std::fabs(a[2] + 3.0f * a[0]) > 1e-3f) {  // note sign diff
                        torn_reads.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }
        });
    }

    std::thread writer([&]{
        for (int i = 1; i <= 100000 && !stop.load(); i++) {
            float v = (float)(i % 10000) + 0.5f;
            write_gyro(v, 2.0f * v, -3.0f * v);
        }
    });

    writer.join();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    stop.store(true);
    for (auto& t : readers) t.join();

    EXPECT_EQ(torn_reads.load(), 0);
    EXPECT_GT(total_reads.load(), 1000u);
}


// ==========================================================================
// Test 3 — Baro Accumulator: sum/count invariant
//
// writer يُضيف `v` إلى sum ويُزيد count تحت lock (كما في
// native_sensor_reader.cpp:122-127). reader يقرأ (sum, count) ثم يُصفّرهما
// تحت lock (كما في android_uorb_publishers.cpp:164-182).
// invariant: عند كل قراءة، sum == count * v (لأن writer يستخدم قيمة ثابتة).
// ==========================================================================
TEST(SharedSensorData, Baro_Accumulator_SumCountInvariant) {
    const float V = 101325.0f;  // 1 atm Pa
    std::atomic<bool> stop{false};
    std::atomic<int> invariant_breaks{0};
    std::atomic<uint64_t> total_snapshots{0};
    std::atomic<uint64_t> total_samples_consumed{0};

    // 2 writers (يحاكي baro + baro-duplicate لاختبار stress متعدد الكتّاب)
    std::vector<std::thread> writers;
    for (int w = 0; w < 2; w++) {
        writers.emplace_back([&]{
            for (int i = 0; i < 50000 && !stop.load(); i++) {
                std::lock_guard<std::mutex> lock(g_sensor_data.baro_mutex);
                g_sensor_data.baro.sum_pressure += V;
                g_sensor_data.baro.sum_temperature += 25.0f;
                g_sensor_data.baro.count++;
            }
        });
    }

    // 1 reader consumes + resets
    std::thread reader([&]{
        while (!stop.load(std::memory_order_relaxed)) {
            std::lock_guard<std::mutex> lock(g_sensor_data.baro_mutex);
            if (g_sensor_data.baro.count > 0) {
                uint32_t c = g_sensor_data.baro.count;
                float s = g_sensor_data.baro.sum_pressure;
                total_snapshots.fetch_add(1);
                total_samples_consumed.fetch_add(c);
                // invariant: sum == count * V (ضمن تقريب float)
                float expected = (float)c * V;
                if (std::fabs(s - expected) > 1e-2f * expected) {
                    invariant_breaks.fetch_add(1);
                }
                g_sensor_data.baro.sum_pressure = 0;
                g_sensor_data.baro.sum_temperature = 0;
                g_sensor_data.baro.count = 0;
            }
        }
    });

    for (auto& t : writers) t.join();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    stop.store(true);
    reader.join();

    // Drain النهائي للتأكد من عدم فقدان عيّنات
    {
        std::lock_guard<std::mutex> lock(g_sensor_data.baro_mutex);
        if (g_sensor_data.baro.count > 0) {
            total_samples_consumed.fetch_add(g_sensor_data.baro.count);
            g_sensor_data.baro.count = 0;
            g_sensor_data.baro.sum_pressure = 0;
        }
    }

    EXPECT_EQ(invariant_breaks.load(), 0)
        << "sum/count invariant violated " << invariant_breaks.load() << " times "
        << "across " << total_snapshots.load() << " snapshots";
    // 2 writers × 50000 samples = 100000 total. يجب أن نستهلكها كلها.
    EXPECT_EQ(total_samples_consumed.load(), 100000u)
        << "Lost samples! writer pushed 100k but reader saw only "
        << total_samples_consumed.load();
}


// ==========================================================================
// Test 4 — Mag Accumulator (مطابق للـ baro)
// ==========================================================================
TEST(SharedSensorData, Mag_Accumulator_InvariantHolds) {
    const float MX = 250.0f, MY = -150.0f, MZ = 450.0f;
    std::atomic<bool> stop{false};
    std::atomic<int> invariant_breaks{0};
    std::atomic<uint64_t> total_samples_consumed{0};

    std::vector<std::thread> writers;
    for (int w = 0; w < 2; w++) {
        writers.emplace_back([&]{
            for (int i = 0; i < 50000 && !stop.load(); i++) {
                std::lock_guard<std::mutex> lock(g_sensor_data.mag_mutex);
                g_sensor_data.mag.sum_field[0] += MX;
                g_sensor_data.mag.sum_field[1] += MY;
                g_sensor_data.mag.sum_field[2] += MZ;
                g_sensor_data.mag.count++;
            }
        });
    }

    std::thread reader([&]{
        while (!stop.load(std::memory_order_relaxed)) {
            std::lock_guard<std::mutex> lock(g_sensor_data.mag_mutex);
            if (g_sensor_data.mag.count > 0) {
                uint32_t c = g_sensor_data.mag.count;
                float sx = g_sensor_data.mag.sum_field[0];
                total_samples_consumed.fetch_add(c);
                float expected_sx = (float)c * MX;
                if (std::fabs(sx - expected_sx) > 1e-2f * std::fabs(expected_sx)) {
                    invariant_breaks.fetch_add(1);
                }
                g_sensor_data.mag.sum_field[0] = 0;
                g_sensor_data.mag.sum_field[1] = 0;
                g_sensor_data.mag.sum_field[2] = 0;
                g_sensor_data.mag.count = 0;
            }
        }
    });

    for (auto& t : writers) t.join();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    stop.store(true);
    reader.join();

    {
        std::lock_guard<std::mutex> lock(g_sensor_data.mag_mutex);
        total_samples_consumed.fetch_add(g_sensor_data.mag.count);
        g_sensor_data.mag.count = 0;
        g_sensor_data.mag.sum_field[0] = 0;
        g_sensor_data.mag.sum_field[1] = 0;
        g_sensor_data.mag.sum_field[2] = 0;
    }

    EXPECT_EQ(invariant_breaks.load(), 0);
    EXPECT_EQ(total_samples_consumed.load(), 100000u);
}


// ==========================================================================
// Test 5 — Independence: mutexes لا يحجبان بعضهما البعض
//
// نتحقّق أن kaccel_mutex لا يُسلسل writes إلى gyro (خصّيصُا إذا دمجنا mutex
// واحد بالخطأ في المستقبل، هذا الاختبار يفشل).
// ==========================================================================
TEST(SharedSensorData, IndependentMutexes_DoNotSerialize) {
    using namespace std::chrono;

    // Writer يمسك accel_mutex لفترة طويلة (200ms)
    std::atomic<bool> accel_held{false};
    std::atomic<bool> release{false};
    std::thread accel_holder([&]{
        std::lock_guard<std::mutex> lock(g_sensor_data.accel_mutex);
        accel_held.store(true);
        while (!release.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    });

    // ننتظر أن يأخذ الـ accel holder القفل
    while (!accel_held.load()) std::this_thread::yield();

    // الآن نقيس: هل writer gyro قادر على التقدّم؟
    auto t0 = steady_clock::now();
    for (int i = 0; i < 1000; i++) {
        write_gyro((float)i, 2.0f * i, -3.0f * i);
    }
    auto t1 = steady_clock::now();
    auto elapsed_ms = duration_cast<milliseconds>(t1 - t0).count();

    release.store(true);
    accel_holder.join();

    // 1000 write_gyro calls يجب أن تُنهى في أقلّ من 50ms (ربما 1-5ms)
    // لو كان mutex مشترك (bug مستقبلي)، ستحتاج 200ms+ بسبب انتظار accel_mutex
    EXPECT_LT(elapsed_ms, 50)
        << "Gyro writes took " << elapsed_ms
        << " ms while accel lock held 200ms — mutexes may have been accidentally merged";
}


// ==========================================================================
// Test 6 — SANITY: negative control — deliberately racy code (enabled via
//          -DPROVOKE_RACE=1). يجب أن يفشل تحت TSan. لا يُبنى افتراضيًا.
//
// هدف هذا الاختبار: إثبات أن TSan في هذا البناء **يكشف** الـ races فعلاً،
// وليس يتجاهلها بهدوء. إذا أضفنا في المستقبل flag يُعطّل TSan بالخطأ،
// الـ negative control يكشف ذلك.
// ==========================================================================
#ifdef PROVOKE_RACE
// متغيّر atomic نجمع فيه القيم المقروءة — يمنع الـ compiler من حذف القراءات
static std::atomic<float> g_reader_sink{0.0f};

TEST(SharedSensorData, SANITY_UnsynchronizedAccessDetected) {
    // نصل للـ data بدون lock عمدًا ⇒ TSan يجب أن يرصد data race
    std::atomic<bool> stop{false};
    std::thread writer([&]{
        for (int i = 0; i < 100000 && !stop.load(); i++) {
            g_sensor_data.accel.data[0] = (float)i;  // NO LOCK
            g_sensor_data.accel.data[1] = (float)(2 * i);
            g_sensor_data.accel.data[2] = (float)(3 * i);
        }
    });
    std::thread reader([&]{
        for (int i = 0; i < 100000 && !stop.load(); i++) {
            // القراءة تُخزَّن في atomic sink — الـ compiler لا يستطيع حذفها
            float a0 = g_sensor_data.accel.data[0];  // NO LOCK
            float a1 = g_sensor_data.accel.data[1];
            float a2 = g_sensor_data.accel.data[2];
            g_reader_sink.store(a0 + a1 + a2, std::memory_order_relaxed);
        }
    });
    writer.join();
    reader.join();
    // إذا وصلنا هنا بدون TSan abort، فـ TSan معطّل!
    FAIL() << "TSan did NOT abort on deliberate data race — sanitizer is disabled. "
           << "sink=" << g_reader_sink.load();
}
#endif


// ==========================================================================
// main
// ==========================================================================
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    int rc = RUN_ALL_TESTS();

    // طباعة ملخّص للإشارة الواضحة في CI
    if (rc == 0) {
        printf("\n[stress_sensor_race] ALL TESTS PASSED\n");
    } else {
        printf("\n[stress_sensor_race] FAILURES DETECTED (rc=%d)\n", rc);
    }
    return rc;
}
