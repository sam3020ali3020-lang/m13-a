/**
 * Numerical validation: compare C++ weight scheduling, LOS guidance,
 * params computation, and fin mixing against Python test vectors.
 *
 * Build: g++ -std=c++17 -O2 -o validate validate_cpp.cpp -lm
 * Run:   ./validate test_vectors.json
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

// ===============================================================
// Minimal JSON parser (no dependencies)
// ===============================================================
struct JVal {
    enum Type { NUL, NUM, BOOL, STR, ARR, OBJ } type = NUL;
    double num = 0;
    bool bval = false;
    std::string str;
    std::vector<JVal> arr;
    std::vector<std::pair<std::string,JVal>> obj;

    double get_num(const char *key) const {
        for (auto &p : obj) if (p.first == key) return p.second.num;
        return 0;
    }
    bool get_bool(const char *key) const {
        for (auto &p : obj) if (p.first == key) return p.second.bval;
        return false;
    }
    const JVal *get_obj(const char *key) const {
        for (auto &p : obj) if (p.first == key) return &p.second;
        return nullptr;
    }
    const JVal *get_arr(const char *key) const {
        for (auto &p : obj) if (p.first == key && p.second.type == ARR) return &p.second;
        return nullptr;
    }
};

static void skip_ws(const char *&p) { while (*p==' '||*p=='\t'||*p=='\n'||*p=='\r') p++; }

static JVal parse_json(const char *&p) {
    skip_ws(p);
    JVal v;
    if (*p == '"') {
        p++;
        while (*p && *p != '"') { if (*p == '\\') { v.str += *++p; } else v.str += *p; p++; }
        if (*p == '"') p++;
        v.type = JVal::STR;
    } else if (*p == '{') {
        v.type = JVal::OBJ; p++;
        while (true) {
            skip_ws(p); if (*p == '}') { p++; break; }
            if (*p == ',') p++;
            skip_ws(p);
            JVal key = parse_json(p);
            skip_ws(p); if (*p == ':') p++;
            JVal val = parse_json(p);
            v.obj.push_back({key.str, val});
        }
    } else if (*p == '[') {
        v.type = JVal::ARR; p++;
        while (true) {
            skip_ws(p); if (*p == ']') { p++; break; }
            if (*p == ',') p++;
            v.arr.push_back(parse_json(p));
        }
    } else if (*p == 't') { v.type = JVal::BOOL; v.bval = true; p += 4;
    } else if (*p == 'f') { v.type = JVal::BOOL; v.bval = false; p += 5;
    } else if (*p == 'n') { v.type = JVal::NUL; p += 4;
    } else {
        v.type = JVal::NUM;
        char *end;
        v.num = strtod(p, &end);
        p = end;
    }
    return v;
}

// ===============================================================
// Pure C++ reimplementation of the functions under test
// (copied from mpc_controller.cpp and los_guidance.cpp)
// ===============================================================

static inline float hermite(float s) {
    s = (s < 0.0f) ? 0.0f : ((s > 1.0f) ? 1.0f : s);
    return 3.0f * s * s - 2.0f * s * s * s;
}

static constexpr float G_ACCEL = 9.80665f;
static constexpr float DEG2RAD = (float)M_PI / 180.0f;

// --- get_params ---
void get_params(float t, float burn_time, float mass_full, float mass_dry,
                float thrust_plateau, float t_tail, float &mass, float &thrust) {
    if (t < burn_time) {
        float f = t / burn_time;
        mass = mass_full - f * (mass_full - mass_dry);
        float t_to_start = burn_time - t_tail;
        if (t > t_to_start) {
            float s = (t - t_to_start) / t_tail;
            thrust = thrust_plateau * (1.0f - s) * (1.0f - s) * (1.0f - s);
        } else {
            thrust = thrust_plateau;
        }
    } else {
        mass = mass_dry;
        thrust = 0.0f;
    }
}

// --- LOS guidance ---
struct LosResult {
    float gamma_ref, chi_ref, dx_safe;
};

LosResult compute_los(float x_pos, float y_pos, float altitude,
                      float target_x, float target_h,
                      float impact_angle_deg, float impact_blend_start, float impact_blend_end,
                      float burn_time, float cruise_progress, float gamma_natural_rad,
                      float t_flight, float gamma_ref_prev, float chi_ref_prev, float dt,
                      bool cruise_alt_set, float cruise_alt_target) {
    float dx = target_x - x_pos;
    float dh = target_h - altitude;
    float dy = -y_pos;

    float range_to_target = sqrtf(dx * dx + dy * dy);
    float dx_safe = (dx > 0.5f) ? dx : 0.5f;
    float range_safe = (range_to_target > 1.0f) ? range_to_target : 1.0f;

    float gamma_los = atan2f(dh, range_safe);
    if (dx < 0.0f) {
        if (gamma_los < -60.0f * DEG2RAD) gamma_los = -60.0f * DEG2RAD;
    }

    float impact_rad = impact_angle_deg * DEG2RAD;
    float progress = 1.0f - dx_safe / ((target_x > 1.0f) ? target_x : 1.0f);
    progress = (progress < 0.0f) ? 0.0f : ((progress > 1.0f) ? 1.0f : progress);

    float k_impact;
    if (progress < impact_blend_start) k_impact = 0.0f;
    else if (progress > impact_blend_end) k_impact = 1.0f;
    else k_impact = hermite((progress - impact_blend_start) / (impact_blend_end - impact_blend_start));

    float gamma_ref_los = (1.0f - k_impact) * gamma_los + k_impact * impact_rad;
    if (gamma_ref_los < -45.0f * DEG2RAD) gamma_ref_los = -45.0f * DEG2RAD;
    if (gamma_ref_los >  15.0f * DEG2RAD) gamma_ref_los =  15.0f * DEG2RAD;

    float chi_ref_los = atan2f(dy, dx_safe);

    float t_rel_burn = t_flight - (burn_time - 0.5f);
    float s_burn = 0.5f * (1.0f + tanhf(t_rel_burn / 0.8f));

    float s_dive;
    float cp0 = cruise_progress, cp1 = cp0 + 0.10f;
    if (cp1 > 0.95f) cp1 = 0.95f;
    if (progress < cp0) s_dive = 0.0f;
    else if (progress > cp1) s_dive = 1.0f;
    else s_dive = hermite((progress - cp0) / (cp1 - cp0));

    float gamma_cruise = s_dive * gamma_ref_los;
    float gamma_ref_raw = (1.0f - s_burn) * gamma_natural_rad + s_burn * gamma_cruise;
    float chi_ref_raw = chi_ref_los;

    float MAX_GREF_RATE = 10.0f * DEG2RAD;
    float dt_ref = (dt > 1e-3f) ? dt : 1e-3f;
    float max_dg = MAX_GREF_RATE * dt_ref;

    float dg = gamma_ref_raw - gamma_ref_prev;
    float dc = chi_ref_raw - chi_ref_prev;
    if (dg >  max_dg) dg =  max_dg;
    if (dg < -max_dg) dg = -max_dg;
    if (dc >  max_dg) dc =  max_dg;
    if (dc < -max_dg) dc = -max_dg;

    LosResult r;
    r.gamma_ref = gamma_ref_prev + dg;
    r.chi_ref = chi_ref_prev + dc;
    r.dx_safe = dx_safe;
    return r;
}

// --- Weight scheduling ---
void compute_weights(float t, float gamma, float gamma_ref_prev,
                     float phi, float alpha, float q_rate, float x_pos,
                     float burn_time, float t_tail, float target_x, float cruise_progress,
                     bool cruise_alt_set, float cruise_alt_target, float H_SCALE,
                     float W[12], float W_e[9]) {
    bool is_boost = (t < burn_time);
    float t_tailoff_start = burn_time - t_tail;
    float t_tailoff_end = burn_time + 2.0f * t_tail;
    bool in_tailoff = (t_tailoff_start < t) && (t < t_tailoff_end);

    float gamma_base, q_w, r_w, de_rate_w, dr_rate_w, alpha_w_boost;

    if (is_boost && !in_tailoff) {
        gamma_base = 250.0f; q_w = 200.0f; r_w = 80.0f;
        de_rate_w = 120.0f; dr_rate_w = 60.0f; alpha_w_boost = 120.0f;
    } else if (in_tailoff) {
        float s_to = (t - t_tailoff_start) / (t_tailoff_end - t_tailoff_start);
        if (s_to > 1.0f) s_to = 1.0f;
        float s_peak = 0.67f;
        float s_up = s_to / s_peak;
        if (s_up > 1.0f) s_up = 1.0f;

        float gamma_act = 250.0f * (1.0f - s_up) * (1.0f - s_up) + 2.0f;
        float q_act = 200.0f + 800.0f * s_up;
        float r_act = 80.0f + 200.0f * s_up;
        float de_act = 120.0f * (1.0f - s_up) + 200.0f * s_up;
        float dr_act = 60.0f + 100.0f * s_up;
        float alpha_act = 120.0f + 800.0f * s_up;

        if (s_to > s_peak) {
            float s_f = (s_to - s_peak) / (1.0f - s_peak);
            float h = hermite(s_f);
            gamma_base = gamma_act + (100.0f - gamma_act) * h;
            q_w = q_act + (60.0f - q_act) * h;
            r_w = r_act + (40.0f - r_act) * h;
            de_rate_w = de_act + (20.0f - de_act) * h;
            dr_rate_w = dr_act + (20.0f - dr_act) * h;
            alpha_w_boost = alpha_act + (40.0f - alpha_act) * h;
        } else {
            gamma_base = gamma_act; q_w = q_act; r_w = r_act;
            de_rate_w = de_act; dr_rate_w = dr_act; alpha_w_boost = alpha_act;
        }
    } else {
        gamma_base = 100.0f; q_w = 60.0f; r_w = 40.0f;
        de_rate_w = 20.0f; dr_rate_w = 20.0f; alpha_w_boost = 40.0f;
    }

    // Match m130_mpc_autopilot.py:644 — tracking error, not |gamma|
    float gamma_err_deg = fabsf((gamma - gamma_ref_prev) * 180.0f / (float)M_PI);
    float alpha_deg_abs = fabsf(alpha * 180.0f / (float)M_PI);
    float q_rate_abs = fabsf(q_rate * 180.0f / (float)M_PI);

    if (is_boost && !in_tailoff) {
        if (gamma_err_deg < 5.0f && q_rate_abs > 3.0f) {
            q_w = q_w + 300.0f * (1.0f - gamma_err_deg / 5.0f);
        }
        if (alpha_deg_abs > 1.0f) {
            float a_sc = alpha_deg_abs / 3.0f;
            alpha_w_boost = alpha_w_boost + 200.0f * ((a_sc > 3.0f) ? 3.0f : a_sc);
        }
    }

    float dx_to_target = target_x - x_pos;
    if (dx_to_target < 0.5f) dx_to_target = 0.5f;
    float progress = 1.0f - dx_to_target / ((target_x > 1.0f) ? target_x : 1.0f);
    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;

    float s_dive;
    float p0 = cruise_progress, p1 = p0 + 0.10f;
    if (p1 > 0.95f) p1 = 0.95f;
    if (progress < p0) s_dive = 0.0f;
    else if (progress > p1) s_dive = 1.0f;
    else s_dive = hermite((progress - p0) / (p1 - p0));

    float h_cruise_w = 0.0f;
    if (!is_boost && cruise_alt_set) {
        float t_since_burn = t - burn_time;
        float early_boost = 1.0f - t_since_burn / 3.0f;
        if (early_boost < 0.0f) early_boost = 0.0f;
        h_cruise_w = (15.0f + 35.0f * early_boost) * (1.0f - s_dive);
    }

    float roll_recovery_start = 5.0f * DEG2RAD;
    float roll_recovery_full = 20.0f * DEG2RAD;
    float roll_abs = fabsf(phi);
    float roll_recovery;
    if (roll_abs <= roll_recovery_start) roll_recovery = 0.0f;
    else if (roll_abs >= roll_recovery_full) roll_recovery = 1.0f;
    else roll_recovery = (roll_abs - roll_recovery_start) / (roll_recovery_full - roll_recovery_start);

    float chi_w = 120.0f * (1.0f - 0.55f * roll_recovery);
    if (chi_w < 25.0f) chi_w = 25.0f;
    float p_w = 20.0f + 160.0f * roll_recovery;
    float phi_w = 80.0f + 520.0f * roll_recovery;
    float da_rate_w = 10.0f - 7.0f * roll_recovery;

    W[0] = h_cruise_w;
    W[1] = gamma_base + 400.0f;
    W[2] = chi_w;
    W[3] = p_w;
    W[4] = q_w;
    W[5] = r_w;
    W[6] = alpha_w_boost;
    W[7] = 60.0f;
    W[8] = phi_w;
    W[9] = de_rate_w;
    W[10] = dr_rate_w;
    W[11] = da_rate_w;

    W_e[0] = h_cruise_w * 0.5f;
    W_e[1] = 300.0f;
    W_e[2] = (40.0f > chi_w) ? 40.0f : chi_w;
    W_e[3] = (40.0f > p_w) ? 40.0f : p_w;
    W_e[4] = 60.0f;
    W_e[5] = 40.0f;
    W_e[6] = 30.0f;
    W_e[7] = 40.0f;
    W_e[8] = (40.0f > phi_w) ? 40.0f : phi_w;
}

// --- Fin mixing ---
void compute_fin_mixing(float de, float dr, float da, float fins[4]) {
    fins[0] = da - de - dr;
    fins[1] = da - de + dr;
    fins[2] = da + de + dr;
    fins[3] = da + de - dr;
}

// ===============================================================
// Test runner
// ===============================================================
int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s test_vectors.json\n", argv[0]);
        return 1;
    }

    // Read JSON file
    std::ifstream ifs(argv[1]);
    if (!ifs) { printf("Cannot open %s\n", argv[1]); return 1; }
    std::string json_str((std::istreambuf_iterator<char>(ifs)),
                          std::istreambuf_iterator<char>());
    const char *p = json_str.c_str();
    JVal root = parse_json(p);

    const JVal *cfg_j = root.get_obj("config");
    float burn_time = (float)cfg_j->get_num("burn_time");
    float mass_full = (float)cfg_j->get_num("mass_full");
    float mass_dry = (float)cfg_j->get_num("mass_dry");
    float thrust_plateau = (float)cfg_j->get_num("thrust_plateau");
    float t_tail = (float)cfg_j->get_num("t_tail");
    float target_x = (float)cfg_j->get_num("target_x");
    float target_h = (float)cfg_j->get_num("target_h");
    float impact_angle_deg = (float)cfg_j->get_num("impact_angle_deg");
    float impact_blend_start = (float)cfg_j->get_num("impact_blend_start");
    float impact_blend_end = (float)cfg_j->get_num("impact_blend_end");
    float cruise_progress = (float)cfg_j->get_num("cruise_progress");
    float gamma_natural_rad = (float)cfg_j->get_num("gamma_natural_rad");
    float H_SCALE = (float)cfg_j->get_num("H_SCALE");

    const JVal *tests = root.get_arr("tests");
    if (!tests) { printf("No 'tests' array\n"); return 1; }

    int pass = 0, fail = 0;
    float max_err = 0.0f;

    for (size_t i = 0; i < tests->arr.size(); i++) {
        const JVal &tc = tests->arr[i];
        const JVal *inp = tc.get_obj("inputs");
        const JVal *exp = tc.get_obj("expected");
        std::string name;
        for (auto &p : tc.obj) if (p.first == "name") name = p.second.str;

        float t = (float)inp->get_num("t");
        float x_pos = (float)inp->get_num("x_pos");
        float y_pos = (float)inp->get_num("y_pos");
        float alt = (float)inp->get_num("altitude");
        float gamma = (float)inp->get_num("gamma");
        float phi = (float)inp->get_num("phi");
        float alpha = (float)inp->get_num("alpha");
        float q_rate = (float)inp->get_num("q_rate");
        float gref_prev = (float)inp->get_num("gamma_ref_prev");
        float cref_prev = (float)inp->get_num("chi_ref_prev");
        float dt = (float)inp->get_num("dt");
        bool cruise_alt_set_v = inp->get_bool("cruise_alt_set");
        float cruise_alt_target_v = (float)inp->get_num("cruise_alt_target");
        float de = (float)inp->get_num("de");
        float dr = (float)inp->get_num("dr");
        float da = (float)inp->get_num("da");

        bool tc_pass = true;
        auto check = [&](const char *field, float got, float expected, float tol) {
            float err = fabsf(got - expected);
            if (err > max_err) max_err = err;
            if (err > tol) {
                printf("  FAIL %s.%s: got=%.8f expected=%.8f err=%.2e\n",
                       name.c_str(), field, got, expected, err);
                tc_pass = false;
            }
        };

        // 1. Params
        float mass_c, thrust_c;
        get_params(t, burn_time, mass_full, mass_dry, thrust_plateau, t_tail,
                   mass_c, thrust_c);
        check("mass", mass_c, (float)exp->get_num("mass"), 1e-3f);
        check("thrust", thrust_c, (float)exp->get_num("thrust"), 1e-1f);

        // 2. LOS
        LosResult los = compute_los(x_pos, y_pos, alt, target_x, target_h,
                                    impact_angle_deg, impact_blend_start, impact_blend_end,
                                    burn_time, cruise_progress, gamma_natural_rad,
                                    t, gref_prev, cref_prev, dt,
                                    cruise_alt_set_v, cruise_alt_target_v);
        check("gamma_ref", los.gamma_ref, (float)exp->get_num("gamma_ref"), 1e-5f);
        check("chi_ref", los.chi_ref, (float)exp->get_num("chi_ref"), 1e-5f);
        check("dx_safe", los.dx_safe, (float)exp->get_num("dx_safe"), 1e-3f);

        // 3. Weights
        float W_c[12], W_e_c[9];
        compute_weights(t, gamma, los.gamma_ref, phi, alpha, q_rate, x_pos,
                       burn_time, t_tail, target_x, cruise_progress,
                       cruise_alt_set_v, cruise_alt_target_v, H_SCALE,
                       W_c, W_e_c);
        const JVal *W_exp = exp->get_arr("W");
        const JVal *We_exp = exp->get_arr("W_e");
        for (int j = 0; j < 12; j++) {
            char buf[32]; snprintf(buf, sizeof(buf), "W[%d]", j);
            check(buf, W_c[j], (float)W_exp->arr[j].num, 1e-2f);
        }
        for (int j = 0; j < 9; j++) {
            char buf[32]; snprintf(buf, sizeof(buf), "W_e[%d]", j);
            check(buf, W_e_c[j], (float)We_exp->arr[j].num, 1e-2f);
        }

        // 4. Fin mixing
        float fins_c[4];
        compute_fin_mixing(de, dr, da, fins_c);
        const JVal *fins_exp = exp->get_arr("fins");
        for (int j = 0; j < 4; j++) {
            char buf[16]; snprintf(buf, sizeof(buf), "fin[%d]", j);
            check(buf, fins_c[j], (float)fins_exp->arr[j].num, 1e-6f);
        }

        if (tc_pass) {
            pass++;
            printf("  PASS %s\n", name.c_str());
        } else {
            fail++;
        }
    }

    printf("\n========================================\n");
    printf("Results: %d PASS, %d FAIL (max error: %.2e)\n", pass, fail, max_err);
    printf("========================================\n");

    return fail > 0 ? 1 : 0;
}
