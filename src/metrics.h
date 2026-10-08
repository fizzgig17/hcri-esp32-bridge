// metrics.h -- on-board colorimetry for the Torch Bearer bridge.
//
// A port of the hCRI Companion app's analyzeSpectrum() (itself a port of
// hCRI.io's spd.php "hires" path): x/y, Ohno CCT/Duv against the integrated
// Planckian locus, and CIE 13.3 Ra / R9. Rf/Rg are not computed here.
// Header-only so the same file can be compiled on a PC for testing.

#pragma once
#include <math.h>
#include <stdint.h>
#include "metrics_tables.h"
#include "corr_table.h"

struct Metrics {
    bool ok;
    float x, y, cct, duv;
    int ra, r9;
};

// ---- correction -----------------------------------------------------------
// 0 = show/send the raw spectrum (default while the curve is still being validated); 1 = apply the fitted curve.
#ifndef TB_APPLY_CORRECTION
#define TB_APPLY_CORRECTION 0
#endif
// Multiplies a raw spectrum (1 nm steps from start_nm) by the HPCS-fitted curve.
static inline void apply_correction(float *spd, int npts, int start_nm) {
#if !TB_APPLY_CORRECTION
    (void)spd; (void)npts; (void)start_nm;
    return;
#endif
    for (int i = 0; i < npts; i++) {
        int k = start_nm + i - CORR_FIRST_NM;
        if (k < 0) k = 0;
        if (k >= CORR_N) k = CORR_N - 1;
        spd[i] *= CORR[k];
    }
}

// ---- helpers --------------------------------------------------------------
namespace tbm {

static const int N = 401;  // 380..780 nm at 1 nm

static inline double interp5(const float *t, double w) {
    if (w <= 380) return t[0];
    if (w >= 780) return t[80];
    double x = (w - 380) / 5.0;
    int k = (int)floor(x);
    double q = x - k;
    return t[k] * (1 - q) + t[k + 1] * q;
}

struct Tabs {
    bool ready = false;
    double X[N], Y[N], Z[N];
    float invl5[N];  // 1/lambda^5 (lambda in um)
    float inv_l[N];  // 1/lambda (um)
};
static Tabs T;
static double test[N], ref[N];

static void init_tabs() {
    if (T.ready) return;
    for (int i = 0; i < N; i++) {
        T.X[i] = interp5(CMF_X, 380 + i);
        T.Y[i] = interp5(CMF_Y, 380 + i);
        T.Z[i] = interp5(CMF_Z, 380 + i);
        double l = (380 + i) * 1e-3;
        T.inv_l[i] = (float)(1.0 / l);
        T.invl5[i] = (float)(1.0 / (l * l * l * l * l));
    }
    T.ready = true;
}

// measured spectrum (1 nm grid starting at start_nm), linear interpolation, flat-clamped
static inline double spd_at(const float *v, int n, int start, double w) {
    if (w <= start) return v[0];
    if (w >= start + n - 1) return v[n - 1];
    double p = w - start;
    int k = (int)floor(p);
    double q = p - k;
    return v[k] * (1 - q) + v[k + 1] * q;
}

struct UV { double u, v; };

static UV planck_uv(double Tk) {
    // c2 = h*c/k with the same constants the app uses; radiance scale cancels in u,v
    const double c2um = 6.626e-34 * 3e8 / 1.381e-23 * 1e6;  // um*K
    double X = 0, Y = 0, Z = 0;
    for (int i = 0; i < N; i++) {
        float e = expf((float)(c2um / Tk) * T.inv_l[i]);
        double val = T.invl5[i] / (e - 1.0f);
        X += val * T.X[i];
        Y += val * T.Y[i];
        Z += val * T.Z[i];
    }
    double s = X + Y + Z, x = X / s, y = Y / s, d = -2 * x + 12 * y + 3;
    return {4 * x / d, 6 * y / d};
}

static double robertson_cct(double x, double y) {
    double d0 = -2 * x + 12 * y + 3, u = 4 * x / d0, v = 6 * y / d0;
    for (int i = 1; i < 31; i++) {
        double di = (v - ROBERTSON[i][2]) - ROBERTSON[i][3] * (u - ROBERTSON[i][1]);
        double pi = (v - ROBERTSON[i - 1][2]) - ROBERTSON[i - 1][3] * (u - ROBERTSON[i - 1][1]);
        if (pi * di <= 0 || i == 30) {
            double f = pi / (pi - di);
            return 1e6 / (ROBERTSON[i - 1][0] + f * (ROBERTSON[i][0] - ROBERTSON[i - 1][0]));
        }
    }
    return 6504.0;
}

struct XYZd { double X, Y, Z; };
static XYZd xyz_of(const double *v) {
    XYZd r = {0, 0, 0};
    for (int i = 0; i < N; i++) { r.X += v[i] * T.X[i]; r.Y += v[i] * T.Y[i]; r.Z += v[i] * T.Z[i]; }
    return r;
}
static UV cri_uv(const XYZd &c) {
    double d = c.X + 15 * c.Y + 3 * c.Z;
    if (d == 0) return {0.2009, 0.3220};
    return {4 * c.X / d, 6 * c.Y / d};
}

static void blackbody(double Tk, double *out) {
    const double h = 6.626e-34, c = 3e8, k = 1.381e-23;
    for (int i = 0; i < N; i++) {
        double m = (380 + i) * 1e-9;
        out[i] = (2 * h * c * c / pow(m, 5)) / (exp(h * c / (m * k * Tk)) - 1);
    }
}

static void daylight(double cct, double *out) {
    double Tt = fmax(4000.0, fmin(25000.0, cct));
    double xD = Tt <= 7000.0 ? -4.6070e9 / (Tt * Tt * Tt) + 2.9678e6 / (Tt * Tt) + 0.09911e3 / Tt + 0.244063
                             : -2.0064e9 / (Tt * Tt * Tt) + 1.9018e6 / (Tt * Tt) + 0.24748e3 / Tt + 0.237040;
    double yD = -3.000 * xD * xD + 2.870 * xD - 0.275;
    double M = 0.0241 + 0.2562 * xD - 0.7341 * yD;
    double M1 = (-1.3515 - 1.7703 * xD + 5.9114 * yD) / M;
    double M2 = (0.0300 - 31.4424 * xD + 30.0717 * yD) / M;
    for (int i = 0; i < N; i++) {
        double idx = (380 + i - 300) / 10.0;
        int j = (int)floor(idx);
        double f = idx - j;
        if (j < 0) { j = 0; f = 0; }
        if (j >= 53) { j = 52; f = 1; }
        out[i] = (DAYLIGHT_S0[j] + f * (DAYLIGHT_S0[j + 1] - DAYLIGHT_S0[j])) +
                 M1 * (DAYLIGHT_S1[j] + f * (DAYLIGHT_S1[j + 1] - DAYLIGHT_S1[j])) +
                 M2 * (DAYLIGHT_S2[j] + f * (DAYLIGHT_S2[j + 1] - DAYLIGHT_S2[j]));
    }
}

static void cri(double cct, int *ra, int *r9) {
    double Tref = fmax(1667.0, fmin(25000.0, cct));
    if (Tref >= 5000.0) daylight(Tref, ref); else blackbody(Tref, ref);
    double rmax = 0;
    for (int i = 0; i < N; i++) if (ref[i] > rmax) rmax = ref[i];
    if (rmax > 0) for (int i = 0; i < N; i++) ref[i] /= rmax;

    XYZd tW = xyz_of(test), rW = xyz_of(ref);
    UV tUV = cri_uv(tW), rUV = cri_uv(rW);
    double u_t = tUV.u, v_t = tUV.v, u_r = rUV.u, v_r = rUV.v;
    double k_t = tW.Y > 0 ? 100.0 / tW.Y : 1.0, k_r = rW.Y > 0 ? 100.0 / rW.Y : 1.0;
    double c_t = v_t > 0 ? (4.0 - u_t - 10.0 * v_t) / v_t : 0.0;
    double d_t = v_t > 0 ? (1.708 * v_t + 0.404 - 1.481 * u_t) / v_t : 0.0;
    double c_r = v_r > 0 ? (4.0 - u_r - 10.0 * v_r) / v_r : 0.0;
    double d_r = v_r > 0 ? (1.708 * v_r + 0.404 - 1.481 * u_r) / v_r : 0.0;

    static double tr[N], rr[N];
    int Ri[16] = {0};
    for (int s = 1; s <= 9; s++) {  // only R1..R9 are needed (Ra = mean R1-8, plus R9)
        const float *tab = TCS_5NM[s - 1];
        for (int i = 0; i < N; i++) {
            double r = interp5(tab, 380 + i);
            tr[i] = r * test[i];
            rr[i] = r * ref[i];
        }
        XYZd tX = xyz_of(tr), rX = xyz_of(rr);
        double tY = tX.Y * k_t, rY = rX.Y * k_r;
        UV tu = cri_uv(tX);
        double c_tcs = tu.v > 0 ? (4.0 - tu.u - 10.0 * tu.v) / tu.v : 0.0;
        double d_tcs = tu.v > 0 ? (1.708 * tu.v + 0.404 - 1.481 * tu.u) / tu.v : 0.0;
        double denom = 16.518 + 1.481 * (c_r / c_t) * c_tcs - (d_r / d_t) * d_tcs;
        double u_a, v_a;
        if (fabs(denom) < 1e-10) { u_a = tu.u; v_a = tu.v; }
        else {
            u_a = (10.872 + 0.404 * (c_r / c_t) * c_tcs - 4.0 * (d_r / d_t) * d_tcs) / denom;
            v_a = 5.52 / denom;
        }
        double W_t = 25.0 * pow(fmax(0.001, tY), 1.0 / 3.0) - 17.0;
        double U_t = 13.0 * W_t * (u_a - u_r), V_t = 13.0 * W_t * (v_a - v_r);
        UV ru = cri_uv(rX);
        double W_r = 25.0 * pow(fmax(0.001, rY), 1.0 / 3.0) - 17.0;
        double U_r = 13.0 * W_r * (ru.u - u_r), V_r = 13.0 * W_r * (ru.v - v_r);
        double dE = sqrt((W_t - W_r) * (W_t - W_r) + (U_t - U_r) * (U_t - U_r) + (V_t - V_r) * (V_t - V_r));
        Ri[s] = (int)floor(100.0 - 4.6 * dE + 0.5);  // JS Math.round
    }
    double sum = 0;
    for (int i = 1; i <= 8; i++) sum += Ri[i];
    *ra = (int)floor(sum / 8.0 + 0.5);
    *r9 = Ri[9];
}

}  // namespace tbm

// ---- entry point ------------------------------------------------------------
// spd: n values on a 1 nm grid starting at start_nm (any scale; normalised here).
static bool compute_metrics(const float *spd, int n, int start_nm, Metrics *out) {
    using namespace tbm;
    *out = {false, 0.3333f, 0.3333f, 0, 0, 0, 0};
    float mxv = 0;
    for (int i = 0; i < n; i++) if (spd[i] > mxv) mxv = spd[i];
    if (n < 2 || !(mxv > 0)) return false;
    init_tabs();

    static float norm[1024];
    if (n > 1024) n = 1024;
    for (int i = 0; i < n; i++) norm[i] = spd[i] / mxv;
    for (int i = 0; i < N; i++) test[i] = spd_at(norm, n, start_nm, 380 + i);

    XYZd w = xyz_of(test);
    double s = w.X + w.Y + w.Z;
    if (!(s > 0)) return false;
    double x = w.X / s, y = w.Y / s;
    double d = -2 * x + 12 * y + 3, u = 4 * x / d, v = 6 * y / d;

    double seed = robertson_cct(x, y);
    seed = fmax(1100.0, fmin(24000.0, seed));
    double bestT = seed, bestD2 = INFINITY;
    for (double Tk = fmax(1000.0, seed - 400.0); Tk <= fmin(25000.0, seed + 400.0); Tk += 10.0) {
        UV p = planck_uv(Tk);
        double dd = (u - p.u) * (u - p.u) + (v - p.v) * (v - p.v);
        if (dd < bestD2) { bestD2 = dd; bestT = Tk; }
    }
    double lo = fmax(1000.0, bestT - 12.0), hi = fmin(25000.0, bestT + 12.0);
    for (double Tk = lo; Tk <= hi; Tk += 0.1) {
        UV p = planck_uv(Tk);
        double dd = (u - p.u) * (u - p.u) + (v - p.v) * (v - p.v);
        if (dd < bestD2) { bestD2 = dd; bestT = Tk; }
    }
    UV p = planck_uv(bestT);
    double duv = sqrt(bestD2);
    if (v - p.v < 0) duv = -duv;

    int ra, r9;
    cri(bestT, &ra, &r9);
    *out = {true, (float)x, (float)y, (float)bestT, (float)duv, ra, r9};
    return true;
}
