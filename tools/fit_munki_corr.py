#!/usr/bin/env python3
"""Fits the ColorMunki spectral correction (src/munki_corr.h) against an HPCS.

Measure the same lamp, in the same spot, with the ColorMunki (through the bridge) and with the HPCS, using the
hCRI Companion app, and share each reading as CSV from History. Use firmware whose src/munki_corr.h still has
MK_CORR_FITTED 0, so the ColorMunki readings are uncorrected. Several lamps of different types and CCTs (warm and
cool LEDs, an incandescent) give a better fit than many readings of one lamp.

    tools/fit_munki_corr.py --pair munki_lamp1.csv hpcs_lamp1.csv --pair munki_lamp2.csv hpcs_lamp2.csv ...
    tools/fit_munki_corr.py ... --write        # also writes src/munki_corr.h

It prints, for every lamp, the CCT / Duv difference from the HPCS before and after correction, and with three or
more lamps a leave-one-out check (each lamp corrected with a curve fitted without it), which is the honest estimate
of how much the correction helps on lamps it has not seen. The correction is shape only: it is scaled so lux stays
the same.
"""
import argparse, math, re, sys, datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
NM = list(range(380, 731))          # the ColorMunki output grid (1 nm)


def cmfs():
    t = (ROOT / 'src' / 'metrics_tables.h').read_text()
    out = []
    for n in ('CMF_X', 'CMF_Y', 'CMF_Z'):
        m = re.search(r'%s\[81\] = \{([^}]*)\}' % n, t)
        out.append([float(v) for v in m.group(1).replace('\n', '').split(',') if v.strip()])
    return out                       # 380..780 nm at 5 nm


CMF = cmfs()


def cmf_at(k, nm):
    x = (nm - 380) / 5.0
    i = min(max(int(math.floor(x)), 0), 79)
    q = x - i
    return CMF[k][i] * (1 - q) + CMF[k][i + 1] * q


def read_csv(path):
    pts = []
    started = False
    for line in Path(path).read_text().splitlines():
        m = re.match(r'^\s*(\d+(?:\.\d+)?)\s*,\s*(-?[\d.eE+-]+)\s*$', line)
        if m:
            pts.append((float(m.group(1)), float(m.group(2))))
            started = True
        elif started and not line.strip():
            break                    # the share CSV's summary comes after a blank line
    if len(pts) < 10:
        sys.exit('%s: no spectrum found (expected "nm,value" rows)' % path)
    return pts


def resample(pts):
    out = []
    j = 0
    for nm in NM:
        while j + 1 < len(pts) and pts[j + 1][0] < nm:
            j += 1
        if nm <= pts[0][0]:
            out.append(pts[0][1] if nm == pts[0][0] else float('nan'))
        elif j + 1 >= len(pts):
            out.append(pts[-1][1] if nm == pts[-1][0] else float('nan'))
        else:
            (a, va), (b, vb) = pts[j], pts[j + 1]
            out.append(va + (vb - va) * (nm - a) / (b - a))
    return out


def xyz(s):
    X = Y = Z = 0.0
    for nm, v in zip(NM, s):
        if v == v:
            X += v * cmf_at(0, nm); Y += v * cmf_at(1, nm); Z += v * cmf_at(2, nm)
    return X, Y, Z


def cct_duv(s):
    """CCT (Robertson-free Ohno-style search on the Planckian locus in uv) and Duv; good to ~1 K for this purpose."""
    X, Y, Z = xyz(s)
    d = X + 15 * Y + 3 * Z
    u, v = 4 * X / d, 6 * Y / d

    def planck_uv(T):
        P = [2 * 6.62607015e-34 * 2.99792458e8 ** 2 / ((nm * 1e-9) ** 5) /
             (math.exp(6.62607015e-34 * 2.99792458e8 / (nm * 1e-9 * 1.380649e-23 * T)) - 1) for nm in NM]
        Xp, Yp, Zp = xyz(P)
        dp = Xp + 15 * Yp + 3 * Zp
        return 4 * Xp / dp, 6 * Yp / dp

    lo, hi = 1000.0, 20000.0
    for _ in range(60):              # golden-section search on distance to the locus
        a = hi - (hi - lo) / 1.618; b = lo + (hi - lo) / 1.618
        da = math.dist((u, v), planck_uv(a)); db = math.dist((u, v), planck_uv(b))
        if da < db: hi = b
        else: lo = a
    T = (lo + hi) / 2
    pu, pv = planck_uv(T)
    duv = math.dist((u, v), (pu, pv)) * (1 if v > pv else -1)
    return T, duv


def fit(pairs):
    num = [0.0] * len(NM); den = [0.0] * len(NM)
    for m, h in pairs:
        ym, yh = xyz(m)[1], xyz(h)[1]
        pk = max(v for v in m if v == v)
        for i, (a, b) in enumerate(zip(m, h)):
            if a == a and b == b and a > 0.02 * pk:
                w = a / pk
                num[i] += w * (b / yh) / (a / ym)
                den[i] += w
    r = [n / d if d > 0 else float('nan') for n, d in zip(num, den)]
    first = next(i for i, v in enumerate(r) if v == v)
    last = max(i for i, v in enumerate(r) if v == v)
    for i in range(len(r)):          # hold the end values flat where there is no usable signal
        if i < first: r[i] = r[first]
        elif i > last: r[i] = r[last]
        elif r[i] != r[i]:
            k = i
            while r[k] != r[k]: k += 1
            r[i] = r[k]
    sig = 8.0                        # smooth: the ColorMunki resolves ~10 nm, so the curve must not be finer
    sm = []
    for i in range(len(r)):
        acc = wsum = 0.0
        for j in range(max(0, i - 24), min(len(r), i + 25)):
            w = math.exp(-0.5 * ((i - j) / sig) ** 2); acc += w * r[j]; wsum += w
        sm.append(min(max(acc / wsum, 0.5), 2.0))
    # Shape only: scale so the corrected readings keep their lux on average.
    k = sum(xyz(m)[1] for m, _ in pairs) / sum(xyz([a * c for a, c in zip(m, sm)])[1] for m, _ in pairs)
    return [c * k for c in sm]


def report(name, m, h, c):
    tm, dm = cct_duv(m); th, dh = cct_duv(h); tc, dc = cct_duv([a * b for a, b in zip(m, c)])
    print('  %-28s HPCS %6.0f K %+.4f | raw %+6.0f K %+.4f | corrected %+6.0f K %+.4f'
          % (name[:28], th, dh, tm - th, dm - dh, tc - th, dc - dh))
    return tm - th, tc - th, dm - dh, dc - dh


def rms(v): return math.sqrt(sum(x * x for x in v) / len(v)) if v else float('nan')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pair', nargs=2, action='append', metavar=('MUNKI_CSV', 'HPCS_CSV'), required=True)
    ap.add_argument('--write', action='store_true', help='write src/munki_corr.h')
    a = ap.parse_args()
    names = [Path(p[0]).stem for p in a.pair]
    pairs = [(resample(read_csv(m)), resample(read_csv(h))) for m, h in a.pair]
    c = fit(pairs)
    print('Fitted on all %d lamp(s): difference from the HPCS (CCT K, Duv)' % len(pairs))
    res = [report(n, m, h, c) for n, (m, h) in zip(names, pairs)]
    print('  RMS CCT error: raw %.0f K -> corrected %.0f K;  RMS Duv error: raw %.4f -> corrected %.4f'
          % (rms([r[0] for r in res]), rms([r[1] for r in res]), rms([r[2] for r in res]), rms([r[3] for r in res])))
    if len(pairs) >= 3:
        print('\nLeave-one-out (each lamp corrected with a curve fitted on the others):')
        lo = []
        for i, (n, (m, h)) in enumerate(zip(names, pairs)):
            lo.append(report(n, m, h, fit(pairs[:i] + pairs[i + 1:])))
        print('  RMS CCT error: raw %.0f K -> corrected %.0f K;  RMS Duv error: raw %.4f -> corrected %.4f'
              % (rms([r[0] for r in lo]), rms([r[1] for r in lo]), rms([r[2] for r in lo]), rms([r[3] for r in lo])))
    else:
        print('\n(With three or more lamps this also prints a leave-one-out check.)')
    if a.write:
        rows = '\n'.join('    ' + ','.join('%.5f' % v for v in c[i:i + 10]) + ',' for i in range(0, len(c), 10))
        (ROOT / 'src' / 'munki_corr.h').write_text(
            '// GENERATED by tools/fit_munki_corr.py -- spectral correction for the ColorMunki, fitted against an HPCS\n'
            '// (multiply the 1 nm spectrum by MK_CORR[nm - 380]). Shape only: normalised so lux is unchanged.\n'
            '// Fitted %s on %d lamp(s): %s\n'
            '#pragma once\n#define MK_CORR_FITTED 1\n#define MK_CORR_FIRST_NM 380\n#define MK_CORR_N 351\n'
            'static const float MK_CORR[MK_CORR_N] = {\n%s\n};\n'
            % (datetime.date.today().isoformat(), len(pairs), ', '.join(names), rows))
        print('\nWrote src/munki_corr.h (MK_CORR_FITTED 1).')


if __name__ == '__main__':
    main()
