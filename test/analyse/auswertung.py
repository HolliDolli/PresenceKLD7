#!/usr/bin/env python3
"""Auswertung Parallelbetrieb Detektor v0 (KLD7 Person / KLD7 Fahrzeug).

Stand 24.11.2026. Methodik siehe Projekt-Doc "KLD7-Projektstand.md",
Befund "24.11.2026: Auswertung Parallelbetrieb".

Aufruf:
  python3 auswertung.py --logs KLD7_20261121.csv KLD7_20261122.csv ... \
      --replay replay.csv --hist hist.json [--tag0 2026-11-21]

Eingaben:
  --logs    Tagesdateien KLD7_YYYYMMDD.csv, lueckenlos aufeinanderfolgend
  --replay  Ausgabe von test/harness (alle FRAM-Zeilen der Logs am Stueck):
              grep -h '^FRAM' KLD7_*.csv > fram.csv
              g++ -O2 -o harness ../harness.cpp && ./harness fram.csv replay.csv
  --hist    HA-History als JSON: {"entity_id": [[iso_zeit, state], ...], ...}
            fuer binary_sensor.hue_motion_sued_occupancy und
            binary_sensor.gkeingang_door_sensor (HA haelt ca. 10 Tage)

Ausgabe: Kennzahlen zu den Punkten 1-7 auf stdout.

Hinweise:
- Seit 24.11.2026 ca. 12:45 traegt KDET den Zeitpunkt des Zustandswechsels
  (trigger.to_state.last_changed), die Zeilen koennen also ausserhalb der
  zeitlichen Reihenfolge stehen. Das Skript sortiert deshalb nach Zeit.
- LABL: Label "irrtum" (ab 24.11.2026) verwirft das unmittelbar vorangehende
  Label. Testzeilen 21.11.2026 12:58:25-12:58:28 werden ausgeschlossen.
"""
import argparse
import json
import re

import numpy as np
import pandas as pd

# --- Parameter (wie im Detektor bzw. in der Auswertung vom 20./24.11.) ---
ZONE = dict(y0=2.5, y1=10.5, x0=-7.0, x1=3.0, v0=0.8, v1=10.0)
FAST = 15.0          # km/h, schnelle Punkte = Fahrzeug
WARMUP = 30          # s am Anfang nicht vergleichen
RE_PT = re.compile(r'\[(-?[\d.]+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\]')


def load_logs(files, t0):
    fr, pts, kd, lb = [], [], [], []
    for fn in files:
        for line in open(fn, newline=''):
            p = line.rstrip('\r\n').split('; ')
            if p[0] == 'FRAM' and len(p) >= 12:
                fr.append((p[1], p[4], int(p[5])))
                for m in RE_PT.finditer(p[11]):
                    pts.append((p[1],) + tuple(float(x) for x in m.groups()))
            elif p[0] == 'KDET':
                kd.append((p[1], p[5], p[6]))
            elif p[0] == 'LABL':
                lb.append((p[1], p[5]))
    sec = lambda s: (pd.to_datetime(s) - t0).dt.total_seconds()
    fr = pd.DataFrame(fr, columns=['t', 'wc', 'f'])
    fr['t'] = sec(fr.t)
    fr['wc'] = pd.to_numeric(fr.wc, errors='coerce')
    fr = fr.sort_values('t', kind='stable').reset_index(drop=True)
    pts = pd.DataFrame(pts, columns=['t', 'd', 's', 'a', 'm'])
    pts['t'] = sec(pts.t)
    r = np.radians(pts.a)
    pts['x'] = pts.d * np.sin(r) / 100
    pts['y'] = pts.d * np.cos(r) / 100
    pts = pts.sort_values('t', kind='stable').reset_index(drop=True)
    kd = pd.DataFrame(kd, columns=['t', 'person', 'veh'])
    kd['t'] = sec(kd.t)
    kd = kd.sort_values('t', kind='stable').reset_index(drop=True)
    lb = pd.DataFrame(lb, columns=['t', 'label'])
    lb['t'] = sec(lb.t)
    lb = lb.sort_values('t', kind='stable').reset_index(drop=True)
    return fr, pts, kd, lb


def load_hist(fn, t0):
    out = {}
    for k, v in json.load(open(fn)).items():
        df = pd.DataFrame(v, columns=['t', 's'])
        t = pd.to_datetime(df.t, format='ISO8601', utc=True)
        df['t'] = (t.dt.tz_convert('Europe/Berlin').dt.tz_localize(None) - t0).dt.total_seconds()
        out[k] = df
    return out


def series(times, states, n):
    """Zustand je Sekunde aus Wechselzeitpunkten."""
    a = np.zeros(n, bool)
    cur, last = False, 0
    for t, s in zip(times, states):
        i = max(0, min(n, int(np.floor(t))))
        a[last:i] = cur
        cur, last = s, i
    a[last:] = cur
    return a


def episodes(a):
    d = np.diff(np.r_[0, a.astype(int), 0])
    return list(zip(np.where(d == 1)[0], np.where(d == -1)[0]))


def cnt(arr, a, b):
    return np.searchsorted(arr, b) - np.searchsorted(arr, a)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--logs', nargs='+', required=True)
    ap.add_argument('--replay', required=True)
    ap.add_argument('--hist', required=True)
    ap.add_argument('--tag0', help='erster Tag (YYYY-MM-DD), Standard: aus erstem Log')
    a = ap.parse_args()
    files = sorted(a.logs)
    tag0 = a.tag0 or re.search(r'(\d{8})', files[0]).group(1)
    t0 = pd.Timestamp(tag0)
    ndays = len(files)
    N = ndays * 86400
    hm = lambda s: (t0 + pd.Timedelta(seconds=int(s))).strftime('%d.%m. %H:%M:%S')

    fr, pts, kd, lb = load_logs(files, t0)
    h = load_hist(a.hist, t0)
    print(f'{len(fr)} FRAM, {len(pts)} Punkte, {len(kd)} KDET, {len(lb)} LABL, {ndays} Tage ab {tag0}')

    esp_p = series(kd.t, kd.person == 'on', N)
    esp_v = series(kd.t, kd.veh == 'on', N)
    rp = pd.read_csv(a.replay)
    rp['s'] = rp.t - int(t0.timestamp())   # harness rechnet Ortszeit als UTC
    m = (rp.s >= 0) & (rp.s < N)
    rep_p = np.zeros(N, bool); rep_p[rp.s[m]] = rp.person[m] == 1
    rep_v = np.zeros(N, bool); rep_v[rp.s[m]] = rp.vehicle[m] == 1

    # 1. ESP gegen Replay
    print('\n1. ESP gegen Offline-Replay')
    sl = slice(WARMUP, N)
    for nm, e, r in [('Person', esp_p, rep_p), ('Fahrzeug', esp_v, rep_v)]:
        E = [x for x in episodes(e) if x[0] >= WARMUP]
        R = [x for x in episodes(r) if x[0] >= WARMUP]
        ov = lambda A, B: [any(b[0] < a1 + 2 and b[1] > a0 - 2 for b in B) for a0, a1 in A]
        oe, orr = ov(E, R), ov(R, E)
        print(f'  {nm}: {100 * (e[sl] == r[sl]).mean():.2f} % Sekunden gleich, '
              f'ESP-Episoden {sum(oe)}/{len(E)} im Replay, Replay-Episoden {sum(orr)}/{len(R)} am ESP')
        print('    nur ESP:', [hm(x[0]) for x, o in zip(E, oe) if not o])
        print('    nur Replay:', [hm(x[0]) for x, o in zip(R, orr) if not o])
    same = (kd.person == kd.person.shift()) & (kd.veh == kd.veh.shift())
    print(f'  KDET-Zeilen ohne Aenderung: {same.sum()}', [hm(t) for t in kd.t[same]])

    pt = pts.t.values; ps = np.abs(pts.s.values); px = pts.x.values; py = pts.y.values
    fast_t = np.sort(pt[ps >= FAST])
    slow_near_t = np.sort(pt[(ps >= 0.8) & (ps <= 10) & (pts.d.values < 1000) & (py >= 2.5)])
    zone = ((py >= ZONE['y0']) & (py <= ZONE['y1']) & (px >= ZONE['x0']) & (px <= ZONE['x1'])
            & (ps >= ZONE['v0']) & (ps <= ZONE['v1']))
    zc = np.bincount(np.floor(pt[zone]).astype(int).clip(0, N - 1), minlength=N)

    # 2. Person gegen Hue Sued
    hue = h['binary_sensor.hue_motion_sued_occupancy']
    hue_on = series(hue.t, hue.s == 'on', N)
    trig = hue.t[(hue.s == 'on') & (hue.s.shift() != 'on') & (hue.t >= 0) & (hue.t < N)].values
    def cls(t):
        if cnt(fast_t, t - 6, t + 4) >= 15: return 'Fahrzeug'
        if cnt(slow_near_t, t - 15, t + 5) >= 30: return 'Person'
        return 'unklar'
    hc = np.array([cls(t) for t in trig])
    E = [e for e in episodes(esp_p) if e[0] >= WARMUP]
    print(f'\n2. KLD7 Person gegen Hue Sued: {len(trig)} Ausloesungen',
          pd.Series(hc).value_counts().to_dict())
    hits, lat = [], []
    for t, c in zip(trig, hc):
        ti = int(t)
        hit = esp_p[max(0, ti - 15):ti + 6].any()
        if c == 'Person':
            hits.append((t, hit))
            if hit:
                lat.append(next(a0 - t for a0, a1 in E if a0 <= ti + 5 and a1 >= ti - 15))
    for c in ['Fahrzeug', 'unklar']:
        mm = hc == c
        print(f'  Person an bei Hue-{c}: {sum(esp_p[max(0, int(t) - 15):int(t) + 6].any() for t in trig[mm])}/{mm.sum()}')
    nh = sum(x[1] for x in hits)
    print(f'  Treffer: {nh}/{len(hits)} = {100 * nh / max(1, len(hits)):.0f} %, verpasst:',
          [hm(t) for t, x in hits if not x])
    lat = np.array(lat)
    if len(lat):
        print(f'  an spaetestens bei Hue: {100 * (lat <= 0).mean():.0f} %, bis Hue+2 s: {100 * (lat <= 2).mean():.0f} %')
    noh = [(a0, a1) for a0, a1 in E if not hue_on[max(0, a0 - 10):a1 + 10].any()]
    print(f'  Episoden {len(E)}, ohne Hue (+-10 s): {len(noh)} = {len(noh) / ndays:.1f}/Tag')
    for a0, a1 in noh:
        mz = (pt >= a0 - 3) & (pt < a1) & zone
        print(f'    {hm(a0)} Dauer {a1 - a0} s, x {px[mz].mean():.1f} y {py[mz].mean():.1f}')
    L = []
    for a0, a1 in E:
        s, first, gap = a0 - 1, None, 0
        while s > a0 - 60:
            if zc[s] >= 3: first, gap = s, 0
            else:
                gap += 1
                if gap > 2: break
            s -= 1
        if first is not None: L.append(a0 - first)
    L = np.array(L)
    if len(L):
        print(f'  ab Zonensignal (>=3 Pkt/s) bis an: Median {np.median(L):.0f} s, <=3 s: {100 * (L <= 3).mean():.0f} %')

    # 3. Haustuer
    door = h.get('binary_sensor.gkeingang_door_sensor')
    if door is not None:
        d = door[door.s != 'unavailable'].reset_index(drop=True)
        op = d.t[(d.s == 'on') & (d.s.shift() == 'off')].values
        op = op[(op >= 60) & (op < N - 60)]
        ev = []
        for t in op:
            if ev and t - ev[-1][-1] <= 120: ev[-1].append(t)
            else: ev.append([t])
        tab = {}
        for e in ev:
            k = (bool(esp_p[int(e[0]) - 60:int(e[-1]) + 61].any()), bool(hue_on[int(e[0]) - 60:int(e[-1]) + 61].any()))
            tab[k] = tab.get(k, 0) + 1
        rs = np.random.default_rng(1).integers(60, N - 60, 20000)
        print(f'\n3. Haustuer: {len(op)} Oeffnungen, {len(ev)} Ereignisse. KLD7+Hue {tab.get((True, True), 0)}, '
              f'nur KLD7 {tab.get((True, False), 0)}, nur Hue {tab.get((False, True), 0)}, keiner {tab.get((False, False), 0)}. '
              f'Zufall KLD7 {100 * np.mean([esp_p[s - 60:s + 61].any() for s in rs]):.1f} %, '
              f'Hue {100 * np.mean([hue_on[s - 60:s + 61].any() for s in rs]):.1f} %')

    # 4. Labels
    t_test0 = (pd.Timestamp('2026-11-21 12:58:25') - t0).total_seconds()
    L4 = lb[~((lb.t >= t_test0) & (lb.t <= t_test0 + 4))].reset_index(drop=True)
    drop = set()
    for i, r in L4.iterrows():
        if r.label == 'irrtum':
            drop.add(i)
            if i > 0: drop.add(i - 1)
    L4 = L4.drop(index=list(drop))
    print(f'\n4. Labels ({len(L4)}, ohne Testzeilen und irrtum-Paare), Fenster -30...+5 s')
    for _, r in L4.iterrows():
        t = int(r.t)
        print(f'  {hm(t)} {r.label:22s} Person {int(esp_p[t - 30:t + 6].any())} '
              f'Fahrzeug {int(esp_v[t - 30:t + 6].any())} Hue {int(hue_on[t - 30:t + 6].any())} '
              f'schnellePkt {cnt(fast_t, t - 30, t + 5)}')

    # 5. Fahrzeug gegen Vorbeifahrten (dichte Gruppen schneller Punkte)
    g, cur = [], [fast_t[0]]
    for t in fast_t[1:]:
        if t - cur[-1] <= 3: cur.append(t)
        else: g.append(cur); cur = [t]
    g.append(cur)
    dens = lambda c: max(np.searchsorted(c, x + 3) - i for i, x in enumerate(c))
    P = [np.array(c) for c in g if len(c) >= 15 and c[0] >= WARMUP and dens(np.array(c)) >= 10]
    miss = [c for c in P if not esp_v[int(c[0]) - 2:int(c[-1]) + 4].any()]
    Ps = np.array([(c[0], c[-1]) for c in P])
    EV = [e for e in episodes(esp_v) if e[0] >= WARMUP]
    nf = [a0 for a0, a1 in EV if not ((Ps[:, 0] < a1 + 3) & (Ps[:, 1] > a0 - 3)).any()]
    print(f'\n5. Fahrzeug: {len(P)} Vorbeifahrten, erkannt {len(P) - len(miss)} = {100 - 100 * len(miss) / len(P):.1f} %, '
          f'verpasst {[hm(c[0]) for c in miss]}; Episoden ohne Vorbeifahrt {len(nf)} = {len(nf) / ndays:.1f}/Tag')

    # 6. Regen
    wc = np.full(N, np.nan)
    s_ = np.floor(fr.t.values).astype(int); ok = (s_ >= 0) & (s_ < N)
    wc[s_[ok]] = fr.wc.values[ok]
    wc = pd.Series(wc).ffill().bfill().values
    rain = (wc >= 500) & (wc < 600)
    print(f'\n6. Regen: {100 * rain.mean():.1f} % der Zeit mit Wettercode 5xx')
    if rain.any():
        rh = sum(1 for t, x in hits if x and rain[int(t)]); rn = sum(1 for t, x in hits if rain[int(t)])
        print(f'  Treffer bei Regen {rh}/{rn}; Episoden ohne Hue bei Regen {sum(1 for a0, _ in noh if rain[a0])} '
              f'in {rain.sum() / 86400:.2f} Regentagen')

    # 7. Neuinitialisierungen
    f = fr.f.values; t = fr.t.values
    R = np.where(np.diff(f) < 0)[0]
    print(f'\n7. Neuinitialisierungen K-LD7 (Ruecksprung Frame-Nr.): {len(R)} = {len(R) / ndays:.1f}/Tag',
          [hm(t[i + 1]) for i in R])


if __name__ == '__main__':
    main()
