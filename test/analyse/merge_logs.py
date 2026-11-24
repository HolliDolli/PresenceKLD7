#!/usr/bin/env python3
"""Fuehrt zwei Teile eines KLD7-Tageslogs zu einer Datei zusammen.

Anlass: 23.11.2026, Umbenennung log2.csv -> KLD7.csv um 09:18.
Aufruf: python3 merge_logs.py teil1.csv teil2.csv KLD7_YYYYMMDD.csv

- Kopf der HA-Notifications (2 Zeilen) aus Teil 1, nur einmal
- K-Zeilen (Kopfzeilen) einmal oben
- Datenzeilen unveraendert, Dubletten entfernt, stabil nach Zeitstempel sortiert
- Leerzeilen entfernt, Ausgabe mit CRLF
"""
import sys


def main():
    a, b, out = sys.argv[1:4]
    rd = lambda p: open(p, newline='').read().replace('\r\n', '\n').split('\n')
    la, lb = rd(a), rd(b)
    hdr = la[:2]
    k = []
    for l in la + lb:
        if l.startswith('K;') and l not in k:
            k.append(l)
    seen, data = set(), []
    for l in la[2:] + lb[2:]:
        if not l.strip() or l.startswith('K;') or l in seen:
            continue
        seen.add(l)
        data.append(l)
    n = len(data)
    data.sort(key=lambda l: l.split('; ')[1])
    with open(out, 'w', newline='') as f:
        f.write('\r\n'.join(hdr + k + data) + '\r\n')
    print(f'{n} Datenzeilen, {len(k)} Kopfzeilen -> {out}')


if __name__ == '__main__':
    main()
