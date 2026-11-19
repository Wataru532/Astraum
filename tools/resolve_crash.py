"""Resolve 'Astraum.exe+0x...' offsets in a crash report using build/Astraum.map.

usage: python tools/resolve_crash.py <crash-report.txt> [build/Astraum.map]
"""
import bisect
import re
import sys

IMAGE_BASE = 0x140000000


def load_map(path):
    syms = []
    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            t = line.split()
            if len(t) == 2 and t[0].startswith('0x') and not t[1].startswith('0x'):
                try:
                    syms.append((int(t[0], 16), t[1]))
                except ValueError:
                    pass
    syms.sort()
    return syms


def main():
    report = sys.argv[1]
    mapfile = sys.argv[2] if len(sys.argv) > 2 else 'build/Astraum.map'
    syms = load_map(mapfile)
    addrs = [s[0] for s in syms]
    with open(report, encoding='utf-8', errors='replace') as f:
        for line in f:
            m = re.search(r'Astraum\.exe\+0x([0-9a-f]+)', line)
            if not m:
                continue
            a = IMAGE_BASE + int(m.group(1), 16)
            i = bisect.bisect_right(addrs, a) - 1
            name = syms[i][1] if i >= 0 else '?'
            print(line.rstrip(), ' ->', name, '+0x%x' % (a - syms[i][0]) if i >= 0 else '')


if __name__ == '__main__':
    main()
