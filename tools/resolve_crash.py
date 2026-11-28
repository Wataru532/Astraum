"""Resolve 'Astraum.exe+0x...' offsets in a crash report using build/Astraum.map.

usage: python tools/resolve_crash.py <crash-report.txt> [build/Astraum.map]
"""
import bisect
import re
import sys

IMAGE_BASE = 0x140000000
SYM_RE = re.compile(r'^\s+(0x[0-9a-fA-F]+)\s+(\S.*)$')


def load_map(path):
    syms = []
    image_base = IMAGE_BASE
    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            m = SYM_RE.match(line)
            if not m:
                continue
            name = m.group(2).strip()
            if name.startswith('0x') or name.startswith('load address '):
                continue
            try:
                addr = int(m.group(1), 16)
            except ValueError:
                continue
            if name == '__image_base__' or name.startswith('__image_base__ '):
                image_base = addr
                continue
            if '=' in name and name.lstrip().startswith('.'):
                continue
            syms.append((addr, name))
    syms.sort()
    return syms, image_base


def main():
    if len(sys.argv) < 2:
        sys.exit('usage: python tools/resolve_crash.py <crash-report.txt> [build/Astraum.map]')
    report = sys.argv[1]
    mapfile = sys.argv[2] if len(sys.argv) > 2 else 'build/Astraum.map'
    syms, image_base = load_map(mapfile)
    addrs = [s[0] for s in syms]
    with open(report, encoding='utf-8', errors='replace') as f:
        for line in f:
            m = re.search(r'Astraum\.exe\+0x([0-9a-fA-F]+)', line)
            if not m:
                continue
            a = image_base + int(m.group(1), 16)
            i = bisect.bisect_right(addrs, a) - 1
            name = syms[i][1] if i >= 0 else '?'
            print(line.rstrip(), ' ->', name, '+0x%x' % (a - syms[i][0]) if i >= 0 else '')


if __name__ == '__main__':
    main()
