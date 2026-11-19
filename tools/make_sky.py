"""Builds the night-sky data embedded in Astraum (assets/sky).

Usage:  python -I tools/make_sky.py <source-dir> <out-dir>

Sources (see assets/sky/SOURCES.txt):
  ybsc5.gz               Yale Bright Star Catalogue, 5th revised edition (Hoffleit & Warren 1991), the ASCII
                         version distributed by the Harvard-Smithsonian CfA (tdc-www.harvard.edu/catalogs/bsc5.html)
  milkyway_2020_8k.exr   NASA/Goddard Space Flight Center Scientific Visualization Studio, "Deep Star Maps 2020"
                         (svs.gsfc.nasa.gov/4851): the Milky Way background in celestial coordinates (Gaia DR2:
                         ESA/Gaia/DPAC)

Written:
  stars.bin      uint32 count, then per star float32 RA [rad], Dec [rad] (J2000), V magnitude, B-V colour index;
                 every star of the catalogue with a position and a magnitude (to ~6.5 mag: the naked-eye sky)
  milkyway.jpg   8192 x 4096, equirectangular in equatorial J2000 coordinates, column 0 = RA 12h and RA
                 increasing to the LEFT (the map's own layout: as the sky is seen from inside), row 0 = Dec +90;
                 values gamma-encoded (linear ** (1 / 2.2)) after scaling the 99.97th percentile to white
"""
import gzip
import struct
import sys

import numpy as np
from PIL import Image


def stars(src, out):
    rows = []
    with gzip.open(f"{src}/ybsc5.gz", "rt", encoding="latin1") as f:
        for line in f:
            line = line.rstrip("\n")
            if len(line) < 107:
                continue
            try:
                rah, ram, ras = int(line[75:77]), int(line[77:79]), float(line[79:83])
                sgn = -1.0 if line[83] == "-" else 1.0
                ded, dem, des = int(line[84:86]), int(line[86:88]), int(line[88:90])
                vmag = float(line[102:107])
            except ValueError:
                continue  # (a few novae / non-stellar entries have no J2000 position)
            try:
                bv = float(line[109:114])
            except ValueError:
                bv = 0.6  # (no colour: a Sun-like white)
            ra = (rah + ram / 60 + ras / 3600) * 15.0
            dec = sgn * (ded + dem / 60 + des / 3600)
            rows.append((np.radians(ra), np.radians(dec), vmag, bv))
    rows.sort(key=lambda r: r[2])  # brightest first
    with open(f"{out}/stars.bin", "wb") as f:
        f.write(struct.pack("<I", len(rows)))
        for r in rows:
            f.write(struct.pack("<4f", *r))
    # checks: Sirius, Polaris
    sir = min(rows, key=lambda r: r[2])
    print(f"stars: {len(rows)}, brightest V {sir[2]:.2f} at RA {np.degrees(sir[0]):.2f} Dec {np.degrees(sir[1]):.2f} (Sirius: 101.29 -16.72)")


def milkyway(src, out):
    import OpenEXR
    with OpenEXR.File(f"{src}/milkyway_2020_8k.exr") as f:
        a = np.asarray(f.channels()["RGB"].pixels, np.float32)
    import cv2
    h, w = a.shape[:2]
    # the faint stars the map is made of are pixel noise at this size: a light blur, then along each row as much
    # more as the row is shorter on the sphere (an equal smoothing everywhere: no radial streaks at the poles)
    a = cv2.GaussianBlur(a, (0, 0), 1.7)
    out_rows = np.empty_like(a)
    band = 16
    for y0 in range(0, h, band):
        dec = np.radians(90 - (y0 + band / 2) / h * 180)
        sx = min(1.3 / max(np.cos(dec), 1e-4), w / 6)
        if sx < 1.5:
            out_rows[y0:y0 + band] = a[y0:y0 + band]
            continue
        pad = int(min(w, 3 * sx + 2))
        strip = a[y0:y0 + band]
        wrapped = np.concatenate([strip[:, -pad:], strip, strip[:, :pad]], axis=1)
        k = int(2 * round(3 * sx) + 1)
        blurred = cv2.GaussianBlur(wrapped, (min(k, 2 * (w // 2) + 1), 1), sx, sigmaY=0.01)
        out_rows[y0:y0 + band] = blurred[:, pad:pad + w]
    a = out_rows
    # the sky between the band dark: the floor of the map (sky glow of the faint stars) taken off
    lum = a.mean(axis=2)
    floor = float(np.percentile(lum, 50))
    white = float(np.percentile(lum, 99.95))
    v = np.clip((a - floor) / (white - floor), 0, 1) ** (1 / 2.2)
    print(f"milky way: floor {floor:.4f}, white {white:.4f}")
    img = Image.fromarray(np.round(v * 255).astype(np.uint8), "RGB")
    img.save(f"{out}/milkyway.jpg", quality=92, optimize=True, subsampling=0)
    # check: the galactic centre (RA 266.4, Dec -28.9) is the brightest part of the band
    s = np.asarray(img.convert("L").resize((360, 180), Image.BOX), np.float32)
    y, x = np.unravel_index(np.argmax(s), s.shape)
    print(f"milky way: {img.size}, white = {white:.4f}, brightest at RA {(180 - x - 0.5) % 360:.0f} Dec {90 - y - 0.5:.0f} (galactic centre 266 -29)")


if __name__ == "__main__":
    stars(sys.argv[1], sys.argv[2])
    milkyway(sys.argv[1], sys.argv[2])
