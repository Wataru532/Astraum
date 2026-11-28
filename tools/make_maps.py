"""Builds the surface maps embedded in Astraum (assets/maps) from public-domain NASA / USGS data.

Usage:  python -I tools/make_maps.py <source-dir> <out-dir>

Sources (all public domain, see assets/maps/SOURCES.txt):
  earth_color.jpg   NASA Blue Marble "Land_ocean_ice_2048.jpg" (Wikimedia Commons)
  earth_srtm.jpg    NASA Blue Marble "Srtm_ramp2.world.21600x10800.jpg" (land elevation, grey ramp)
  moon_color.jpg    NASA SVS CGI Moon Kit, LROC WAC colour 2k (svs.gsfc.nasa.gov/4720)
  moon_ldem_4.tif   NASA SVS CGI Moon Kit, LOLA elevation 4 px/deg, float km
  mars_mola.img     PDS MGS MOLA MEGDR megt90n000cb.img, 4 px/deg, int16 big-endian metres
  mars_color.jpg    USGS / NASA Ames Viking MDIM 2.1 colour mosaic, 1 km (Wikimedia Commons)

Every map is written in the game's layout: equirectangular, column 0 = longitude -180 (180 W),
east to the right, row 0 = 90 N. Heights are stored losslessly as 16 bit codes in the red (high byte)
and green (low byte) channels of a PNG: metres = code * 0.6 - 12000.
"""
import sys
import numpy as np
from PIL import Image

H_OFS, H_STEP = -12000.0, 0.6
W, H = 2048, 1024

Image.MAX_IMAGE_PIXELS = None  # the Mars mosaic is 228 Mpx (read in DCT draft mode, see below)


def save_height(path, h):
    h = np.nan_to_num(h, nan=0.0, posinf=0.0, neginf=0.0)
    code = np.clip(np.round((h - H_OFS) / H_STEP), 0, 65535).astype(np.uint16)
    rgb = np.zeros(code.shape + (3,), np.uint8)
    rgb[..., 0] = code >> 8
    rgb[..., 1] = code & 255
    Image.fromarray(rgb, "RGB").save(path, optimize=True)


def resize_f(a, w, h):
    return np.asarray(Image.fromarray(a.astype(np.float32), "F").resize((w, h), Image.BOX), np.float32)


def lonlat_px(a, lon, lat):
    hh, ww = a.shape[:2]
    x = int((lon + 180) / 360 * ww) % ww
    y = min(hh - 1, max(0, int((90 - lat) / 180 * hh)))
    return x, y


def window_mean(a, lon, lat, r=3):
    x, y = lonlat_px(a, lon, lat)
    return float(a[max(0, y - r):y + r + 1, max(0, x - r):x + r + 1].mean())


def main():
    src, out = sys.argv[1], sys.argv[2]
    log = []

    # ---------------- Earth
    col = Image.open(f"{src}/earth_color.jpg").convert("RGB").resize((W, H), Image.LANCZOS)
    col.save(f"{out}/earth_color.jpg", quality=90, optimize=True)
    g = Image.open(f"{src}/earth_srtm.jpg").convert("L")
    ga = np.asarray(g, np.float32)
    # the grey ramp: 12 = sea level (the oceans are flat 12), 255 = the highest summits; calibrated on
    # Everest (8848 m). Ocean and coast pixels become 0 m (the sea surface is what the game shows).
    SEA = 12.0
    x, y = lonlat_px(ga, 86.925, 27.988)
    ev = float(ga[y - 8:y + 9, x - 8:x + 9].max())
    scale = 8848.0 / max(ev - SEA, 1.0)
    he = resize_f(np.maximum(ga - SEA, 0.0) * scale, W, H)
    log.append(f"earth: grey at Everest {ev:.0f} -> {scale:.1f} m per step; Mont Blanc {window_mean(he, 6.865, 45.832, 1):.0f} m, "
               f"Tibet {window_mean(he, 88, 33, 4):.0f} m, Sahara {window_mean(he, 10, 23, 4):.0f} m, Pacific {window_mean(he, -150, 0, 4):.0f} m")
    save_height(f"{out}/earth_height.png", he)

    # ---------------- Moon
    mc = Image.open(f"{src}/moon_color.jpg").convert("RGB").resize((W, H), Image.LANCZOS)
    mc.save(f"{out}/moon_color.jpg", quality=90, optimize=True)
    ml = np.asarray(Image.open(f"{src}/moon_ldem_4.tif"), np.float32)
    if np.nanmax(np.abs(ml)) < 100:  # km
        ml = ml * 1000.0
    # orientation check: the dark maria lie low (brightness and height correlate on the near side)
    bright = np.asarray(mc.convert("L").resize((ml.shape[1], ml.shape[0]), Image.BOX), np.float32)
    def corr(a, b):
        mask = np.isfinite(a) & np.isfinite(b)
        if not mask.any():
            return 0.0
        aa = a[mask] - a[mask].mean()
        bb = b[mask] - b[mask].mean()
        denom = float(np.sqrt((aa * aa).sum() * (bb * bb).sum()))
        return float((aa * bb).sum() / denom) if denom > 0 else 0.0
    c0, c1 = corr(bright, ml), corr(bright, np.roll(ml, ml.shape[1] // 2, axis=1))
    if c1 > c0:
        ml = np.roll(ml, ml.shape[1] // 2, axis=1)
    log.append(f"moon: elevation {ml.min():.0f}..{ml.max():.0f} m, colour/height correlation {max(c0, c1):.2f} (other half-turn {min(c0, c1):.2f}); "
               f"Mare Imbrium {window_mean(ml, -16, 33):.0f} m, highlands {window_mean(ml, 10, -30):.0f} m")
    save_height(f"{out}/moon_height.png", ml)

    # ---------------- Mars
    raw = np.fromfile(f"{src}/mars_mola.img", dtype=">i2").reshape(720, 1440).astype(np.float32)
    mola = np.roll(raw, 720, axis=1)  # the file starts at 0 E; the game starts at 180 W
    yy, xx = np.unravel_index(np.argmax(mola), mola.shape)
    lon_max = xx / 1440 * 360 - 180 + 0.125
    lat_max = 90 - yy / 720 * 180 - 0.125
    log.append(f"mars: elevation {mola.min():.0f}..{mola.max():.0f} m, highest point at {lat_max:.1f} N {lon_max:.1f} E (Olympus Mons: 18.7 N -133.8 E); "
               f"Valles Marineris (Coprates) floor {window_mean(mola, -65, -12, 0):.0f} m vs. rim {window_mean(mola, -65, -16, 0):.0f} m; Hellas {window_mean(mola, 70, -42, 3):.0f} m")
    save_height(f"{out}/mars_height.png", mola)
    m = Image.open(f"{src}/mars_color.jpg")
    m.draft("RGB", (m.width // 8, m.height // 8))  # DCT scaling: decodes at 1/8 size
    m = m.convert("RGB").resize((W, H), Image.LANCZOS)
    ma = np.asarray(m.convert("L"), np.float32)
    # orientation check: Syrtis Major (8 N 70 E) is one of the darkest regions, Hellas (42 S 70 E) bright
    s0 = window_mean(ma, 70, 8, 6) - window_mean(ma, 70, -42, 6)
    s1 = window_mean(np.roll(ma, W // 2, axis=1), 70, 8, 6) - window_mean(np.roll(ma, W // 2, axis=1), 70, -42, 6)
    if s1 < s0:
        m = Image.fromarray(np.roll(np.asarray(m), W // 2, axis=1))
    log.append(f"mars colour: Syrtis Major minus Hellas brightness {min(s0, s1):.0f} (other half-turn {max(s0, s1):.0f})")
    m.save(f"{out}/mars_color.jpg", quality=90, optimize=True)
    print("\n".join(log))


if __name__ == "__main__":
    main()
