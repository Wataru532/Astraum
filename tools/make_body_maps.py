"""Builds the colour maps of the bodies whose relief Astraum generates (assets/maps/<body>_color.jpg).

Usage:  python -I tools/make_body_maps.py <source-dir> <out-dir>

Sources: spacecraft global mosaics, public domain (NASA / USGS Astrogeology / JPL), see assets/maps/SOURCES.txt.
Every map is written in the game's layout: equirectangular, column 0 = longitude 180 W, east to the right, row 0 = 90 N,
4096 x 2048 (Mercury 8192 x 4096). Where a mosaic has no data (Pluto's far south, the edges of Triton's coverage) the
gap is filled with the mosaic's own mean colour, blurred into the border. Black-and-white mosaics get the body's
colour (from its colour photos), and every map is scaled to the body's brightness (geometric albedo).
"""
import sys

import numpy as np
from PIL import Image, ImageFilter

Image.MAX_IMAGE_PIXELS = None

# file, body, size, tint (None: the mosaic's own colour), mean brightness (0..255 sRGB), saturation kept
BODIES = [
    ("Mercury_MESSENGER_MDIS_Basemap_MD3Color_Mosaic_Global_64ppd.tif", "mercury", 8192, None, 112, 0.25),
    ("Io_GalileoSSI-Voyager_Global_Mosaic_ClrMerge_1km.tif", "io", 4096, None, 175, 0.85),
    ("Europa_Voyager_GalileoSSI_global_mosaic_500m.tif", "europa", 4096, (1.0, 0.94, 0.84), 185, 1.0),
    ("Ganymede_Voyager_GalileoSSI_Global_ClrMosaic_1435m.tif", "ganymede", 4096, None, 125, 0.6),
    ("Callisto_Voyager_GalileoSSI_global_mosaic_1km.tif", "callisto", 4096, (1.0, 0.93, 0.84), 82, 1.0),
    ("Enceladus_Cassini_mosaic_global_110m.tif", "enceladus", 4096, (0.98, 0.99, 1.0), 228, 1.0),
    ("Triton_Voyager2_ClrMosaic_GlobalFill_600m.tif", "triton", 4096, None, 180, 0.7),
    ("Pluto_NewHorizons_Global_Mosaic_300m_Jul2017_8bit.tif", "pluto", 4096, (1.0, 0.86, 0.70), 150, 1.0),
    ("Charon_NewHorizons_Global_Mosaic_300m_Jul2017_8bit.tif", "charon", 4096, (1.0, 0.97, 0.94), 108, 1.0),
    ("Ceres_Dawn_FC_DLR_global_20ppd_Oct2015.tif", "ceres", 4096, (1.0, 0.99, 0.97), 72, 1.0),
    ("Phobos_ME_SRC_Mosaic_Global_16ppd.tif", "phobos", 4096, (1.0, 0.95, 0.90), 58, 1.0),
    ("PIA07782.tif", "jupiter", 4096, None, None, 1.0),
]


def left_edge_lon(img):
    """Longitude [deg] of the left edge of a global equirectangular GeoTIFF (from its tie point and pixel scale)."""
    tags = getattr(img, "tag_v2", {})
    scale, tie = tags.get(33550), tags.get(33922)
    if not scale or not tie:
        return None
    full = scale[0] * img.size[0]  # (the map spans 360 degrees: the units cancel)
    x = tie[3] - tie[0] * scale[0]
    # projected maps count x from their central meridian (GeoKeys ProjNatOriginLong 3080 / ProjCenterLong 3088)
    central = 0.0
    keys, dbl = tags.get(34735), tags.get(34736)
    if keys and dbl and abs(full - 360.0) > 1e-6:
        k = list(keys)
        for i in range(4, len(k), 4):
            kid, loc, cnt, val = k[i:i + 4]
            if kid in (3080, 3088) and loc == 34736:
                central = float(dbl[val])
    lon = central + x / full * 360.0
    return (lon + 180.0) % 360.0 - 180.0


def to_rgb_float(img):
    if img.mode in ("I;16", "I;16B", "I"):
        a = np.asarray(img, np.float32)
        a = a / max(1.0, float(np.percentile(a[a > 0], 99.9)) if (a > 0).any() else 1.0)
        a = np.clip(a, 0, 1)
        return np.dstack([a, a, a]), a <= 0
    if img.mode == "F":
        a = np.asarray(img, np.float32)
        bad = ~np.isfinite(a) | (a <= -1e30)
        a[bad] = 0
        hi = float(np.percentile(a[~bad], 99.9)) if (~bad).any() else 1.0
        a = np.clip(a / max(hi, 1e-9), 0, 1)
        return np.dstack([a, a, a]), bad | (a <= 0)
    if img.mode not in ("RGB", "L"):
        img = img.convert("RGB")
    a = np.asarray(img, np.float32) / 255.0
    if a.ndim == 2:
        return np.dstack([a, a, a]), a <= 0
    return a, a.max(axis=2) <= 0


def make(src, out, fname, body, W, tint, mean, sat):
    img = Image.open(f"{src}/{fname}")
    lon0 = left_edge_lon(img)
    # reduce first (the mosaics are up to 23040 px wide), keeping the no-data mask
    w0, h0 = img.size
    a, bad = to_rgb_float(img)
    H = W // 2
    small = np.dstack([np.asarray(Image.fromarray(a[..., c]).resize((W, H), Image.BOX), np.float32) for c in range(3)])
    badf = np.asarray(Image.fromarray(bad.astype(np.float32)).resize((W, H), Image.BOX), np.float32)
    # columns: the game's map starts at 180 W
    if lon0 is None:
        lon0 = -180.0
    shift = int(round((lon0 + 180.0) / 360.0 * W)) % W
    small = np.roll(small, shift, axis=1)
    badf = np.roll(badf, shift, axis=1)
    good = badf < 0.5
    # brightness / colour
    lum = small.mean(axis=2)
    if tint is not None:
        g = lum
        small = np.dstack([g * tint[0], g * tint[1], g * tint[2]])
    if sat < 1.0:
        g = small.mean(axis=2, keepdims=True)
        small = g + (small - g) * sat
    if mean is not None and good.any():
        m = float(small[good].mean()) * 255.0
        small = small * (mean / max(m, 1e-6))
    # the gaps: the other hemisphere mirrored and blurred where it was seen (a surface of the body's own kind
    # rather than a flat band), the mean colour elsewhere; the border blurred into it
    if (~good).any():
        mean_c = small[good].mean(axis=0)
        mirror = small[::-1]
        mgood = good[::-1]
        fill = np.where(mgood[..., None], mirror, mean_c)
        fill_img = Image.fromarray(np.clip(fill * 255, 0, 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(W / 512))
        fill = np.asarray(fill_img, np.float32) / 255
        fill = mean_c + (fill - mean_c) * 0.6
        k = np.clip((badf - 0.2) / 0.6, 0, 1)[..., None]
        small = small * (1 - k) + fill * k
        blurred = np.asarray(Image.fromarray(np.clip(small * 255, 0, 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(6)), np.float32) / 255
        edge = np.asarray(Image.fromarray((k[..., 0] * 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(10)), np.float32)[..., None] / 255
        small = small * (1 - edge * 0.7) + blurred * (edge * 0.7)
    rgb = np.clip(np.round(small * 255), 0, 255).astype(np.uint8)
    Image.fromarray(rgb, "RGB").save(f"{out}/{body}_color.jpg", quality=90, optimize=True)
    print(f"{body}: {fname} {w0}x{h0} {img.mode}, left edge {lon0:.1f} deg -> shifted {shift} px, no data {100 * (1 - good.mean()):.0f} %, "
          f"mean {float(rgb.mean()):.0f}")


if __name__ == "__main__":
    only = sys.argv[3:] if len(sys.argv) > 3 else None
    for f, b, w, t, m, s in BODIES:
        if only and b not in only:
            continue
        make(sys.argv[1], sys.argv[2], f, b, w, t, m, s)
