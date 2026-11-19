"""Builds the high-resolution surface tiles of Astraum (assets/maps/*_hi.pak) from public-domain
NASA / USGS data. The game shows them near a body (the area below the camera); from afar it keeps the
global maps of tools/make_maps.py.

Usage:  python -I tools/make_detail_maps.py <source-dir> <maps-dir>

<maps-dir> must already hold the global maps (they are the reference for the orientation checks). The
global colour maps of the Earth and the Moon are then rewritten from the same pictures as their tiles.
Sources (all public domain, see assets/maps/SOURCES.txt):
  earth_color_12000.jpg  NASA Blue Marble "Whole world - land and oceans 12000" (24000 x 12000, Wikimedia Commons)
  earth_srtm.jpg         NASA Blue Marble "Srtm_ramp2.world.21600x10800.jpg" (as for the global map)
  moon_color_8k.tif      NASA SVS CGI Moon Kit, LROC WAC colour 8k (svs.gsfc.nasa.gov/4720)
  moon_ldem_16.tif       NASA SVS CGI Moon Kit, LOLA elevation 16 px/deg, float km
  mars_mola_16.img       PDS MGS MOLA MEGDR megt90n000eb.img, 16 px/deg, int16 big-endian metres
  mars_color.jpg         USGS / NASA Ames Viking MDIM 2.1 colour mosaic, 1 km (as for the global map)

Pack layout (little endian): "SFTP", version 1, kind (0 colour JPEG tiles, 1 elevation PNG tiles: 16 bit
code in R high / G low, metres = code * 0.6 - 12000), grid w, grid h, tile w, tile h, full w, full h,
then grid w * grid h (offset, size) pairs and the tile files. Tile (0, 0) starts at 180 W, 90 N.
"""
import io
import struct
import sys
import numpy as np
from PIL import Image

H_OFS, H_STEP = -12000.0, 0.6
GW, GH = 16, 8                     # tiles around / pole to pole
COLOR_W, COLOR_H = 8192, 4096

Image.MAX_IMAGE_PIXELS = None  # (the source mosaics are large; read from known public-domain sources only)


def resize_f(a, w, h):
    return np.asarray(Image.fromarray(a.astype(np.float32), "F").resize((w, h), Image.BOX), np.float32)


def corr(a, b):
    a = a - a.mean()
    b = b - b.mean()
    return float((a * b).sum() / np.sqrt((a * a).sum() * (b * b).sum() + 1e-12))


def orient(arr, ref):
    """roll by half a turn if that matches the reference (a validated global map) better"""
    h, w = ref.shape
    small = resize_f(arr if arr.ndim == 2 else arr.mean(axis=2), w, h)
    c0, c1 = corr(small, ref), corr(np.roll(small, w // 2, axis=1), ref)
    if c1 > c0:
        return np.roll(arr, arr.shape[1] // 2, axis=1), c1, True
    return arr, c0, False


def height_code(h):
    code = np.clip(np.round((h - H_OFS) / H_STEP), 0, 65535).astype(np.uint16)
    rgb = np.zeros(code.shape + (3,), np.uint8)
    rgb[..., 0] = code >> 8
    rgb[..., 1] = code & 255
    return rgb


def ref_height(path):
    a = np.asarray(Image.open(path).convert("RGB"), np.float32)
    return (a[..., 0] * 256 + a[..., 1]) * H_STEP + H_OFS


def write_pack(path, kind, tiles, tw, th, fw, fh):
    head = struct.pack("<4s9I", b"SFTP", 1, kind, GW, GH, tw, th, fw, fh, 0)
    n = GW * GH
    table_size = n * 8
    offs, blob = [], bytearray()
    base = len(head) + table_size
    for t in tiles:
        offs.append((base + len(blob), len(t)))
        blob += t
    with open(path, "wb") as f:
        f.write(head)
        for o, s in offs:
            f.write(struct.pack("<II", o, s))
        f.write(blob)
    return len(head) + table_size + len(blob)


def pack_color(img, path):
    img = img.resize((COLOR_W, COLOR_H), Image.LANCZOS)
    tw, th = COLOR_W // GW, COLOR_H // GH
    tiles = []
    for ty in range(GH):
        for tx in range(GW):
            b = io.BytesIO()
            img.crop((tx * tw, ty * th, (tx + 1) * tw, (ty + 1) * th)).save(b, "JPEG", quality=88, optimize=True)
            tiles.append(b.getvalue())
    return write_pack(path, 0, tiles, tw, th, COLOR_W, COLOR_H)


def pack_height(h, path):
    hh, ww = h.shape
    assert ww % GW == 0 and hh % GH == 0, (ww, hh)
    tw, th = ww // GW, hh // GH
    rgb = height_code(h)
    tiles = []
    for ty in range(GH):
        for tx in range(GW):
            b = io.BytesIO()
            Image.fromarray(rgb[ty * th:(ty + 1) * th, tx * tw:(tx + 1) * tw], "RGB").save(b, "PNG", optimize=True)
            tiles.append(b.getvalue())
    return write_pack(path, 1, tiles, tw, th, ww, hh)


def main():
    src, out = sys.argv[1], sys.argv[2]
    log = []

    # ---------------- Earth
    ref = np.asarray(Image.open(f"{out}/earth_color.jpg").convert("L"), np.float32)
    col = Image.open(f"{src}/earth_color_12000.jpg").convert("RGB")
    col.draft("RGB", (col.width // 2, col.height // 2))
    ca = np.asarray(col.resize((COLOR_W, COLOR_H), Image.LANCZOS))
    ca, c, rolled = orient(ca, ref)
    log.append(f"earth colour: correlation with the global map {c:.2f}{' (rolled half a turn)' if rolled else ''}, "
               f"{pack_color(Image.fromarray(ca), out + '/earth_color_hi.pak') >> 20} MB")
    # the global map from the same picture: both levels show the same colours (no seam, no change on loading)
    Image.fromarray(ca).resize((2048, 1024), Image.LANCZOS).save(f"{out}/earth_color.jpg", quality=90, optimize=True)
    g = np.asarray(Image.open(f"{src}/earth_srtm.jpg").convert("L"), np.float32)
    SEA = 12.0
    x = int((86.925 + 180) / 360 * g.shape[1]); y = int((90 - 27.988) / 180 * g.shape[0])
    ev = float(g[y - 8:y + 9, x - 8:x + 9].max())
    he = resize_f(np.maximum(g - SEA, 0.0) * (8848.0 / max(ev - SEA, 1.0)), 8192, 4096)
    he, c, rolled = orient(he, ref_height(f"{out}/earth_height.png"))
    log.append(f"earth elevation 8192 x 4096: {he.min():.0f}..{he.max():.0f} m, correlation {c:.3f}{' (rolled)' if rolled else ''}, "
               f"{pack_height(he, out + '/earth_height_hi.pak') >> 20} MB")

    # ---------------- Moon
    ref = np.asarray(Image.open(f"{out}/moon_color.jpg").convert("L"), np.float32)
    mc = np.asarray(Image.open(f"{src}/moon_color_8k.tif").convert("RGB"))
    mc, c, rolled = orient(mc, ref)
    log.append(f"moon colour: correlation {c:.2f}{' (rolled)' if rolled else ''}, {pack_color(Image.fromarray(mc), out + '/moon_color_hi.pak') >> 20} MB")
    Image.fromarray(mc).resize((2048, 1024), Image.LANCZOS).save(f"{out}/moon_color.jpg", quality=90, optimize=True)
    ml = np.asarray(Image.open(f"{src}/moon_ldem_16.tif"), np.float32)
    if np.nanmax(np.abs(ml)) < 100:  # km
        ml = ml * 1000.0
    ml, c, rolled = orient(ml, ref_height(f"{out}/moon_height.png"))
    log.append(f"moon elevation {ml.shape[1]} x {ml.shape[0]}: {ml.min():.0f}..{ml.max():.0f} m, correlation {c:.3f}{' (rolled)' if rolled else ''}, "
               f"{pack_height(ml, out + '/moon_height_hi.pak') >> 20} MB")

    # ---------------- Mars
    raw = np.fromfile(f"{src}/mars_mola_16.img", dtype=">i2").reshape(2880, 5760).astype(np.float32)
    mola, c, rolled = orient(raw, ref_height(f"{out}/mars_height.png"))
    yy, xx = np.unravel_index(np.argmax(mola), mola.shape)
    log.append(f"mars elevation 5760 x 2880: {mola.min():.0f}..{mola.max():.0f} m, correlation {c:.3f}{' (rolled)' if rolled else ''}, "
               f"highest at {90 - (yy + 0.5) / 16:.1f} N {(xx + 0.5) / 16 - 180:.1f} E, {pack_height(mola, out + '/mars_height_hi.pak') >> 20} MB")
    ref = np.asarray(Image.open(f"{out}/mars_color.jpg").convert("L"), np.float32)
    m = Image.open(f"{src}/mars_color.jpg")
    m.draft("RGB", (m.width // 2, m.height // 2))  # DCT scaling: decodes at 1/2 size (still above 8192)
    ma = np.asarray(m.convert("RGB").resize((COLOR_W, COLOR_H), Image.LANCZOS))
    ma, c, rolled = orient(ma, ref)
    log.append(f"mars colour: correlation {c:.2f}{' (rolled)' if rolled else ''}, {pack_color(Image.fromarray(ma), out + '/mars_color_hi.pak') >> 20} MB")
    print("\n".join(log))


if __name__ == "__main__":
    main()
