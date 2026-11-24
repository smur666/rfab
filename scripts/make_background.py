"""Generate a 720x1280 (portrait) checkerboard background.jpg for the
RFAV-3 POC player — matches the pre-rotated portrait RFAV frames and the
glass-native 720x1280 draw buffer.

Matches the in-code fallback colors (dark 24,28,36 / lite 96,104,116), 64 px
squares. Written next to the rfav so the Tab5's P4 HW JPEG decoder picks it up
at /avatar/background.jpg. Kept well under the app's 1 MB limit.
"""
import os
from PIL import Image

W, H, SQ = 720, 1280, 64
DARK = (24, 28, 36)
LITE = (96, 104, 116)

img = Image.new("RGB", (W, H))
px = img.load()
for y in range(H):
    for x in range(W):
        px[x, y] = LITE if (((x // SQ) + (y // SQ)) & 1) else DARK

out_dir = os.path.join(os.path.dirname(__file__), "..", "sd_stage", "avatar")
os.makedirs(out_dir, exist_ok=True)
out = os.path.normpath(os.path.join(out_dir, "background.jpg"))
img.save(out, "JPEG", quality=90)
print(f"wrote {out}  [{os.path.getsize(out)} B]")
