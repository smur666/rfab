import struct, sys

def jpeg_size(path):
    with open(path, "rb") as f:
        d = f.read()
    if d[:2] != b"\xff\xd8":
        return "NOT A JPEG (magic %s)" % d[:4].hex()
    i = 2
    while i < len(d) - 9:
        if d[i] != 0xFF:
            i += 1
            continue
        m = d[i + 1]
        if m in (0xC0, 0xC1, 0xC2, 0xC3):
            h, w = struct.unpack(">HH", d[i + 5 : i + 9])
            return "JPEG %dx%d (SOF%d)" % (w, h, m - 0xC0)
        if m == 0xD8 or m == 0x01 or 0xD0 <= m <= 0xD7:
            i += 2
            continue
        if m == 0xDA:
            break  # start of scan — no more headers
        ln = struct.unpack(">H", d[i + 2 : i + 4])[0]
        i += 2 + ln
    return "JPEG (no SOF found before scan)"

p = sys.argv[1]
print(p, "->", jpeg_size(p))
