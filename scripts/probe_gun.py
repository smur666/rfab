import struct, sys

path = sys.argv[1] if len(sys.argv) > 1 else r"C:\workspace\m5stackTab5\elena\gun.rfav"
d = open(path, "rb").read()
n = len(d)
print("file size", n)

magic = d[:4]
ver = struct.unpack_from("<H", d, 4)[0]
w = struct.unpack_from("<H", d, 6)[0]
h = struct.unpack_from("<H", d, 8)[0]
fps = d[10]
flags = d[11]
fc = struct.unpack_from("<H", d, 12)[0]
ft_off = struct.unpack_from("<I", d, 16)[0]
fd_off = struct.unpack_from("<I", d, 20)[0]
fd_sz = struct.unpack_from("<I", d, 24)[0]
hdr_crc = struct.unpack_from("<I", d, 28)[0]
print(f"magic={magic} ver={ver} {w}x{h} fps={fps} flags=0x{flags:02x} frames={fc}")
print(f"table@{ft_off:#x} data@{fd_off:#x} data_sz={fd_sz} hdr_crc={hdr_crc:#010x}")

# frame table entries
i = ft_off
lens = []
maxj = 0
for k in range(fc):
    o, j, r, c = struct.unpack_from("<IIII", d, i + k * 16)
    lens.append((o, j))
    maxj = max(maxj, j)
print("first 3 entries:", lens[:3])
print("last entry:", lens[-1], "max jpeg_len:", maxj)
print("mask bytes:", w * h // 8, "(1-bit)  | 4-bit would be", w * h // 2)

# scan JPEGs back-to-back from data start as sanity
i = fd_off
cnt = 0
while i + 2 < n and d[i:i + 2] == b"\xff\xd8":
    j = d.find(b"\xff\xd9", i)
    if j < 0:
        print("SOI without EOI at", i)
        break
    cnt += 1
    if cnt <= 2:
        print(f"frame {cnt}: off={i:#x} jpeg_len={j - i + 2}")
    i = j + 2
print("jpeg scan count:", cnt)
print("tail 16B:", d[-16:].hex())
