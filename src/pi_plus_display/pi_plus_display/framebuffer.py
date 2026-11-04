# Bild -> SSD1309-Speicherlayout und UART-Rahmen (Vorschlag fuer die eigene Firmware).
# SSD1309 im Page-Modus: 8 Pages a 8 Zeilen, je Page 128 Bytes,
# ein Byte = 8 senkrechte Pixel einer Spalte, Bit 0 oben. Zusammen 1024 Byte.
# Achtung: Im Rahmenformat A5 5A LEN TYP ... 66 47 74 ist LEN nur ein Byte
# (max. 255). Ein ganzer Framebuffer passt also nicht in einen Rahmen. Vorschlag:
# pro Page ein Rahmen vom Typ 0x24 mit [Page-Index, 128 Datenbytes, CRC16].

import struct

FRAME_HEAD = b"\xa5\x5a"
FRAME_TAIL = b"\x66\x47\x74"
TYPE_FB_PAGE = 0x24


# PIL-Bild 128x64 (Modus "1") -> 1024 Bytes im SSD1309-Layout
def to_pages(img):
    img = img.convert("1")
    w, h = img.size
    px = img.load()
    out = bytearray(w * h // 8)
    for page in range(h // 8):
        for x in range(w):
            b = 0
            for bit in range(8):
                if px[x, page * 8 + bit]:
                    b |= 1 << bit
            out[page * w + x] = b
    return bytes(out)


def crc16_ccitt(data, crc=0xFFFF):
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def frame(typ, payload):
    return FRAME_HEAD + bytes([len(payload) + 1, typ]) + payload + FRAME_TAIL


# Liefert die Rahmen fuer ein Bild. Mit changed_only_from (vorheriges
# 1024-Byte-Abbild) werden nur geaenderte Pages geschickt, das spart Bandbreite.
def framebuffer_frames(img, changed_only_from=None):
    data = to_pages(img)
    frames = []
    for page in range(8):
        chunk = data[page * 128 : (page + 1) * 128]
        if changed_only_from is not None and changed_only_from[page * 128 : (page + 1) * 128] == chunk:
            continue
        payload = bytes([page]) + chunk
        payload += struct.pack("<H", crc16_ccitt(payload))
        frames.append(frame(TYPE_FB_PAGE, payload))
    return frames, data
