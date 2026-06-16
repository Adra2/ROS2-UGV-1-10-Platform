"""SBUS frame parser — pure Python, no external dependencies.

SBUS protocol (Futaba / FrSky):
  - 25 bytes per frame
  - Byte 0:    0x0F  (start byte)
  - Bytes 1-22: 16 channels packed as 11-bit values, LSB first
  - Byte 23:   flags (failsafe, frame-lost, ch17, ch18)
  - Byte 24:   0x00  (end byte)

Raw channel values range: 172–1811
  - Centre: ~992
  - Full low: 172
  - Full high: 1811
"""

SBUS_START = 0x0F
SBUS_END = 0x00
SBUS_FRAME_LEN = 25

# Flag byte (byte 23) bitmasks
FLAG_CH17 = 0x01
FLAG_CH18 = 0x02
FLAG_FRAME_LOST = 0x04
FLAG_FAILSAFE = 0x08

# Raw value range constants
RAW_MIN = 172
RAW_MID = 992
RAW_MAX = 1811


def parse_frame(frame: bytes) -> dict | None:
    """Parse a 25-byte SBUS frame.

    Returns a dict with keys:
      channels  : list of 16 raw int values (172-1811)
      failsafe  : bool
      frame_lost: bool
    Returns None if the frame is invalid.
    """
    if len(frame) != SBUS_FRAME_LEN:
        return None
    if frame[0] != SBUS_START or frame[24] != SBUS_END:
        return None

    # Unpack 16 x 11-bit channels from bytes 1-22
    # Each channel is 11 bits, packed LSB-first across byte boundaries
    data = frame[1:23]
    channels = []
    bit_pos = 0

    for _ in range(16):
        value = 0
        for bit in range(11):
            byte_idx = (bit_pos + bit) // 8
            bit_idx = (bit_pos + bit) % 8
            if data[byte_idx] & (1 << bit_idx):
                value |= (1 << bit)
        channels.append(value)
        bit_pos += 11

    flags = frame[23]

    return {
        'channels': channels,
        'failsafe': bool(flags & FLAG_FAILSAFE),
        'frame_lost': bool(flags & FLAG_FRAME_LOST),
    }


def raw_to_normalized(raw: int) -> float:
    """Convert raw SBUS value (172-1811) to normalized float (-1.0 to +1.0).

    Centre (992) maps to 0.0.
    """
    raw = max(RAW_MIN, min(RAW_MAX, raw))
    if raw >= RAW_MID:
        return (raw - RAW_MID) / (RAW_MAX - RAW_MID)
    else:
        return (raw - RAW_MID) / (RAW_MID - RAW_MIN)


def raw_to_mode(raw: int) -> int:
    """Convert a 3-position switch channel to mode int (0, 1, 2).

    Typical Futaba C switch positions:
      Low  (~172):  mode 0 — RC manual
      Mid  (~992):  mode 1 — autonomous
      High (~1811): mode 2 — failsafe / stop
    """
    if raw < 500:
        return 0
    elif raw < 1400:
        return 1
    else:
        return 2