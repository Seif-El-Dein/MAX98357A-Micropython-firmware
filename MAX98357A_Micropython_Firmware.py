# ===== Robust WAV player for Pico W + MAX98357A (MicroPython) =====
# Handles: PCM (8/16/24/32), float32 WAV, μ-law (G.711u), A-law (G.711a), IMA ADPCM
# Mono/Stereo; broad sample-rate support. Streams + converts on the fly.
# ------------------------------------------------------------------

import machine
from machine import Pin, SPI, I2S
import uos
import struct

# ---------- TUNABLES ----------
CPU_FREQ_HZ = 250_000_000
SD_SPI_ID   = 0
SD_SCK_PIN  = 2
SD_MOSI_PIN = 3
SD_MISO_PIN = 4
SD_CS_PIN   = 5
SD_SPI_BAUD = 50_000_000

WAV_FILE_PATH = "/sd/h01.WAV"

# Default audio config if header is missing/unexpected
DEF_SAMPLE_RATE = 32000
DEF_BITS        = 32
DEF_FORMAT      = I2S.MONO

# I2S pins (Pico W)
I2S_ID = 1
SCK_PIN = 16   # BCLK
WS_PIN  = 17   # LRCK
SD_PIN  = 18   # DIN

# I2S internal ring buffer (bytes). Large ibuf reduces IRQ pressure.
I2S_IBUF_BYTES = 12288

# Size of bytes read from the WAV file per iteration (will be aligned)
READ_BUF_BYTES = 8192
# ------------------------------

# --- SD init (compatible across firmwares) ---
_sd_mod = None
for _name in ("hardware.sdcard", "sdcard", "machine"):
    try:
        _sd_mod = __import__(_name)
        break
    except:
        pass
if _sd_mod is None:
    raise RuntimeError("No sdcard module found (expected hardware.sdcard / sdcard / machine)")

SDCard = None
if hasattr(_sd_mod, "SDCard"):
    SDCard = _sd_mod.SDCard
elif hasattr(_sd_mod, "sdcard") and hasattr(_sd_mod.sdcard, "SDCard"):
    SDCard = _sd_mod.sdcard.SDCard
else:
    raise RuntimeError("Could not locate SDCard class in available modules")

try:
    machine.freq(CPU_FREQ_HZ)
except:
    pass

spi = SPI(
    SD_SPI_ID,
    baudrate=400_000,
    polarity=0,
    phase=0,
    bits=8,
    firstbit=SPI.MSB,
    sck=Pin(SD_SCK_PIN),
    mosi=Pin(SD_MOSI_PIN),
    miso=Pin(SD_MISO_PIN),
)
cs = Pin(SD_CS_PIN, Pin.OUT, value=1)

sd = SDCard(spi, cs)
try:
    if hasattr(sd, "init_spi"):
        sd.init_spi(SD_SPI_BAUD)
    else:
        spi.init(baudrate=SD_SPI_BAUD, polarity=0, phase=0, bits=8, firstbit=SPI.MSB)
except:
    pass

try:
    uos.mount(sd, "/sd")
except OSError:
    pass

# ---------- WAV parsing ----------
def _read_exact(f, n):
    b = f.read(n)
    if b is None or len(b) < n:
        raise EOFError("Unexpected EOF")
    return b

def parse_wav_header(f):
    """
    Parse RIFF/WAVE header including 'fmt ' extras.
    Supports WAVE_FORMAT_EXTENSIBLE (65534) by resolving SubFormat to PCM/float/etc.
    Returns dict:
      {
        'audio_format': int,            # 1=PCM, 3=float32, 6=A-law, 7=μ-law, 17=IMA ADPCM (or as resolved)
        'channels': int,
        'sample_rate': int,
        'bits_per_sample': int,         # original (or valid bits if provided)
        'block_align': int,
        'avg_bytes_per_sec': int,
        'samples_per_block': int|None,  # for IMA ADPCM
        'data_offset': int,
        'data_size': int|None
      }
    """
    f.seek(0)
    def _read_exact(ff, n):
        b = ff.read(n)
        if b is None or len(b) < n:
            raise EOFError("Unexpected EOF")
        return b

    hdr = _read_exact(f, 12)
    if hdr[0:4] != b'RIFF' or hdr[8:12] != b'WAVE':
        raise ValueError("Not a RIFF/WAVE file")

    info = {
        'audio_format': None,
        'channels': None,
        'sample_rate': None,
        'bits_per_sample': None,
        'block_align': None,
        'avg_bytes_per_sec': None,
        'samples_per_block': None,
        'data_offset': None,
        'data_size': None
    }

    while True:
        chunk = f.read(8)
        if not chunk or len(chunk) < 8:
            break
        c_id = chunk[0:4]
        c_sz = struct.unpack_from("<I", chunk, 4)[0]

        if c_id == b"fmt ":
            fmt = _read_exact(f, c_sz)
            if len(fmt) < 16:
                raise ValueError("fmt chunk too small")

            (audio_format, channels, sample_rate,
             avg_bytes_per_sec, block_align, bits_per_sample) = struct.unpack_from("<HHIIHH", fmt, 0)

            info['audio_format'] = audio_format
            info['channels'] = channels
            info['sample_rate'] = sample_rate
            info['avg_bytes_per_sec'] = avg_bytes_per_sec
            info['block_align'] = block_align
            info['bits_per_sample'] = bits_per_sample

            # Handle fmt extras
            if c_sz >= 18:
                cb_size = struct.unpack_from("<H", fmt, 16)[0]
                # IMA ADPCM: cb_size >= 2 and SamplesPerBlock at offset 18
                if cb_size >= 2 and len(fmt) >= 18 + cb_size:
                    # For IMA ADPCM, this is meaningful; for other formats, presence is harmless
                    info['samples_per_block'] = struct.unpack_from("<H", fmt, 18)[0]

                # WAVE_FORMAT_EXTENSIBLE (65534) has at least 22 extra bytes
                if audio_format == 65534 and cb_size >= 22 and len(fmt) >= 18 + 22:
                    # Structure after cbSize:
                    # +0: wValidBitsPerSample (2 bytes)
                    # +2: dwChannelMask       (4 bytes)
                    # +6: SubFormat GUID      (16 bytes)
                    w_valid_bits = struct.unpack_from("<H", fmt, 18)[0]
                    # dwChannelMask = struct.unpack_from("<I", fmt, 20)[0]  # not needed for playback
                    subfmt_first_dword = struct.unpack_from("<I", fmt, 24)[0]

                    # If valid bits look sane, prefer them
                    if 8 <= w_valid_bits <= 32:
                        info['bits_per_sample'] = w_valid_bits

                    # Map SubFormat GUID to a classic WAVE tag:
                    # 0x00000001 -> PCM, 0x00000003 -> IEEE_FLOAT,
                    # 0x00000006 -> A-law, 0x00000007 -> μ-law, 0x00000011 -> IMA ADPCM
                    if subfmt_first_dword in (1, 3, 6, 7, 17):
                        info['audio_format'] = subfmt_first_dword
                    else:
                        # Unknown subformat – keep as extensible; the caller will error out friendlily
                        pass

        elif c_id == b"data":
            info['data_offset'] = f.tell()
            info['data_size'] = c_sz
            # Stop at the start of the data
            break
        else:
            # Skip any other chunk
            f.seek(c_sz, 1)

    # Fill safe defaults if header was incomplete
    if (info['sample_rate'] is None) or (info['channels'] is None):
        info['sample_rate'] = info['sample_rate'] or 32000
        info['channels'] = info['channels'] or 1
        info['bits_per_sample'] = info['bits_per_sample'] or 16
        info['audio_format'] = info['audio_format'] or 1
        info['data_offset'] = info['data_offset'] or 44

    return info


def i2s_format_for_channels(ch):
    return I2S.MONO if ch == 1 else I2S.STEREO

def make_i2s_tx(sample_rate, bits_out, fmt):
    return I2S(
        I2S_ID,
        sck=Pin(SCK_PIN),
        ws=Pin(WS_PIN),
        sd=Pin(SD_PIN),
        mode=I2S.TX,
        bits=bits_out,
        format=fmt,
        rate=sample_rate,
        ibuf=I2S_IBUF_BYTES,
    )

def make_aligned_buffer(buf_bytes, bytes_per_frame):
    if bytes_per_frame <= 0:
        bytes_per_frame = 2
    n_frames = max(1, buf_bytes // bytes_per_frame)
    size = n_frames * bytes_per_frame
    ba = bytearray(size)
    return ba, memoryview(ba), bytes_per_frame

# ---------- Converters ----------
# μ-law / A-law (G.711) decode to int16
# Algorithmic decode (no 256-entry table to save RAM)
def mulaw_to_int16(u):
    u ^= 0xFF
    sign = u & 0x80
    exponent = (u >> 4) & 0x07
    mantissa = u & 0x0F
    sample = ((mantissa << 3) + 0x84) << exponent
    sample -= 0x84
    if sign:
        sample = -sample
    # clamp to int16
    if sample > 32767: sample = 32767
    if sample < -32768: sample = -32768
    return sample

def alaw_to_int16(a):
    a ^= 0x55
    sign = a & 0x80
    exponent = (a >> 4) & 0x07
    mantissa = a & 0x0F
    if exponent == 0:
        sample = (mantissa << 4) + 8
    else:
        sample = ((mantissa << 4) + 0x108) << (exponent - 1)
    if sign:
        sample = -sample
    if sample > 32767: sample = 32767
    if sample < -32768: sample = -32768
    return sample

# IMA ADPCM step/index tables
_IMA_INDEX_TAB = (-1, -1, -1, -1, 2, 4, 6, 8)
_IMA_STEP_TAB = (
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28,
    31, 34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107,
    118, 130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371,
    408, 449, 494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282,
    1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026,
    4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635,
    13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
)

def ima_adpcm_decode_block_stereo(block, samples_per_block):
    """
    Decode a single interleaved IMA ADPCM stereo block to list[int] interleaved L,R int16.
    WAV IMA layout: For stereo, the block contains a header for each channel followed by interleaved nibbles.
    However in WAV, nibbles are actually stored as separate channel streams (not strictly interleaved per nibble).
    Standard layout: [ch0 header][ch1 header][ch0 nibbles...][ch1 nibbles...]
    We'll decode both and then interleave samples.
    """
    # Each channel header: 4 bytes: predictor(int16 LE), index(uint8), reserved(uint8)
    import struct as _s
    o = 0
    # ch0 header
    pred0, = _s.unpack_from("<h", block, o); o += 2
    idx0 = block[o]; o += 1
    o += 1  # reserved
    # ch1 header
    pred1, = _s.unpack_from("<h", block, o); o += 2
    idx1 = block[o]; o += 1
    o += 1

    if idx0 < 0: idx0 = 0
    if idx0 > 88: idx0 = 88
    if idx1 < 0: idx1 = 0
    if idx1 > 88: idx1 = 88

    # First sample is the predictor
    outL = [pred0]
    outR = [pred1]

    # Number of ADPCM codes per channel after headers
    # Each nibble is a sample delta; samples_per_block includes the initial predictor
    samp_to_decode = samples_per_block - 1
    # bytes per channel stream
    bytes_per_ch = (samp_to_decode + 1) // 2

    # channel streams
    ch0_stream = block[o:o+bytes_per_ch]; o += bytes_per_ch
    ch1_stream = block[o:o+bytes_per_ch]; o += bytes_per_ch

    # decode function
    def _dec_stream(stream, pred, idx):
        res = []
        step_tab = _IMA_STEP_TAB
        index_tab = _IMA_INDEX_TAB
        for b in stream:
            # low then high nibble
            for nib in (b & 0x0F, (b >> 4) & 0x0F):
                step = step_tab[idx]
                diff = step >> 3
                if nib & 1: diff += step >> 2
                if nib & 2: diff += step >> 1
                if nib & 4: diff += step
                if nib & 8: diff = -diff
                pred += diff
                if pred > 32767: pred = 32767
                if pred < -32768: pred = -32768
                idx += index_tab[nib & 7]
                if idx < 0: idx = 0
                if idx > 88: idx = 88
                res.append(pred)
                if len(res) >= samp_to_decode:
                    break
            if len(res) >= samp_to_decode:
                break
        return res, pred, idx

    dl, pred0, idx0 = _dec_stream(ch0_stream, pred0, idx0)
    dr, pred1, idx1 = _dec_stream(ch1_stream, pred1, idx1)

    # Interleave including initial predictors
    out = []
    # first sample
    out.append(outL[0]); out.append(outR[0])
    n = min(len(dl), len(dr))
    for i in range(n):
        out.append(dl[i]); out.append(dr[i])
    return out

def ima_adpcm_decode_block_mono(block, samples_per_block):
    import struct as _s
    o = 0
    pred, = _s.unpack_from("<h", block, o); o += 2
    idx = block[o]; o += 1
    o += 1  # reserved

    if idx < 0: idx = 0
    if idx > 88: idx = 88

    out = [pred]
    samp_to_decode = samples_per_block - 1
    # remaining bytes are nibbles
    for b in block[o:]:
        for nib in (b & 0x0F, (b >> 4) & 0x0F):
            step = _IMA_STEP_TAB[idx]
            diff = step >> 3
            if nib & 1: diff += step >> 2
            if nib & 2: diff += step >> 1
            if nib & 4: diff += step
            if nib & 8: diff = -diff
            pred += diff
            if pred > 32767: pred = 32767
            if pred < -32768: pred = -32768
            idx += _IMA_INDEX_TAB[nib & 7]
            if idx < 0: idx = 0
            if idx > 88: idx = 88
            out.append(pred)
            if len(out) >= samples_per_block:
                break
        if len(out) >= samples_per_block:
            break
    return out

# ---------- Streaming playback ----------
def play_wav(path):
    f = open(path, "rb")
    try:
        info = parse_wav_header(f)
        sr   = info['sample_rate'] or DEF_SAMPLE_RATE
        ch   = info['channels'] or 1
        fmt  = info['audio_format'] or 1
        bps  = info['bits_per_sample'] or 16
        blk  = info['block_align'] or 0
        spb  = info['samples_per_block']  # may be None
        data_off = info['data_offset'] or 44
        data_sz  = info['data_size']      # may be None (treat as unknown/stream)

        mp_fmt = i2s_format_for_channels(ch)

        # Decide output width and path
        # We'll use 16-bit output for: PCM8/16, μ-law, A-law, IMA ADPCM
        # We'll use 32-bit output for: PCM32, PCM24 (expanded), float32
        is_pcm = (fmt == 1)
        is_float = (fmt == 3)
        is_alaw  = (fmt == 6)
        is_mulaw = (fmt == 7)
        is_ima   = (fmt == 17)

        if is_pcm:
            if bps in (8, 16):
                bits_out = 16
                out_frame_bytes = 2 * ch
                passthrough_16 = (bps == 16)
            elif bps == 24:
                bits_out = 32  # expand to s32
                out_frame_bytes = 4 * ch
                passthrough_16 = False
            elif bps == 32:
                bits_out = 32
                out_frame_bytes = 4 * ch
                passthrough_16 = False
            else:
                # Unusual PCM (e.g., 20-bit). Expand to 32.
                bits_out = 32
                out_frame_bytes = 4 * ch
                passthrough_16 = False
        elif is_float:
            bits_out = 32
            out_frame_bytes = 4 * ch
            passthrough_16 = False
        elif is_alaw or is_mulaw or is_ima:
            bits_out = 16
            out_frame_bytes = 2 * ch
            passthrough_16 = False
        else:
            raise ValueError("Unsupported WAV format code: {}".format(fmt))

        # Seek to data start
        f.seek(data_off)

        # Create I2S
        i2s = make_i2s_tx(sr, bits_out, mp_fmt)

        try:
            # Buffers
            # Input buffer should hold a multiple of encoded frame/chunk size.
            in_chunk = READ_BUF_BYTES
            in_buf  = bytearray(in_chunk)
            in_mv   = memoryview(in_buf)
            # Output buffer aligned to full frames (destination width)
            out_buf, out_mv, _ = make_aligned_buffer(READ_BUF_BYTES * (4 if bits_out == 32 else 2),
                                                     out_frame_bytes)

            # Helper: write out (full frames)
            def _write_bytes(bmv, nbytes):
                # trim to whole frames
                nbytes = (nbytes // out_frame_bytes) * out_frame_bytes
                if nbytes:
                    i2s.write(bmv[:nbytes])

            if is_pcm and bps == 16 and passthrough_16:
                # Fast path: just stream as-is (already little-endian int16)
                # Read -> write directly (ensure whole frames)
                while True:
                    n = f.readinto(in_mv)
                    if not n:
                        break
                    # ensure even number for stereo/mono frames
                    frame_bytes_in = 2 * ch
                    n = (n // frame_bytes_in) * frame_bytes_in
                    if n:
                        i2s.write(in_mv[:n])

            elif is_pcm and bps == 8:
                # Unsigned 8-bit -> signed 16-bit
                import struct as _s
                o = 0
                while True:
                    n = f.readinto(in_mv)
                    if not n:
                        break
                    # bytes to samples
                    nsamp = n
                    # convert in chunks
                    out_off = 0
                    for i in range(nsamp):
                        v = in_buf[i] - 128  # [-128..127]
                        v = v << 8           # to int16
                        _s.pack_into("<h", out_buf, out_off, v)
                        out_off += 2
                        if ch == 2 and (i % ch) == 0 and ch == 2:
                            # nothing special; 8-bit stereo arrives interleaved already
                            pass
                    _write_bytes(out_mv, out_off)

            elif is_pcm and bps == 24:
                # 24-bit little-endian packed -> 32-bit signed (left-justified into 32)
                import struct as _s
                in_frame = 3 * ch
                # ensure in_buf reads multiples of 3*ch
                while True:
                    n = f.readinto(in_mv)
                    if not n:
                        break
                    n = (n // in_frame) * in_frame
                    out_off = 0
                    i = 0
                    while i < n:
                        # read 3 bytes, sign-extend to 32-bit
                        b0 = in_buf[i]; b1 = in_buf[i+1]; b2 = in_buf[i+2]
                        i += 3
                        val = b0 | (b1 << 8) | (b2 << 16)
                        if val & 0x800000:
                            val -= 0x1000000  # sign extend 24->32
                        _s.pack_into("<i", out_buf, out_off, val << 8)  # align to 32-bit (LSB padded)
                        out_off += 4
                    _write_bytes(out_mv, out_off)

            elif is_pcm and bps == 32:
                # Already 32-bit signed little-endian; stream
                while True:
                    n = f.readinto(in_mv)
                    if not n:
                        break
                    n = (n // (4 * ch)) * (4 * ch)
                    if n:
                        i2s.write(in_mv[:n])

            elif is_float:
                # float32 -> int32 (clip)
                import struct as _s
                SCALE = 2147483647.0
                in_frame = 4 * ch
                while True:
                    n = f.readinto(in_mv)
                    if not n:
                        break
                    n = (n // in_frame) * in_frame
                    if not n:
                        continue
                    nsamp = n // 4
                    out_off = 0
                    # unpack floats in chunks
                    # NOTE: unpacking big arrays at once is faster than per-sample pack/unpack
                    floats = _s.unpack_from("<%df" % nsamp, in_buf, 0)
                    for v in floats:
                        if v > 1.0: v = 1.0
                        elif v < -1.0: v = -1.0
                        iv = int(v * SCALE)
                        _s.pack_into("<i", out_buf, out_off, iv)
                        out_off += 4
                    _write_bytes(out_mv, out_off)

            elif is_mulaw or is_alaw:
                # 8-bit compressed per sample -> int16
                import struct as _s
                decoder = mulaw_to_int16 if is_mulaw else alaw_to_int16
                while True:
                    n = f.readinto(in_mv)
                    if not n:
                        break
                    out_off = 0
                    for i in range(n):
                        s16 = decoder(in_buf[i])
                        _s.pack_into("<h", out_buf, out_off, s16)
                        out_off += 2
                    _write_bytes(out_mv, out_off)

            elif is_ima:
                # IMA ADPCM block-based
                # Need block_align and samples_per_block
                if not blk or not spb:
                    raise ValueError("IMA ADPCM missing block_align/samples_per_block")
                # Read per block, decode to int16 PCM
                import struct as _s
                while True:
                    block = f.read(blk)
                    if not block or len(block) < blk:
                        break
                    if ch == 1:
                        samples = ima_adpcm_decode_block_mono(block, spb)
                        # pack to bytes
                        out_off = 0
                        # interleaving is trivial (mono)
                        for s in samples:
                            _s.pack_into("<h", out_buf, out_off, s)
                            out_off += 2
                        _write_bytes(out_mv, out_off)
                    else:
                        # stereo
                        samples = ima_adpcm_decode_block_stereo(block, spb)
                        out_off = 0
                        for s in samples:
                            _s.pack_into("<h", out_buf, out_off, s)
                            out_off += 2
                        _write_bytes(out_mv, out_off)

            else:
                raise ValueError("Unhandled format combination")

        finally:
            i2s.deinit()

    finally:
        f.close()

# ------------------ RUN ------------------
if __name__ == "__main__":
    try:
        try:
            print("SD root:", uos.listdir("/sd"))
        except:
            pass
        play_wav(WAV_FILE_PATH)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # Graceful error with hint
        print("Playback error:", e)
        print("Tip: If this is an unusual codec, consider converting to PCM 16/32.")
