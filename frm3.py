#!/usr/bin/env python3
"""Host driver for the arduino_frm3_bdm firmware.

Talks the line-oriented text protocol the firmware exposes over USB
CDC. Provides a Python API on top, plus a CLI for common operations.

The FSEC sector (last 1 KB of P-Flash) is skipped during programming.
Erasing it would set SEC[1:0]=11 on the next reset and re-secure the
chip.
"""
import sys, os, re, time, argparse
import serial

# MC9S12XEQ384 P-Flash memory map (RM Table 1-5, note 5):
#
#   Block 1: 128 KB at global 0x780000..0x79FFFF  (PPAGE 0xE0..0xE7)
#   Gap   :          0x7A0000..0x7BFFFF           (PPAGE 0xE8..0xEF, unimplemented)
#   Block 2: 256 KB at global 0x7C0000..0x7FFFFF  (PPAGE 0xF0..0xFF)
#
# A 384 KB dump file is the two blocks concatenated:
#   file bytes 0x00000..0x1FFFF (128 KB) -> Block 1
#   file bytes 0x20000..0x5FFFF (256 KB) -> Block 2
#
# Use src_off_to_global() / sector_to_global() for file-offset to chip
# address translation; the sector index space (0..383) is contiguous and
# the helpers skip the gap.
PFLASH_BLOCK1_BASE = 0x780000
PFLASH_BLOCK1_SIZE = 0x20000        # 128 KB
PFLASH_BLOCK2_BASE = 0x7C0000
PFLASH_BLOCK2_SIZE = 0x40000        # 256 KB
PFLASH_SIZE        = PFLASH_BLOCK1_SIZE + PFLASH_BLOCK2_SIZE   # 384 KB
PFLASH_SECTOR      = 1024
PFLASH_NSECTORS    = PFLASH_SIZE // PFLASH_SECTOR              # 384

# FSEC sector is the last sector of Block 2.
FSEC_SECTOR_BASE   = PFLASH_BLOCK2_BASE + PFLASH_BLOCK2_SIZE - PFLASH_SECTOR

# Alias kept for callers that pass a single base. Prefer src_off_to_global()
# in new code so the split layout is handled.
PFLASH_BASE = PFLASH_BLOCK1_BASE


def src_off_to_global(off):
    """File offset (0..PFLASH_SIZE-1) -> chip global address."""
    if 0 <= off < PFLASH_BLOCK1_SIZE:
        return PFLASH_BLOCK1_BASE + off
    if PFLASH_BLOCK1_SIZE <= off < PFLASH_SIZE:
        return PFLASH_BLOCK2_BASE + (off - PFLASH_BLOCK1_SIZE)
    raise ValueError(f"P-Flash file offset 0x{off:X} out of range")


def sector_to_global(sector_idx):
    """Sector index (0..PFLASH_NSECTORS-1) -> chip global address."""
    return src_off_to_global(sector_idx * PFLASH_SECTOR)


def global_to_src_off(ga):
    """Chip global P-Flash address -> file byte offset.
    Returns None for addresses outside the two blocks (e.g. the gap)."""
    if PFLASH_BLOCK1_BASE <= ga < PFLASH_BLOCK1_BASE + PFLASH_BLOCK1_SIZE:
        return ga - PFLASH_BLOCK1_BASE
    if PFLASH_BLOCK2_BASE <= ga < PFLASH_BLOCK2_BASE + PFLASH_BLOCK2_SIZE:
        return PFLASH_BLOCK1_SIZE + (ga - PFLASH_BLOCK2_BASE)
    return None

DFLASH_BASE      = 0x100000
DFLASH_SIZE      = 0x8000           # 32 KB
DFLASH_SECTOR    = 256

READ_CHUNK       = 1024             # firmware caps rpf/rdflash at 4096;
                                    # 1 KB transfers are USB-CDC-safe.

EE_SIZE          = 4096             # FRM3 firmware exposes a 4 KB EEPROM


# EEE log encoder / decoder.
#
# The chip stores its emulated EEPROM as a circular log in the 32 KB
# D-Flash: 128 blocks of 256 bytes. Each block is a 4-byte header plus
# 63 four-byte commands. The header is 0xFACFFFFE (VALID) or 0xFFFFFFFF
# (EMPTY / NEW). Each command is a 16-bit (cmd | word_idx) prefix plus
# 2 data bytes. Replaying every VALID command in chronological order
# reconstructs the current 4 KB EEPROM image.
_NB_BLOCKS       = 128
_BLOCKSIZE       = 256
_HEADERSIZE      = 4
_CMDSIZE         = 4
_CMDS_PER_BLOCK  = 63
_BLOCK_VALID     = 0xFACF
_BLOCK_VALID_TAIL= 0xFFFE
_BLOCK_EMPTY     = 0xFFFF
_CMD_MASK        = 0xF800
_CMD_VALID       = 0xB800
_CMD_EMPTY       = 0xF800
_MIN_WORDS_OK    = 16
_BT_EMPTY, _BT_NEW, _BT_VALID, _BT_LAST, _BT_INVALID = range(5)


def encode_eeprom_to_dflash(ee):
    """Encode a 4 KB EEPROM image into the 32 KB EEE-log format the FRM3
    firmware reads. Writes the 33 blocks needed to cover all 2048 words,
    leaves remaining blocks at 0xFF (empty)."""
    if len(ee) != EE_SIZE:
        raise ValueError(f"expected {EE_SIZE}-byte EEPROM, got {len(ee)}")
    df = bytearray(b"\xff" * DFLASH_SIZE)
    word_idx = 0
    num_words = EE_SIZE // 2
    blocks_needed = (num_words + _CMDS_PER_BLOCK - 1) // _CMDS_PER_BLOCK
    for b in range(blocks_needed):
        bo = b * _BLOCKSIZE
        df[bo + 0] = (_BLOCK_VALID >> 8) & 0xFF
        df[bo + 1] = _BLOCK_VALID & 0xFF
        df[bo + 2] = (_BLOCK_VALID_TAIL >> 8) & 0xFF
        df[bo + 3] = _BLOCK_VALID_TAIL & 0xFF
        for c in range(_CMDS_PER_BLOCK):
            if word_idx >= num_words:
                break
            co = bo + _HEADERSIZE + c * _CMDSIZE
            cw = _CMD_VALID | word_idx
            df[co + 0] = (cw >> 8) & 0xFF
            df[co + 1] = cw & 0xFF
            df[co + 2] = ee[word_idx * 2 + 0]
            df[co + 3] = ee[word_idx * 2 + 1]
            word_idx += 1
    return bytes(df)


def _be16(buf, off):
    return (buf[off] << 8) | buf[off + 1]


def dflash_to_eeprom(dflash):
    """Decode 32 KB raw D-Flash → (4 KB EEPROM, dict with ok/corrupt/words_recovered).

    Pass 1 classifies block headers. Pass 2 finds the LAST (partially-filled)
    block. Pass 3 replays VALID commands in chronological order starting from
    the block AFTER the end-block. Words missing from the log default to 0xFF.
    """
    if len(dflash) < _NB_BLOCKS * _BLOCKSIZE:
        return (b"\xff" * EE_SIZE,
                {"ok": False, "corrupt": True, "words_recovered": 0,
                 "endblock": -1, "message": "input too small"})

    block_types = [_BT_INVALID] * _NB_BLOCKS
    first_empty = [-1] * _NB_BLOCKS
    corrupt = False

    # Pass 1 - header classification
    for b in range(_NB_BLOCKS):
        base = b * _BLOCKSIZE
        h0, h1 = _be16(dflash, base), _be16(dflash, base + 2)
        if h0 == _BLOCK_EMPTY:
            block_types[b] = _BT_EMPTY if h1 == _BLOCK_EMPTY else _BT_NEW
        elif h0 == _BLOCK_VALID:
            block_types[b] = _BT_VALID
        else:
            block_types[b] = _BT_INVALID
            corrupt = True

    # Pass 2 - find LAST blocks (VALID with at least one empty command slot)
    for b in range(_NB_BLOCKS):
        if block_types[b] != _BT_VALID: continue
        cmd_base = b * _BLOCKSIZE + _HEADERSIZE
        for i in range(_CMDS_PER_BLOCK):
            info = _be16(dflash, cmd_base + i * _CMDSIZE)
            cmd = info & _CMD_MASK
            if cmd == _CMD_EMPTY:
                block_types[b] = _BT_LAST
                first_empty[b] = i
                break
            elif cmd != _CMD_VALID:
                corrupt = True

    # Determine endblock
    endblock = -1
    last_idxs = [i for i, t in enumerate(block_types) if t == _BT_LAST]
    if len(last_idxs) == 1:
        endblock = last_idxs[0]
    if endblock < 0:
        # Find NEW chain - endblock = block just before
        scan = 0
        while scan < _NB_BLOCKS and block_types[scan] in (_BT_NEW, _BT_EMPTY):
            scan += 1
        first_new, ended = -1, False
        for k in range(_NB_BLOCKS * 2):
            idx = (scan + k) % _NB_BLOCKS
            bt = block_types[idx]
            if bt == _BT_NEW:
                if first_new < 0:    first_new = idx
                elif ended:          first_new = -2; break
            elif bt != _BT_EMPTY and first_new >= 0:
                ended = True
            if first_new >= 0 and k > _NB_BLOCKS:
                break
        if first_new >= 0:
            endblock = (first_new + _NB_BLOCKS - 1) % _NB_BLOCKS
    if endblock < 0:
        # Fallback - longest EMPTY/NEW run
        best_start, best_len = 0, 0
        cur_start, cur_len = 0, 0
        for i in range(_NB_BLOCKS * 2):
            idx = i % _NB_BLOCKS
            if block_types[idx] in (_BT_EMPTY, _BT_NEW):
                if cur_len == 0: cur_start = idx
                cur_len += 1
                if cur_len > best_len:
                    best_len = cur_len; best_start = cur_start
            else:
                cur_len = 0
            if cur_len >= _NB_BLOCKS:
                break
        endblock = (best_start + _NB_BLOCKS - 1) % _NB_BLOCKS if best_len else _NB_BLOCKS - 1

    # Pass 3 - replay commands
    ee = bytearray(b"\xff" * EE_SIZE)
    words = 0
    start = (endblock + 1) % _NB_BLOCKS
    for step in range(_NB_BLOCKS):
        b = (start + step) % _NB_BLOCKS
        if block_types[b] not in (_BT_VALID, _BT_LAST): continue
        cmd_base = b * _BLOCKSIZE + _HEADERSIZE
        limit = first_empty[b] if block_types[b] == _BT_LAST and first_empty[b] >= 0 else _CMDS_PER_BLOCK
        for i in range(limit):
            info = _be16(dflash, cmd_base + i * _CMDSIZE)
            if (info & _CMD_MASK) != _CMD_VALID: continue
            addr = (info & ~_CMD_MASK) * 2
            if addr + 1 < EE_SIZE:
                ee[addr]     = dflash[cmd_base + i * _CMDSIZE + 2]
                ee[addr + 1] = dflash[cmd_base + i * _CMDSIZE + 3]
                words += 1

    ok = (words >= _MIN_WORDS_OK)
    return (bytes(ee), {
        "ok": ok, "corrupt": corrupt, "words_recovered": words,
        "endblock": endblock,
        "message": "decode complete" if ok else "no recognizable EEPROM data",
    })


def eeprom_info(ee):
    """Best-effort decode of FRM3 EEPROM metadata. Returns a dict with VIN /
    production date / programming date / part numbers."""
    def _date(yh, yl, mm, dd):
        return f"{ee[dd]:02x}.{ee[mm]:02x}.{ee[yh]:02x}{ee[yl]:02x}"
    def _pn(off, n=6):
        return ee[off:off + n].hex()
    vin_raw = ee[0xFD3:0xFD3 + 17]
    return {
        "valid_magic": ee[0:4] == b"\xDE\xAD\xBE\xEF",
        "vin":         "".join(chr(c) if 0x20 <= c < 0x7F else "?" for c in vin_raw),
        "prod_date":   _date(0xFBB, 0xFBC, 0xFBD, 0xFBE),
        "prog_date":   _date(0xF86, 0xF87, 0xF88, 0xF89),
        "hw_nr":       _pn(0xF97),
        "sw_nr":       _pn(0xF8A),
        "zb_nr":       _pn(0xF65),
        "sticker":     _pn(0xFBF),
    }


# BDM serial driver
class BDM:
    """Line-oriented protocol on top of the Arduino Serial.

    Each command yields one OK/ERR line (and optionally a data line).
    Streaming commands (wpflash/wdflash) follow a '.' ACK protocol per
    256-byte burst between the OK-ready line and the final OK done line.
    """
    def __init__(self, port="/dev/ttyACM0", baud=1_000_000, settle=2.5):
        # Drain anything the Arduino might still be transmitting from a
        # killed previous session: open the port briefly, eat bytes for
        # ~0.5 s, then re-open at the actual baud rate.
        try:
            drain = serial.Serial(port, baud, timeout=0.1)
            t0 = time.time()
            while time.time() - t0 < 0.5:
                drain.read(8192)
            drain.close()
            time.sleep(0.2)
        except Exception:
            pass
        self.ser = serial.Serial(port, baud, timeout=0.5)
        time.sleep(settle)                # let R4 USB CDC enumerate
        self.ser.reset_input_buffer()
        # Drain any output the Arduino started during settle (e.g. a
        # banner from a fresh reset, or trailing bytes from a prior
        # in-flight command response).
        t0 = time.time()
        while time.time() - t0 < 0.4:
            if not self.ser.read(8192):
                break
        self.ser.write(b"\n\n")
        self.ser.flush()
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    # ---- one-shot commands -------------------------------------------------
    def cmd(self, line, timeout=10):
        self.ser.reset_input_buffer()
        self.ser.write((line + "\n").encode()); self.ser.flush()
        deadline = time.time() + timeout
        lines = []
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw: continue
            s = raw.decode("ascii", "replace").rstrip()
            if not s: continue
            lines.append(s)
            if s.startswith("OK ") or s == "OK" or s.startswith("ERR "):
                return "\n".join(lines)
        return "\n".join(lines)

    def enter(self):
        r = self.cmd("enter", timeout=10)
        if "in active BDM" not in r:
            raise IOError(f"enter failed: {r!r}")
        return r

    # ---- typed reads -------------------------------------------------------
    def read_byte(self, addr):
        r = self.cmd(f"rb {addr:X}")
        m = re.search(r"= 0x([0-9A-Fa-f]+)", r)
        if not m: raise IOError(f"rb {addr:04X} failed: {r!r}")
        return int(m.group(1), 16)

    def read_word(self, addr):
        r = self.cmd(f"rw {addr:X}")
        m = re.search(r"= 0x([0-9A-Fa-f]+)", r)
        if not m: raise IOError(f"rw {addr:04X} failed: {r!r}")
        return int(m.group(1), 16)

    # ---- bulk reads --------------------------------------------------------
    def _bulk_read_once(self, cmd, addr, count, timeout=20):
        self.ser.reset_input_buffer()
        self.ser.write(f"{cmd} {addr:X} {count:X}\n".encode())
        self.ser.flush()
        deadline = time.time() + timeout
        buf = b""
        while time.time() < deadline:
            chunk = self.ser.read(8192)
            if chunk: buf += chunk
            if buf.count(b"\n") >= 2: break
            # Check for an ERR line that aborts mid-burst
            if b"ERR " in buf and b"\n" in buf:
                line = buf.split(b"\n", 1)[0].decode("ascii", "replace")
                if "ERR " in line:
                    raise IOError(f"{cmd}: {line.strip()}")
        lines = buf.decode("ascii", "replace").splitlines()
        # Find OK header among lines (the firmware emits OK first now)
        for ln in lines:
            if ln.startswith("ERR "):
                raise IOError(f"{cmd}: {ln.strip()}")
        if len(lines) < 2 or not lines[0].startswith(f"OK {cmd}"):
            raise IOError(f"{cmd} {addr:X} {count:X} failed: header={lines[:1]!r}")
        try:
            data = bytes.fromhex(lines[1].strip())
        except ValueError:
            raise IOError(f"{cmd}: bad hex payload")
        if len(data) != count:
            raise IOError(f"{cmd}: got {len(data)} bytes, expected {count}")
        return data

    def _bulk_read(self, cmd, addr, count, timeout=20, max_attempts=4):
        """Wraps _bulk_read_once with retry on mid-burst BDM failure. The
        firmware now emits ERR mid-stream on detected failures instead of
        returning silent 0xFF sentinels."""
        last_err = None
        for attempt in range(max_attempts):
            try:
                return self._bulk_read_once(cmd, addr, count, timeout)
            except IOError as e:
                last_err = e
                # Brief settle before retry
                time.sleep(0.05)
        raise last_err

    def read_pflash(self, addr, count):
        return self._bulk_read("rpf", addr, count)

    def read_dflash(self, addr, count):
        return self._bulk_read("rdflash", addr, count)

    # ---- streaming program -------------------------------------------------
    def _stream_program(self, cmd, base, data, log=None):
        """Drive wpflash/wdflash with the '.'-ack protocol. Returns the chip's
        final OK / ERR line. If `log` is given, emits a progress line every
        ~2 KB acked so the host sees the write is alive during a long bulk
        burst (e.g. the 32 KB D-Flash write that takes ~30 s)."""
        n = len(data)
        self.ser.reset_input_buffer()
        self.ser.write(f"{cmd} {base:X} {n:X}\n".encode())
        self.ser.flush()

        burst = 256
        deadline = time.time() + 5
        ready = False
        while time.time() < deadline:
            ln = self.ser.readline()
            if not ln: continue
            s = ln.decode("ascii", "replace").rstrip()
            if s.startswith("OK ready"):
                m = re.search(r"OK ready \d+ (\d+)", s)
                if m: burst = int(m.group(1))
                ready = True; break
            if s.startswith("ERR"):
                return s
        if not ready:
            return f"ERR no ready ({cmd} {base:X})"

        off = 0
        t_stream_start = time.time()
        # Threshold for progress logging: only worth it for chunks where the
        # whole transfer takes >~5 s (otherwise the log is noise).
        log_every = (n // 16) if (log and n >= 4096) else 0
        next_log = log_every
        while off < n:
            chunk = data[off : off + min(burst, n - off)]
            self.ser.write(chunk); self.ser.flush()
            off += len(chunk)
            t0 = time.time()
            while time.time() - t0 < 30:
                c = self.ser.read(1)
                if not c: continue
                if c == b".":
                    break
                if c in (b"\r", b"\n"):
                    continue
                if c == b"E":
                    rest = self.ser.readline().decode("ascii", "replace").rstrip()
                    return "E" + rest
            else:
                return f"ERR no ack after {off} bytes"
            if log_every and off >= next_log and off < n:
                dt = time.time() - t_stream_start
                rate = off / max(1e-3, dt) / 1024
                eta = (n - off) / max(1, off / max(1e-3, dt))
                log(f"    {cmd} streaming {off}/{n} ({100*off//n}%) "
                    f"{rate:.1f} KB/s ETA={eta:.0f}s")
                next_log += log_every

        deadline = time.time() + 10
        while time.time() < deadline:
            ln = self.ser.readline()
            if not ln: continue
            s = ln.decode("ascii", "replace").rstrip()
            if s.startswith("OK ") or s.startswith("ERR "):
                return s
        return "ERR no final OK"

    def write_pflash_chunk(self, base, data, log=None):
        return self._stream_program("wpflash", base, data, log=log)

    def write_dflash_chunk(self, base, data, log=None):
        return self._stream_program("wdflash", base, data, log=log)

    # ---- erases ------------------------------------------------------------
    def erase_p_sector(self, addr, timeout=8):
        r = self.cmd(f"epsec {addr:X}", timeout=timeout)
        return r.startswith("OK epsec")

    def erase_d_sector(self, addr, timeout=8):
        r = self.cmd(f"edsec {addr:X}", timeout=timeout)
        return r.startswith("OK edsec")

    def erase_dflash_all(self, timeout=120):
        return self.cmd("edflash", timeout=timeout)


# High-level dump / program / verify
def dump_region(bdm, base, size, reader, label, log=print):
    """Read `size` bytes starting at global `base` via the given bulk reader."""
    out = bytearray()
    t0 = time.time()
    for off in range(0, size, READ_CHUNK):
        n = min(READ_CHUNK, size - off)
        for attempt in range(4):
            try:
                out += reader(base + off, n)
                break
            except IOError as e:
                if attempt == 3:
                    raise
                # Drain any lingering bytes from the failed read before
                # re-entering BDM, otherwise the next command response
                # gets concatenated to the previous payload.
                try:
                    bdm.ser.timeout = 0.2
                    while bdm.ser.read(8192):
                        pass
                finally:
                    bdm.ser.timeout = 0.5
                bdm.enter()
        if (off // READ_CHUNK) % 16 == 0:
            done = off + n
            rate = done / max(1e-3, time.time() - t0) / 1024
            log(f"  {label}: {done}/{size} ({100*done//size}%) {rate:.0f} KB/s")
    log(f"  {label}: done in {time.time()-t0:.1f}s")
    return bytes(out)


def dump_pflash(bdm, outpath, log=print):
    """Dump 384 KB of P-Flash to a single linear file. The chip's two
    physical blocks (128 KB at 0x780000 + 256 KB at 0x7C0000) are read
    separately and concatenated."""
    bdm.enter()
    log(f"Dumping P-Flash ({PFLASH_SIZE//1024} KB) to {outpath}")
    block1 = dump_region(bdm, PFLASH_BLOCK1_BASE, PFLASH_BLOCK1_SIZE,
                         bdm.read_pflash, "P-Flash block 1 (128 KB)", log)
    block2 = dump_region(bdm, PFLASH_BLOCK2_BASE, PFLASH_BLOCK2_SIZE,
                         bdm.read_pflash, "P-Flash block 2 (256 KB)", log)
    data = block1 + block2
    with open(outpath, "wb") as f: f.write(data)
    log(f"Saved {len(data)} bytes")


def dump_dflash(bdm, outpath, log=print):
    bdm.enter()
    log(f"Dumping D-Flash ({DFLASH_SIZE//1024} KB) to {outpath}")
    data = dump_region(bdm, DFLASH_BASE, DFLASH_SIZE, bdm.read_dflash, "D-Flash", log)
    with open(outpath, "wb") as f: f.write(data)
    log(f"Saved {len(data)} bytes")


def program_pflash(bdm, srcpath, log=print, start_sector=0, end_sector=None):
    """Erase + write + read-back-verify each P-Flash sector. If the verify
    diff is non-zero, retry the whole sector up to MAX attempts.

    `start_sector` / `end_sector` (0-based sector indices into P-Flash; valid
    range 0..383) let you resume mid-flash after a failure or program a
    specific range.

    The FSEC sector (index 383) is never erased: the FSEC byte going
    through 0xFF for even a moment risks locking the chip on a
    mid-operation reset. Only 1->0 bit transitions are programmed
    directly. If the backup requires 0->1 transitions, the sector
    fails with a clear error and must be fixed manually."""
    MAX_ATTEMPTS = 6
    data = open(srcpath, "rb").read()
    if len(data) < PFLASH_SIZE:
        raise ValueError(f"{srcpath} is {len(data)} bytes, expected ≥ {PFLASH_SIZE}")

    nsectors = PFLASH_SIZE // PFLASH_SECTOR
    if end_sector is None:
        end_sector = nsectors - 1
    start_sector = max(0, min(start_sector, nsectors - 1))
    end_sector   = max(start_sector, min(end_sector, nsectors - 1))

    bdm.enter()
    log(f"Programming P-Flash from {srcpath}")
    log(f"  range: sectors {start_sector}..{end_sector} of {nsectors} "
        f"(global 0x{sector_to_global(start_sector):X} → "
        f"0x{sector_to_global(end_sector)+PFLASH_SECTOR-1:X}), FSEC skipped")
    t0 = time.time()
    fail_count = 0
    total = end_sector - start_sector + 1
    for k, i in enumerate(range(start_sector, end_sector + 1)):
        sect = sector_to_global(i)
        backup = data[i * PFLASH_SECTOR : (i + 1) * PFLASH_SECTOR]
        ok = False
        last_msg = "no attempt"
        is_fsec = (sect >= FSEC_SECTOR_BASE)
        for attempt in range(MAX_ATTEMPTS):
            if is_fsec:
                # FSEC sector: never erase. The FSEC byte (0x7FFF0F) must
                # stay at 0xFE. Read the chip first to confirm every needed
                # bit transition is 1->0 (safe with PROGRAM_P_FLASH alone).
                # This read is a pre-write safety gate, not a post-write
                # verify.
                fsec_byte = bdm.read_byte(0xFF0F)
                if fsec_byte != 0xFE:
                    last_msg = f"FSEC byte = 0x{fsec_byte:02X} (must be 0xFE)"
                    break
                if backup[0x30F] != 0xFE:
                    last_msg = f"backup FSEC byte = 0x{backup[0x30F]:02X}, must be 0xFE"
                    break
                try:
                    chip = bdm.read_pflash(sect, PFLASH_SECTOR)
                except IOError as e:
                    last_msg = f"read-back failed: {e}"; continue
                need_erase = any((backup[j] & ~chip[j]) != 0 for j in range(PFLASH_SECTOR))
                if need_erase:
                    last_msg = ("FSEC sector has bits clear in chip that backup "
                                "wants set - would require erase (dangerous). "
                                "Manual recovery needed.")
                    break
                if chip == backup:
                    ok = True; break
                msg = bdm.write_pflash_chunk(sect, backup)
                if msg.startswith("OK"):
                    ok = True; break
                last_msg = msg
            else:
                if not bdm.erase_p_sector(sect):
                    last_msg = "erase failed"; continue
                # The FTM checks FSTAT after every phrase, so a successful
                # wpflash response means every phrase programmed cleanly.
                # Run verify-pflash separately for a byte-for-byte check.
                msg = bdm.write_pflash_chunk(sect, backup)
                if msg.startswith("OK"):
                    ok = True; break
                last_msg = msg
        if not ok:
            fail_count += 1
            log(f"  sector {i} (0x{sect:X}): FAILED after {MAX_ATTEMPTS} attempts - {last_msg}")
        if k % 32 == 0:
            elapsed = time.time() - t0
            rate = (k + 1) / max(1e-3, elapsed)
            eta = (total - (k + 1)) / max(1e-3, rate)
            log(f"  sector {i} (0x{sect:X}) [{k+1}/{total}] rate={rate*60:.0f}/min ETA={eta:.0f}s"
                f"{' [FSEC]' if is_fsec else ''}")
    log(f"P-Flash programming done in {time.time()-t0:.0f}s (failures: {fail_count})")
    return fail_count == 0


def program_dflash(bdm, srcpath, log=print):
    """Erase + bulk-write D-Flash. The FTM checks FSTAT after every phrase
    write; if `wdflash` returns OK, every phrase succeeded. Run
    verify-dflash afterwards if you want a full byte-for-byte cross-check."""
    data = open(srcpath, "rb").read()
    if len(data) != DFLASH_SIZE:
        raise ValueError(f"{srcpath} is {len(data)} bytes, expected {DFLASH_SIZE}")
    bdm.enter()
    log(f"Erasing full D-Flash")
    log("  " + bdm.erase_dflash_all())
    log(f"Programming D-Flash bulk write")
    t0 = time.time()
    msg = bdm.write_dflash_chunk(DFLASH_BASE, data, log=log)
    log(f"  {msg}  ({time.time()-t0:.0f}s)")
    return msg.startswith("OK")


def verify_region(bdm, base, size, reader, refpath, label, fix_callback=None, log=print):
    """Compare chip-side memory to a reference file. Optionally call
    fix_callback(sector_base, sector_data) on each mismatching aligned region."""
    ref = open(refpath, "rb").read()
    if len(ref) < size:
        raise ValueError(f"{refpath} is {len(ref)} bytes, expected ≥ {size}")
    bdm.enter()
    log(f"Reading {label} ({size//1024} KB) for verify")
    chip = dump_region(bdm, base, size, reader, label, log)
    diffs = sum(1 for a, b in zip(chip, ref[:size]) if a != b)
    log(f"{label}: {size - diffs}/{size} bytes match ({diffs} diff)")
    return diffs, chip


def program_fsec_sector(bdm, srcpath, log=print):
    """Program the last 1 KB of P-Flash (FSEC sector at 0x7FFC00..0x7FFFFF)
    from the backup file. This sector holds the reset vector, IVT,
    backdoor key, and the FPROT / EEPROT / FOPT / FSEC NV bytes.

    The sector is not erased: only 1->0 bit transitions are programmed,
    which keeps the FSEC byte at its current value (must already be
    0xFE; this is checked before any write). If the backup requires any
    0->1 transition, the function aborts before touching the chip.

    Useful when the FSEC sector's IVT has been wiped and the chip can
    no longer boot because the reset vector reads as 0xFFFF."""
    data = open(srcpath, "rb").read()
    if len(data) < PFLASH_SIZE:
        raise ValueError(f"{srcpath} is {len(data)} bytes, expected ≥ {PFLASH_SIZE}")
    fsec_file_off = global_to_src_off(FSEC_SECTOR_BASE)
    target = data[fsec_file_off : fsec_file_off + PFLASH_SECTOR]
    if len(target) != PFLASH_SECTOR:
        raise ValueError("file too short for FSEC sector")

    bdm.enter()

    # Confirm the chip's current FSEC byte is 0xFE (unsecured) before
    # touching anything in this sector.
    fsec = bdm.read_byte(0xFF0F)
    if fsec != 0xFE:
        raise IOError(f"chip FSEC = 0x{fsec:02X}, refusing to touch the "
                      f"sector. Recover via UNSECURE_FLASH first.")
    if target[0x30F] != 0xFE:
        raise ValueError(f"backup FSEC byte = 0x{target[0x30F]:02X}, must be 0xFE")
    log(f"  FSEC byte: chip=0x{fsec:02X} backup=0x{target[0x30F]:02X} ✓ unsecured")

    # Read current sector and classify diffs
    log("Reading current FSEC sector for safety check")
    chip = bdm.read_pflash(FSEC_SECTOR_BASE, PFLASH_SECTOR)
    program_count = 0
    erase_count   = 0
    for i in range(PFLASH_SECTOR):
        if chip[i] != target[i]:
            if (target[i] & ~chip[i]) == 0:
                program_count += 1
            else:
                erase_count += 1
    log(f"  diffs: {program_count} programmable, {erase_count} need-erase")
    if erase_count > 0:
        log("ABORT: some bytes have bits clear in chip that target wants set."
            " Would require erasing the sector, which is dangerous.")
        # Show a few problem bytes
        for i in range(PFLASH_SECTOR):
            if chip[i] != target[i] and (target[i] & ~chip[i]) != 0:
                log(f"    @+0x{i:03X}: chip=0x{chip[i]:02X} target=0x{target[i]:02X}")
                if i > 0 and i > 20: break
        return False
    if program_count == 0:
        log("FSEC sector already matches backup - nothing to do.")
        return True

    # Now program. wpflash will issue PROGRAM_P_FLASH per 8-byte phrase.
    # Bytes already at target value (e.g. FSEC byte itself) are no-ops.
    log(f"Programming FSEC sector via wpflash ({program_count} bytes will change)")
    t0 = time.time()
    msg = bdm.write_pflash_chunk(FSEC_SECTOR_BASE, target, log=log)
    log(f"  {msg}  ({time.time()-t0:.1f}s)")
    if not msg.startswith("OK"):
        return False

    # Confirm FSEC byte is still 0xFE. If it changed during programming,
    # the chip will secure itself on the next reset and the session is
    # the last chance to write to it.
    fsec2 = bdm.read_byte(0xFF0F)
    log(f"  FSEC byte after program: 0x{fsec2:02X} (must be 0xFE)")
    if fsec2 != 0xFE:
        log("ALERT: FSEC byte changed during program - chip will secure on next reset!")
        return False
    return True


def verify_pflash(bdm, refpath, log=print):
    """Compare chip P-Flash to reference, skipping the FSEC sector.
    Reads both physical blocks (split memory map) and concatenates to
    match the linear 384 KB reference file."""
    ref = open(refpath, "rb").read()
    bdm.enter()
    log(f"Reading P-Flash for verify against {refpath}")
    block1 = dump_region(bdm, PFLASH_BLOCK1_BASE, PFLASH_BLOCK1_SIZE,
                         bdm.read_pflash, "P-Flash block 1", log)
    block2 = dump_region(bdm, PFLASH_BLOCK2_BASE, PFLASH_BLOCK2_SIZE,
                         bdm.read_pflash, "P-Flash block 2", log)
    chip = block1 + block2
    # FSEC sector lives at the end of Block 2 - its file offset is
    # therefore PFLASH_SIZE - PFLASH_SECTOR (last sector of the file).
    fsec_off = PFLASH_SIZE - PFLASH_SECTOR
    diffs = sum(
        1 for i, (a, b) in enumerate(zip(chip, ref[:PFLASH_SIZE]))
        if a != b and i < fsec_off
    )
    log(f"P-Flash: {diffs} byte diff in non-FSEC region "
        f"(FSEC sector @ 0x{FSEC_SECTOR_BASE:X} skipped by design)")
    return diffs


def verify_dflash(bdm, refpath, log=print):
    diffs, _ = verify_region(
        bdm, DFLASH_BASE, DFLASH_SIZE, bdm.read_dflash, refpath, "D-Flash", log=log
    )
    return diffs


# EEPROM workflow - what 99% of FRM3 recoveries actually want.
def dump_eeprom(bdm, outpath, log=print):
    """Read the 4 KB EEPROM directly from the EEE buffer RAM. The EEE
    engine handles loading from D-Flash log on chip startup, so the buffer
    holds the current EEPROM state. If EEE isn't set up, this returns
    whatever junk is in buffer RAM - check eeequery first."""
    bdm.enter()
    log("eeequery: " + bdm.cmd("eeequery", timeout=10))
    log("Reading 4 KB EEE buffer RAM @ 0x13_F000")
    ee = bdm._bulk_read("reee", 0, EE_SIZE)
    with open(outpath, "wb") as f: f.write(ee)
    log(f"Saved {len(ee)} bytes to {outpath}")
    info = eeprom_info(ee)
    log(f"  VIN:       {info['vin']}")
    log(f"  HW nr:     {info['hw_nr']}")
    log(f"  SW nr:     {info['sw_nr']}")
    log(f"  ZB nr:     {info['zb_nr']}")
    log(f"  Prod date: {info['prod_date']}")
    log(f"  Prog date: {info['prog_date']}")
    return ee


def program_eeprom(bdm, srcpath, log=print):
    """Write a 4 KB EEPROM image to the chip via the hardware EEE engine.

    Sequence:
        1. fullpartition 0 16  destructive; erases D-Flash and sets the
                               EEE partition (0 user sectors, 16 EEE
                               buffer-RAM sectors = full 4 KB backed by
                               the entire 32 KB D-Flash log).
        2. enableeee           start the EEE engine.
        3. weee                stream the 4 KB image into EEE buffer
                               RAM; the engine logs each write to
                               D-Flash transparently.

    A successful `weee` response means every word was accepted by the
    engine. Run verify-eeprom for a byte-for-byte check."""
    ee = open(srcpath, "rb").read()
    if len(ee) != EE_SIZE:
        raise ValueError(f"{srcpath} is {len(ee)} bytes, expected {EE_SIZE}")
    bdm.enter()

    log("EEE workflow - DESTRUCTIVE (erases all D-Flash)")
    log("Step 1/3: full-partition D-Flash (DFPART=0 ERPART=16)")
    r = bdm.cmd("fullpartition 0 16", timeout=60)
    log(f"  {r}")
    if not r.startswith("OK"):
        return False

    log("Step 2/3: enable EEE engine")
    r = bdm.cmd("enableeee", timeout=10)
    log(f"  {r}")
    if not r.startswith("OK"):
        return False

    log(f"Step 3/3: stream {EE_SIZE} bytes into EEE buffer RAM @ 0x13_F000")
    t0 = time.time()
    msg = bdm._stream_program("weee", 0, ee, log=log)
    log(f"  {msg}  ({time.time()-t0:.1f}s)")
    if not msg.startswith("OK"):
        return False

    log("EEE state: " + bdm.cmd("eeequery", timeout=10))
    return True


def verify_eeprom(bdm, refpath, log=print):
    """Read EEE buffer RAM, compare to 4 KB reference."""
    ref = open(refpath, "rb").read()
    if len(ref) != EE_SIZE:
        raise ValueError(f"{refpath} is {len(ref)} bytes, expected {EE_SIZE}")
    bdm.enter()
    log("eeequery: " + bdm.cmd("eeequery", timeout=10))
    chip = bdm._bulk_read("reee", 0, EE_SIZE)
    diffs = sum(1 for a, b in zip(chip, ref) if a != b)
    log(f"EEE buffer vs {refpath}: {diffs}/{EE_SIZE} byte diff")
    return diffs


# CLI
def main(argv=None):
    ap = argparse.ArgumentParser(description="frm3 v2 - MC9S12XEQ384 BDM host")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("probe")
    sub.add_parser("enter")
    sub.add_parser("status")
    sub.add_parser("sync")
    sp = sub.add_parser("rb");   sp.add_argument("addr")
    sp = sub.add_parser("dump-eeprom");    sp.add_argument("outfile")
    sp = sub.add_parser("program-eeprom"); sp.add_argument("infile")
    sp = sub.add_parser("verify-eeprom");  sp.add_argument("reffile")
    sp = sub.add_parser("info");           sp.add_argument("eepromfile")
    sp = sub.add_parser("dump-pflash");    sp.add_argument("outfile")
    sp = sub.add_parser("dump-dflash");    sp.add_argument("outfile")
    sp = sub.add_parser("program-pflash"); sp.add_argument("infile")
    sp.add_argument("--start-sector", type=int, default=0, help="resume from this sector index (0..383)")
    sp.add_argument("--end-sector", type=int, default=None, help="stop after this sector index")
    sp = sub.add_parser("program-fsec-sector"); sp.add_argument("infile",
        help="P-Flash backup. Programs FSEC sector (IVT, backdoor, FPROT) without erase.")
    sp = sub.add_parser("program-dflash"); sp.add_argument("infile")
    sp = sub.add_parser("verify-pflash");  sp.add_argument("reffile")
    sp = sub.add_parser("verify-dflash");  sp.add_argument("reffile")
    sp = sub.add_parser("restore")
    sp.add_argument("pflash"); sp.add_argument("dflash")

    args = ap.parse_args(argv)

    # Offline command: needs no chip
    if args.cmd == "info":
        ee = open(args.eepromfile, "rb").read()
        if len(ee) != EE_SIZE:
            print(f"ERR {args.eepromfile} is {len(ee)} bytes, expected {EE_SIZE}")
            return 1
        info = eeprom_info(ee)
        for k, v in info.items():
            print(f"  {k:11s}: {v}")
        return 0

    bdm = BDM(args.port, args.baud)

    if args.cmd in ("probe", "status", "sync", "enter"):
        print(bdm.cmd(args.cmd, timeout=15))
        return 0
    if args.cmd == "rb":
        bdm.enter()
        print(bdm.cmd(f"rb {int(args.addr, 16):X}"))
        return 0
    if args.cmd == "dump-eeprom":
        dump_eeprom(bdm, args.outfile); return 0
    if args.cmd == "program-eeprom":
        return 0 if program_eeprom(bdm, args.infile) else 1
    if args.cmd == "verify-eeprom":
        return 0 if verify_eeprom(bdm, args.reffile) == 0 else 2
    if args.cmd == "dump-pflash":
        dump_pflash(bdm, args.outfile); return 0
    if args.cmd == "dump-dflash":
        dump_dflash(bdm, args.outfile); return 0
    if args.cmd == "program-pflash":
        return 0 if program_pflash(bdm, args.infile,
                                   start_sector=args.start_sector,
                                   end_sector=args.end_sector) else 1
    if args.cmd == "program-fsec-sector":
        return 0 if program_fsec_sector(bdm, args.infile) else 1
    if args.cmd == "program-dflash":
        return 0 if program_dflash(bdm, args.infile) else 1
    if args.cmd == "verify-pflash":
        return 0 if verify_pflash(bdm, args.reffile) == 0 else 2
    if args.cmd == "verify-dflash":
        return 0 if verify_dflash(bdm, args.reffile) == 0 else 2
    if args.cmd == "restore":
        if not program_pflash(bdm, args.pflash): return 1
        if verify_pflash(bdm, args.pflash) != 0:  return 2
        if not program_dflash(bdm, args.dflash): return 3
        if verify_dflash(bdm, args.dflash) != 0:  return 4
        print("\n*** RESTORE COMPLETE ***")
        return 0


if __name__ == "__main__":
    sys.exit(main())
