#!/usr/bin/env python3
"""Tkinter GUI for the arduino_frm3_bdm tool.

Wraps the BDM driver and EEPROM workflow from frm3.py in a single
window. No extra dependencies beyond Python's stdlib and pyserial.

Layout:
  Top    : connection (port / baud / Connect button / status light)
  Middle : tabs for Quick (EEPROM), Advanced (raw flash), Diagnostics
  Bottom : scrolling log and progress bar

Long BDM operations run in a worker thread so the UI stays responsive.
"""
import os, sys, threading, queue, time, traceback
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext

import frm3
from frm3 import (
    BDM, EE_SIZE, DFLASH_SIZE, PFLASH_SIZE, DFLASH_BASE,
    eeprom_info, dflash_to_eeprom,
    dump_eeprom, program_eeprom, verify_eeprom,
    dump_dflash, program_dflash, verify_dflash,
    dump_pflash, program_pflash, verify_pflash,
)


# Cross-thread log channel
class LogChannel:
    """Drop-in `print`-like logger that funnels into a Tk text widget.
    Safe to call from a worker thread - it just queues lines."""
    def __init__(self):
        self.q = queue.Queue()

    def __call__(self, *args, **kw):
        line = " ".join(str(a) for a in args)
        self.q.put(line)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("FRM3 BDM")
        self.geometry("900x720")
        self.minsize(720, 560)

        self.bdm = None
        self.worker = None
        self.log_channel = LogChannel()
        # Tracks the last interactive BDM command timestamp. The FRM3
        # board has an external WDT that resets the chip a few seconds
        # after firmware stops running, and the CPU is halted while in
        # active BDM. If more than `_bdm_dwell_s` has elapsed between
        # interactive commands, the chip has likely dropped out of BDM
        # and `_raw_cmd` triggers an automatic re-enter.
        self._last_cmd_at = 0.0
        self._bdm_dwell_s = 2.5

        self._build_ui()
        self.after(80, self._drain_log)
        self.after(800, self._tick_status)

    # ------------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------------
    def _build_ui(self):
        # ----- Top: connection bar ------------------------------------------
        top = ttk.Frame(self, padding=(10, 8))
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar(value="/dev/ttyACM0")
        ttk.Entry(top, textvariable=self.port_var, width=22)\
            .grid(row=0, column=1, padx=(4, 16))

        ttk.Label(top, text="Baud:").grid(row=0, column=2, sticky="w")
        self.baud_var = tk.StringVar(value="1000000")
        ttk.Entry(top, textvariable=self.baud_var, width=10)\
            .grid(row=0, column=3, padx=(4, 16))

        self.connect_btn = ttk.Button(top, text="Connect", command=self._on_connect)
        self.connect_btn.grid(row=0, column=4, padx=4)

        self.status_canvas = tk.Canvas(top, width=14, height=14, highlightthickness=0)
        self.status_dot = self.status_canvas.create_oval(2, 2, 12, 12, fill="#cc4444", outline="")
        self.status_canvas.grid(row=0, column=5, padx=(20, 4))
        self.status_label = ttk.Label(top, text="Disconnected")
        self.status_label.grid(row=0, column=6, sticky="w")

        top.columnconfigure(7, weight=1)

        # ----- Middle: notebook ---------------------------------------------
        nb = ttk.Notebook(self, padding=(8, 4))
        nb.pack(side=tk.TOP, fill=tk.BOTH, expand=False)
        self._build_quick_tab(nb)
        self._build_advanced_tab(nb)
        self._build_diag_tab(nb)

        # ----- Bottom: log + progress ---------------------------------------
        bot = ttk.Frame(self, padding=(10, 4))
        bot.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        ttk.Label(bot, text="Log:").pack(anchor="w")
        self.log = scrolledtext.ScrolledText(bot, height=14, font=("monospace", 9))
        self.log.pack(fill=tk.BOTH, expand=True, pady=(2, 6))
        self.log.configure(state="disabled")

        prog_frame = ttk.Frame(bot)
        prog_frame.pack(fill=tk.X)
        self.progress = ttk.Progressbar(prog_frame, mode="indeterminate", length=200)
        self.progress.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.busy_label = ttk.Label(prog_frame, text="Idle")
        self.busy_label.pack(side=tk.RIGHT, padx=(8, 0))

    def _build_quick_tab(self, nb):
        frame = ttk.Frame(nb, padding=12)
        nb.add(frame, text="Quick (EEPROM)")

        ttk.Label(frame, text="Most FRM3 recoveries only need this tab.",
                  foreground="#555").pack(anchor="w", pady=(0, 8))

        ee_box = ttk.LabelFrame(frame, text="EEPROM (4 KB, the EEPROM the firmware exposes)",
                                padding=10)
        ee_box.pack(fill=tk.X, pady=(0, 8))

        ttk.Button(ee_box, text="Dump EEPROM…",
                   command=lambda: self._pick_save("Dump EEPROM",
                                                  [("EEPROM 4 KB", "*.bin")],
                                                  self._run_dump_eeprom))\
            .grid(row=0, column=0, padx=4, pady=4, sticky="ew")

        ttk.Button(ee_box, text="Program EEPROM…",
                   command=lambda: self._pick_open("Program EEPROM",
                                                  [("EEPROM 4 KB", "*.bin")],
                                                  self._run_program_eeprom))\
            .grid(row=0, column=1, padx=4, pady=4, sticky="ew")

        ttk.Button(ee_box, text="Verify EEPROM…",
                   command=lambda: self._pick_open("Verify EEPROM against file",
                                                  [("EEPROM 4 KB", "*.bin")],
                                                  self._run_verify_eeprom))\
            .grid(row=0, column=2, padx=4, pady=4, sticky="ew")

        ttk.Button(ee_box, text="Read EEPROM info from file…",
                   command=lambda: self._pick_open("Read EEPROM info (offline)",
                                                  [("EEPROM 4 KB", "*.bin")],
                                                  self._run_offline_info))\
            .grid(row=0, column=3, padx=4, pady=4, sticky="ew")

        for c in range(4):
            ee_box.columnconfigure(c, weight=1)

        # ---- one-click EEPROM recovery -------------------------------------
        # The standard FRM3 repair workflow per the FRM Repair Guide:
        #   1. Read the chip's raw 32 KB D-Flash (contains the EEE log even
        #      when the EEE buffer RAM reads as garbage / 0xFF)
        #   2. Decode the log → 4 KB EEPROM image
        #   3. Erase chip + re-program EEPROM via the EEE engine
        # Reading the EEE buffer RAM directly via `reee` doesn't work for
        # bricked modules where EEE was never enabled - D-Flash is the
        # source of truth.
        fix_box = ttk.LabelFrame(frame,
            text="Fix EEPROM (read D-Flash → decode → re-program via EEE)",
            padding=10)
        fix_box.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(fix_box,
            text="One-click recovery: reads the chip's 32 KB D-Flash log, "
                 "decodes it to a 4 KB EEPROM image (saved as a backup), then "
                 "erases the chip and re-writes the EEPROM via the EEE engine.",
            foreground="#555", wraplength=720)\
            .pack(anchor="w", pady=(0, 6))
        ttk.Button(fix_box, text="Run EEPROM fix…",
                   command=self._run_fix_eeprom)\
            .pack(anchor="w")

        info_box = ttk.LabelFrame(frame, text="Last EEPROM metadata", padding=8)
        info_box.pack(fill=tk.X, pady=(8, 0))
        self.info_text = tk.Text(info_box, height=8, font=("monospace", 10),
                                  background="#f8f8f8")
        self.info_text.pack(fill=tk.X)
        self.info_text.configure(state="disabled")

    def _build_advanced_tab(self, nb):
        frame = ttk.Frame(nb, padding=12)
        nb.add(frame, text="Advanced (raw flash)")

        ttk.Label(frame,
                  text="Raw P-Flash (384 KB) and raw D-Flash (32 KB) - for "
                       "firmware recovery, not EEPROM repair.",
                  foreground="#555", wraplength=720)\
            .pack(anchor="w", pady=(0, 8))

        p_box = ttk.LabelFrame(frame, text="P-Flash (384 KB firmware)", padding=10)
        p_box.pack(fill=tk.X, pady=(0, 8))
        ttk.Button(p_box, text="Dump P-Flash…",
                   command=lambda: self._pick_save("Dump P-Flash",
                                                  [("P-Flash 384 KB", "*.bin")],
                                                  self._run_dump_pflash))\
            .grid(row=0, column=0, padx=4, pady=4, sticky="ew")
        ttk.Button(p_box, text="Program P-Flash…",
                   command=lambda: self._pick_open("Program P-Flash",
                                                  [("P-Flash 384 KB", "*.bin")],
                                                  self._run_program_pflash))\
            .grid(row=0, column=1, padx=4, pady=4, sticky="ew")
        ttk.Button(p_box, text="Verify P-Flash…",
                   command=lambda: self._pick_open("Verify P-Flash",
                                                  [("P-Flash 384 KB", "*.bin")],
                                                  self._run_verify_pflash))\
            .grid(row=0, column=2, padx=4, pady=4, sticky="ew")
        for c in range(3): p_box.columnconfigure(c, weight=1)

        # Program-range control row - lets you resume from sector N after a
        # mid-run failure instead of redoing all 384 sectors. Empty / 0 / blank
        # means "from the start"; empty `end` means "all the way to the end".
        rng_row = ttk.Frame(p_box)
        rng_row.grid(row=1, column=0, columnspan=3, pady=(8, 0), sticky="ew")
        ttk.Label(rng_row, text="Program range - start sector:")\
            .pack(side=tk.LEFT)
        self.pflash_start = tk.StringVar(value="0")
        ttk.Entry(rng_row, textvariable=self.pflash_start, width=6)\
            .pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(rng_row, text="end sector (blank = 383):")\
            .pack(side=tk.LEFT)
        self.pflash_end = tk.StringVar(value="")
        ttk.Entry(rng_row, textvariable=self.pflash_end, width=6)\
            .pack(side=tk.LEFT, padx=(4, 0))
        ttk.Label(rng_row, text="  (valid 0..383 - sector 383 = FSEC, always skipped)",
                  foreground="#777").pack(side=tk.LEFT)

        d_box = ttk.LabelFrame(frame, text="D-Flash (32 KB raw - not the decoded EEPROM)",
                               padding=10)
        d_box.pack(fill=tk.X, pady=(0, 8))
        ttk.Button(d_box, text="Dump D-Flash…",
                   command=lambda: self._pick_save("Dump D-Flash",
                                                  [("D-Flash 32 KB", "*.bin")],
                                                  self._run_dump_dflash))\
            .grid(row=0, column=0, padx=4, pady=4, sticky="ew")
        ttk.Button(d_box, text="Program D-Flash…",
                   command=lambda: self._pick_open("Program D-Flash (raw)",
                                                  [("D-Flash 32 KB", "*.bin")],
                                                  self._run_program_dflash))\
            .grid(row=0, column=1, padx=4, pady=4, sticky="ew")
        ttk.Button(d_box, text="Verify D-Flash…",
                   command=lambda: self._pick_open("Verify D-Flash",
                                                  [("D-Flash 32 KB", "*.bin")],
                                                  self._run_verify_dflash))\
            .grid(row=0, column=2, padx=4, pady=4, sticky="ew")
        for c in range(3): d_box.columnconfigure(c, weight=1)

    def _build_diag_tab(self, nb):
        frame = ttk.Frame(nb, padding=12)
        nb.add(frame, text="Diagnostics")

        # --- BDM connection / line state --------------------------------
        bdm_box = ttk.LabelFrame(frame, text="BDM connection", padding=8)
        bdm_box.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(bdm_box, text="Probe (BKGD / RESET line state)",
                   command=lambda: self._raw_cmd("probe")).pack(anchor="w", pady=2)
        ttk.Button(bdm_box, text="Re-enter active BDM",
                   command=lambda: self._raw_cmd("enter", timeout=15)).pack(anchor="w", pady=2)
        ttk.Button(bdm_box, text="Re-measure bus period (sync)",
                   command=lambda: self._raw_cmd("sync")).pack(anchor="w", pady=2)

        # --- Flash module status / EEE engine ---------------------------
        flash_box = ttk.LabelFrame(frame, text="Flash module + EEE engine", padding=8)
        flash_box.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(flash_box,
                   text="Read flash registers (FSEC / FCLKDIV / FSTAT / FPROT / DFPROT)",
                   command=lambda: self._raw_cmd("status")).pack(anchor="w", pady=2)
        ttk.Button(flash_box,
                   text="EEE query (DFPART / ERPART / ECOUNT / DEAD / READY)",
                   command=lambda: self._raw_cmd("eeequery", timeout=10)).pack(anchor="w", pady=2)
        ttk.Button(flash_box,
                   text="Chip identity (PARTID / MEMSIZ / mask info)",
                   command=self._diag_chip_identity).pack(anchor="w", pady=2)

        # --- CPU debug / boot test --------------------------------------
        cpu_box = ttk.LabelFrame(frame, text="CPU debug", padding=8)
        cpu_box.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(cpu_box,
                   text="Boot test: reset chip, run firmware, halt + show CPU state",
                   command=self._diag_boot_test).pack(anchor="w", pady=2)
        ttk.Button(cpu_box,
                   text="Read CPU registers (PC / D / X / Y / SP)",
                   command=lambda: self._raw_cmd("rcpu")).pack(anchor="w", pady=2)
        ttk.Button(cpu_box,
                   text="Single-step (TRACE1)",
                   command=lambda: self._raw_cmd("step")).pack(anchor="w", pady=2)

        # --- Raw byte read row ------------------------------------------
        rb_box = ttk.LabelFrame(frame, text="Raw read", padding=8)
        rb_box.pack(fill=tk.X, pady=(0, 6))
        row = ttk.Frame(rb_box); row.pack(anchor="w")
        ttk.Label(row, text="Read byte at addr (hex):").pack(side=tk.LEFT)
        self.rb_addr = tk.StringVar(value="0101")
        ttk.Entry(row, textvariable=self.rb_addr, width=10).pack(side=tk.LEFT, padx=4)
        ttk.Button(row, text="rb", command=self._on_rb).pack(side=tk.LEFT, padx=4)
        ttk.Label(row, text="  PPAGE (rb 0015) is what bulk reads cache.",
                  foreground="#777").pack(side=tk.LEFT)

    # ------------------------------------------------------------------------
    # Helpers - file dialogs, command dispatch, threading
    # ------------------------------------------------------------------------
    def _pick_save(self, title, types, then):
        path = filedialog.asksaveasfilename(title=title, filetypes=types,
                                            defaultextension=".bin")
        if path: then(path)

    def _pick_open(self, title, types, then):
        path = filedialog.askopenfilename(title=title, filetypes=types)
        if path: then(path)

    def _set_busy(self, busy, label=""):
        if busy:
            self.progress.configure(mode="indeterminate")
            self.progress.start(80)
            self.busy_label.configure(text=label or "Working…")
        else:
            self.progress.stop()
            self.progress.configure(mode="determinate", value=0)
            self.busy_label.configure(text="Idle")

    def _runner(self, label, fn):
        if self.worker and self.worker.is_alive():
            messagebox.showwarning("Busy", "A BDM operation is already running.")
            return
        if self.bdm is None:
            messagebox.showwarning("Not connected", "Connect to the Arduino first.")
            return
        self._set_busy(True, label)
        self.log_channel(f"\n=== {label} ===")
        def go():
            try:
                fn()
            except Exception as e:
                self.log_channel(f"ERROR: {e}")
                self.log_channel(traceback.format_exc())
            finally:
                self.after(0, lambda: self._set_busy(False))
        self.worker = threading.Thread(target=go, daemon=True)
        self.worker.start()

    @staticmethod
    def _looks_like_bdm_dropout(resp: str) -> bool:
        # When the external WDT has reset the chip mid-BDM, reads return
        # 0xFF and ACK gets auto-disabled. The signature in a `status`
        # reply is `ack=off` plus uniform 0xFF register values.
        if "ack=off" in resp and "FSEC=0xFF" in resp and "FSTAT=0xFF" in resp:
            return True
        # Single-byte reads after dropout return `rb 0xXXXX = 0xFF`, which
        # can be a legitimate value, so only the multi-register pattern
        # is flagged here.
        return False

    def _raw_cmd(self, cmd, timeout=10):
        if self.bdm is None:
            messagebox.showwarning("Not connected", "Connect to the Arduino first.")
            return
        try:
            now = time.time()
            # Proactive re-enter if the session has been idle long enough
            # that the external WDT has almost certainly reset the chip.
            if cmd != "enter" and self._last_cmd_at \
                    and (now - self._last_cmd_at) > self._bdm_dwell_s:
                self.log_channel(f"(idle {now - self._last_cmd_at:.1f}s, re-entering BDM)")
                r = self.bdm.cmd("enter", timeout=15)
                self.log_channel(r)
            self.log_channel(f"> {cmd}")
            resp = self.bdm.cmd(cmd, timeout=timeout)
            self.log_channel(resp)
            # Reactive recovery: if the response looks like the chip
            # dropped out anyway, re-enter and retry once.
            if cmd != "enter" and self._looks_like_bdm_dropout(resp):
                self.log_channel("(BDM dropout detected - re-entering and retrying)")
                r = self.bdm.cmd("enter", timeout=15)
                self.log_channel(r)
                self.log_channel(f"> {cmd}")
                resp = self.bdm.cmd(cmd, timeout=timeout)
                self.log_channel(resp)
            self._last_cmd_at = time.time()
        except Exception as e:
            self.log_channel(f"ERROR: {e}")

    def _on_rb(self):
        try:
            addr = int(self.rb_addr.get(), 16)
        except ValueError:
            messagebox.showerror("Bad input", "Address must be hex (e.g. 101)")
            return
        self._raw_cmd(f"rb {addr:X}")

    def _diag_chip_identity(self):
        """Read PARTID, MEMSIZ, COPCTL - useful for confirming the chip is
        what the tools expect and that COP is disabled in BDM."""
        if self.bdm is None:
            messagebox.showwarning("Not connected", "Connect to the Arduino first.")
            return
        def _byte(addr):
            r = self.bdm.cmd(f"rb {addr:X}")
            try: return int(r.split("=")[-1].strip(), 16)
            except Exception: return None
        try:
            hi   = _byte(0x001A); lo = _byte(0x001B)
            ms0  = _byte(0x001C); ms1 = _byte(0x001D)
            cop  = _byte(0x003C)
            ppage = _byte(0x0015); epage = _byte(0x0017)
        except Exception as e:
            self.log_channel(f"ERROR reading identity registers: {e}")
            return
        if hi is None or lo is None:
            self.log_channel("ERROR: PARTID read failed")
            return
        partid = (hi << 8) | lo
        self.log_channel(f"  PARTID  (0x001A:1B) = 0x{partid:04X}")
        self.log_channel(f"  MEMSIZ0 (0x001C)    = 0x{ms0:02X}")
        self.log_channel(f"  MEMSIZ1 (0x001D)    = 0x{ms1:02X}")
        self.log_channel(f"  COPCTL  (0x003C)    = 0x{cop:02X}  "
                         f"(RSBCK={(cop>>6)&1} - 1 = COP frozen in BDM)")
        self.log_channel(f"  PPAGE   (0x0015)    = 0x{ppage:02X}")
        self.log_channel(f"  EPAGE   (0x0017)    = 0x{epage:02X}")
        # PARTID 0xC482 = MC9S12XE family mask 3M25J (per FRM Repair Guide).
        # Marketed as MC9S12XEQ384 but the silicon has 512 KB physical
        # accessible through the FTM - split memory map 0x780000-0x79FFFF
        # + 0x7C0000-0x7FFFFF for the "official" 384 KB.
        if partid == 0xC482:
            self.log_channel("  → matches MC9S12XEQ384 (mask 3M25J, used in BMW FRM3)")

    def _diag_boot_test(self):
        """Reset the chip into normal mode, let firmware run, then halt
        and dump CPU registers. Lets you confirm the firmware boots past
        the early init path. Reports PC / PPAGE / SP at the halt point.
        A PC in I/O space (<0x4000) with SP near 0xFFF0 indicates an
        SWI/trap (firmware crash). PC in 0x4000+ across multiple delays
        with varying registers indicates the firmware is running."""
        if self.bdm is None:
            messagebox.showwarning("Not connected", "Connect to the Arduino first.")
            return
        def go():
            self.log_channel("Boot test: reset chip, run firmware, halt at various delays")
            for ms in (1, 5, 50, 250, 1000):
                r = self.bdm.cmd(f"letrun {ms}", timeout=max(8, ms/1000 + 3))
                self.log_channel(f"  letrun {ms:4d}ms: {r}")
        self._runner("Boot test", go)

    # ------------------------------------------------------------------------
    # Connect / status
    # ------------------------------------------------------------------------
    def _on_connect(self):
        if self.bdm is not None:
            try: self.bdm.ser.close()
            except Exception: pass
            self.bdm = None
            self.connect_btn.configure(text="Connect")
            self._set_status(False, "Disconnected")
            return
        port = self.port_var.get().strip()
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Bad baud", "Baud must be an integer")
            return
        self.log_channel(f"Opening {port} at {baud} baud…")
        try:
            self.bdm = BDM(port, baud)
        except Exception as e:
            self.log_channel(f"ERROR: {e}")
            messagebox.showerror("Open failed", str(e))
            return
        self.connect_btn.configure(text="Disconnect")
        self._set_status(True, "Connected - try Enter BDM")
        # Auto-enter BDM
        try:
            res = self.bdm.cmd("enter", timeout=15)
            self.log_channel(res)
            if "in active BDM" in res:
                self._set_status(True, "Connected - active BDM")
            else:
                self._set_status(True, "Connected - enter failed")
        except Exception as e:
            self.log_channel(f"enter: {e}")

    def _set_status(self, connected, text):
        color = "#44aa44" if connected else "#cc4444"
        self.status_canvas.itemconfigure(self.status_dot, fill=color)
        self.status_label.configure(text=text)

    def _tick_status(self):
        # Periodic refresh of status text - placeholder for future use.
        self.after(800, self._tick_status)

    # ------------------------------------------------------------------------
    # Log drain - polled from UI thread
    # ------------------------------------------------------------------------
    def _drain_log(self):
        try:
            while True:
                line = self.log_channel.q.get_nowait()
                self.log.configure(state="normal")
                self.log.insert("end", line + "\n")
                self.log.see("end")
                self.log.configure(state="disabled")
        except queue.Empty:
            pass
        self.after(80, self._drain_log)

    def _set_info(self, info):
        text = (
            f"  Valid magic: {info['valid_magic']}\n"
            f"  VIN:         {info['vin']}\n"
            f"  Prod date:   {info['prod_date']}\n"
            f"  Prog date:   {info['prog_date']}\n"
            f"  HW nr:       {info['hw_nr']}\n"
            f"  SW nr:       {info['sw_nr']}\n"
            f"  ZB nr:       {info['zb_nr']}\n"
            f"  Sticker:     {info['sticker']}\n"
        )
        self.info_text.configure(state="normal")
        self.info_text.delete("1.0", "end")
        self.info_text.insert("1.0", text)
        self.info_text.configure(state="disabled")

    # ------------------------------------------------------------------------
    # Workflow handlers
    # ------------------------------------------------------------------------
    def _run_offline_info(self, path):
        try:
            ee = open(path, "rb").read()
            if len(ee) != EE_SIZE:
                messagebox.showerror("Bad file",
                    f"{path} is {len(ee)} bytes, expected {EE_SIZE}")
                return
            info = eeprom_info(ee)
            self._set_info(info)
            self.log_channel(f"Read metadata from {path}")
            for k, v in info.items():
                self.log_channel(f"  {k:11s}: {v}")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _run_dump_eeprom(self, path):
        def go():
            ee = dump_eeprom(self.bdm, path, log=self.log_channel)
            self.after(0, lambda: self._set_info(eeprom_info(ee)))
        self._runner(f"Dump EEPROM → {path}", go)

    def _run_program_eeprom(self, path):
        def go():
            ok = program_eeprom(self.bdm, path, log=self.log_channel)
            ee = open(path, "rb").read()
            self.after(0, lambda: self._set_info(eeprom_info(ee)))
            self.after(0, lambda: messagebox.showinfo(
                "Done", "EEPROM programmed and verified." if ok
                else "Programming completed with errors - see log."))
        self._runner(f"Program EEPROM from {path}", go)

    def _run_verify_eeprom(self, path):
        def go():
            diffs = verify_eeprom(self.bdm, path, log=self.log_channel)
            ref = open(path, "rb").read()
            self.after(0, lambda: self._set_info(eeprom_info(ref)))
            self.after(0, lambda: messagebox.showinfo(
                "Verify result",
                "EEPROM matches reference." if diffs == 0
                else f"{diffs} byte(s) differ - see log."))
        self._runner(f"Verify EEPROM against {path}", go)

    def _run_fix_eeprom(self):
        """Recover EEPROM from the chip's raw D-Flash log and re-program
        via EEE. Per the FRM Repair Guide:
            1. Read 32 KB D-Flash (full log, even if EEE is disabled)
            2. Decode → 4 KB EEPROM (frm3.dflash_to_eeprom)
            3. Save both bins as backups
            4. Re-program EEPROM through EEE engine (program_eeprom)"""
        if self.bdm is None:
            messagebox.showwarning("Not connected", "Connect to the Arduino first.")
            return
        # Pick a directory for the backup files. The decoded EEPROM is
        # also the input to the re-program step.
        out_dir = filedialog.askdirectory(
            title="Folder to save D-Flash + decoded EEPROM backups")
        if not out_dir:
            return
        ts = time.strftime("%Y%m%d_%H%M%S")
        dflash_path = os.path.join(out_dir, f"dflash_{ts}.bin")
        eeprom_path = os.path.join(out_dir, f"eeprom_{ts}.bin")

        if not messagebox.askokcancel(
                "Confirm EEPROM fix",
                "This will:\n"
                f"  1. Dump 32 KB D-Flash → {dflash_path}\n"
                f"  2. Decode to 4 KB EEPROM → {eeprom_path}\n"
                "  3. ERASE D-Flash + EEPROM region (destructive)\n"
                "  4. Re-program EEPROM via the EEE engine\n\n"
                "Proceed?"):
            return

        log = self.log_channel
        def go():
            try:
                # Step 1: dump raw D-Flash. The full 32 KB is read through
                # the BDM EPAGE window. The D-Flash log is the source of
                # truth, not the EEE buffer RAM, which can be cleared if
                # EEE was never enabled on a bricked module.
                log("Step 1/4: dumping 32 KB D-Flash log")
                dump_dflash(self.bdm, dflash_path, log=log)
                df = open(dflash_path, "rb").read()
                # Step 2: decode the EEE-log format
                log("Step 2/4: decoding D-Flash → 4 KB EEPROM")
                ee, meta = dflash_to_eeprom(df)
                with open(eeprom_path, "wb") as f: f.write(ee)
                log(f"  recovered: {meta.get('words_recovered')}/2048 words "
                    f"(ok={meta.get('ok')}, corrupt={meta.get('corrupt')})")
                info = eeprom_info(ee)
                self.after(0, lambda: self._set_info(info))
                log(f"  VIN={info['vin']}  HW={info['hw_nr']}  SW={info['sw_nr']}")
                # Step 3+4: re-program EEPROM via the EEE engine (program_eeprom
                # does fullpartition + enableeee + weee + readback verify).
                log("Step 3/4: erasing + re-partitioning chip")
                log("Step 4/4: writing decoded EEPROM via EEE engine")
                ok = program_eeprom(self.bdm, eeprom_path, log=log)
                msg = ("EEPROM fix complete." if ok
                       else "EEPROM fix finished with errors - see log.")
                self.after(0, lambda: messagebox.showinfo("Done", msg))
            except Exception as e:
                log(f"ERROR: {e}\n{traceback.format_exc()}")
                self.after(0, lambda: messagebox.showerror(
                    "EEPROM fix failed", str(e)))
        self._runner("EEPROM fix workflow", go)

    def _run_dump_dflash(self, path):
        self._runner(f"Dump D-Flash → {path}",
                     lambda: dump_dflash(self.bdm, path, log=self.log_channel))

    def _run_program_dflash(self, path):
        def go():
            ok = program_dflash(self.bdm, path, log=self.log_channel)
            self.after(0, lambda: messagebox.showinfo(
                "Done", "D-Flash programmed." if ok else "Programming failed - see log."))
        self._runner(f"Program D-Flash from {path}", go)

    def _run_verify_dflash(self, path):
        def go():
            diffs = verify_dflash(self.bdm, path, log=self.log_channel)
            self.after(0, lambda: messagebox.showinfo(
                "Verify result",
                "D-Flash matches reference." if diffs == 0
                else f"{diffs} byte(s) differ - see log."))
        self._runner(f"Verify D-Flash against {path}", go)

    def _run_dump_pflash(self, path):
        self._runner(f"Dump P-Flash → {path}",
                     lambda: dump_pflash(self.bdm, path, log=self.log_channel))

    def _run_program_pflash(self, path):
        def _to_int(s, default):
            s = s.strip()
            if not s: return default
            try:    return int(s)
            except ValueError:
                messagebox.showerror("Bad input",
                    f"Sector number {s!r} isn't an integer (0..383)")
                return None
        start = _to_int(self.pflash_start.get(), 0)
        end_text = self.pflash_end.get().strip()
        end = _to_int(self.pflash_end.get(), None)
        if start is None or (end_text and end is None):
            return
        def go():
            ok = program_pflash(self.bdm, path, log=self.log_channel,
                                start_sector=start, end_sector=end)
            self.after(0, lambda: messagebox.showinfo(
                "Done", "P-Flash programmed." if ok else "Programming had failures - see log."))
        rng = (f"sectors {start}..{end if end is not None else 383}"
               if start > 0 or end is not None else "all sectors")
        self._runner(f"Program P-Flash from {path} ({rng})", go)

    def _run_verify_pflash(self, path):
        def go():
            diffs = verify_pflash(self.bdm, path, log=self.log_channel)
            self.after(0, lambda: messagebox.showinfo(
                "Verify result",
                "P-Flash matches reference (FSEC sector skipped)." if diffs == 0
                else f"{diffs} byte(s) differ in non-FSEC region - see log."))
        self._runner(f"Verify P-Flash against {path}", go)


def main():
    App().mainloop()


if __name__ == "__main__":
    main()
