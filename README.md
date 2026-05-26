# arduino_frm3_bdm

An Arduino-based BDM tool for the MC9S12XEQ384 microcontroller used in
BMW E70/E71/E9x FRM3 footwell modules. It can read, dump, and
reprogram P-Flash, D-Flash, and the 4 KB emulated EEPROM through the
chip's hardware EEE engine.

The common use case is repairing a "bricked" FRM3 by recovering its
EEPROM from the chip's D-Flash log and writing it back through the
correct EEE-engine path. There is a one-click button for this in the
GUI.

## Liability

Use at your own risk. Reprogramming an FRM3 is destructive and can
permanently brick the module if the wrong file, wrong wiring, or wrong
power source is used. The author accepts no responsibility for damage
to the module, the vehicle, its electrical system, or any downstream
loss of use. There is no warranty of any kind, express or implied.

The EEPROM-fix workflow automatically saves a copy of the chip's
D-Flash before it touches anything. That copy is the rollback for that
specific operation.

For full-firmware operations (program-pflash, program-dflash, restore),
make your own backups of P-Flash and D-Flash first. The tool will not
do this for you.

This tool reads and writes manufacturer firmware. You are responsible
for confirming that you have the right to do so on the specific
hardware in question.

## Requirements

### Hardware

* An Arduino Uno R4 WiFi (the Minima also works; the pin mapping
  difference is handled in the firmware automatically).
* Two 1 kΩ resistors.
* A bench 5 V supply with a current limit of 100 mA or more.
* Three jumper wires.
* Soldering iron or test clips to reach the chip's pads on the back
  of the FRM3 PCB. BKGD, RESET, GND, and VCC are all easy to find on
  the back side; they are not buried under components.

### Power

The MC9S12XEQ384 must be powered **directly on its VCC pin from a
bench 5 V supply**. Do not feed 12 V into the FRM3's vehicle
connector. Do not power the chip from the Arduino's 5 V output
either; the USB rail browns out under load and the chip will not
stay in BDM mode.

### Wiring

| Arduino pin | Chip pin | Notes |
| --- | --- | --- |
| D2 | BKGD | through a 1 kΩ resistor in series |
| D3 | RESET | through a 1 kΩ resistor in series |
| GND | GND | direct |
| (none) | VCC | from your bench 5 V supply |

All four pads are on the back of the FRM3 PCB.

### Software

* `arduino-cli` with the `arduino:renesas_uno` core installed.
* Python 3.10 or later, with `pyserial`.
* `tkinter` (usually bundled with the Python install).

## How to use

### 1. Build and upload the firmware

```
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi arduino_frm3_bdm.ino
arduino-cli upload  --fqbn arduino:renesas_uno:unor4wifi -p /dev/ttyACM0 arduino_frm3_bdm.ino
```

Replace `/dev/ttyACM0` with whatever port the Arduino enumerates on.
On Linux this is usually `/dev/ttyACM0` or `/dev/ttyACM1`; on Windows
it will be a `COMxx` port; on macOS, `/dev/cu.usbmodem...`.

### 2. Wire the FRM3

1. Locate BKGD, RESET, GND, and VCC on the back of the FRM3 PCB.
   They are easy to find and not buried under components. If your
   board generation labels them differently, board photos in the
   BMW repair community show the exact pads.
2. Solder the wires (or attach test clips). Add a 1 kΩ resistor in
   series with BKGD and RESET.
3. Connect Arduino GND, bench supply GND, and FRM3 GND together.
4. Connect bench 5 V to the chip's VCC pin. Do not connect anything
   to the FRM3's vehicle connector.
5. Plug the Arduino into your computer over USB.
6. Turn on the bench supply.

### 3. Run the GUI

```
python3 frm3_gui.py
```

In the GUI:

1. At the top, choose the right serial port and click **Connect**.
   The status light should turn green and the log should say
   "OK in active BDM, ...".
2. Pick a tab:
   - **Quick (EEPROM)** for the standard "fix my bricked FRM3"
     workflow. 
     **Before running the fix, dump the D-Flash a few times and
     diff them.** Use the **Dump D-Flash** button on the
     Advanced tab (or `python3 frm3.py dump-dflash`) to save
     two or three separate copies, then compare them with
     `cmp -l` or any binary diff tool. If the dumps don't all
     match exactly, your read path is not reliable yet (check
     wiring, power, ground, and re-seat the connections), and
     you should not run the fix. Running it on top of an
     inconsistent read will erase the chip's D-Flash and
     replace it with whichever variant you happened to capture
     during the destructive step.
     Click **Run EEPROM fix**, pick a folder for the
     backups, confirm the prompt. The tool reads the 32 KB
     D-Flash log from the chip, decodes it to a 4 KB EEPROM
     image, then reprograms the chip through its EEE engine.
     The two backup files (`dflash_<timestamp>.bin` and
     `eeprom_<timestamp>.bin`) end up in the folder you picked.
   - **Advanced (raw flash)** when you want to dump or program
     P-Flash or D-Flash directly. Useful for full-firmware
     recovery after a chip swap.
   - **Diagnostics** for sanity checks: probe the BDM lines,
     read flash registers, query the EEE engine, identify the
     chip, run a boot test.
3. Watch the log at the bottom of the window. Most operations log
   their progress and a clear OK or error at the end.

### 4. CLI workflow (alternative to the GUI)

If you would rather script things, every action the GUI does is
available through `frm3.py`:

```
python3 frm3.py dump-eeprom    out.bin
python3 frm3.py program-eeprom in.bin
python3 frm3.py verify-eeprom  ref.bin
python3 frm3.py info           ee.bin

python3 frm3.py dump-pflash    out.bin
python3 frm3.py program-pflash in.bin
python3 frm3.py program-pflash in.bin --start-sector 100 --end-sector 127
python3 frm3.py verify-pflash  ref.bin

python3 frm3.py dump-dflash    out.bin
python3 frm3.py program-dflash in.bin
python3 frm3.py verify-dflash  ref.bin

python3 frm3.py probe
python3 frm3.py status
python3 frm3.py rb <addr_hex>
```

The `restore` subcommand runs program + verify for both P-Flash and
D-Flash in one shot.

All subcommands accept `--port` and `--baud`; defaults are
`/dev/ttyACM0` and `1000000`.

## Accreditation

Chip-side details come from the NXP MC9S12XE-Family Reference Manual
(Rev. 1.21), in particular Chapter 7 (BDM) and Chapter 26
(S12XFTM384K2V1 flash module).

The EEE-log to EEPROM decoder in `frm3.py` is inspired by the
algorithm published by Tom van Leeuwen at
https://gitlab.com/tomvleeuwen/dflash_to_eeprom.

The wider FRM3 recovery workflow was assembled from publicly
available write-ups, repair notes, and community discussion across
various automotive ECU and BMW repair forums.

## License

This project is licensed under the MIT License. See the `LICENSE`
file for the full text.

In short: you may use, modify, and redistribute this software for any
purpose, including commercially, provided the copyright notice and
permission notice are preserved. The software is provided as-is,
with no warranty.
