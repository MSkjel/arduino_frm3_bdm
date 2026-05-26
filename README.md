# arduino_frm3_bdm

An Arduino-based BDM tool for the MC9S12XEQ384 microcontroller used in
BMW E70/E71/E9x FRM3 footwell modules. Reads, dumps, and reprograms
P-Flash, D-Flash, and the 4 KB emulated EEPROM through the chip's
hardware EEE engine.

## Liability

Use at your own risk. Reprogramming an FRM3 is destructive and can
permanently brick the module if the wrong file, wrong wiring, or wrong
power source is used. The author accepts no responsibility for damage
to the module, the vehicle, its electrical system, or any downstream
loss of use. There is no warranty of any kind, express or implied.

For the EEPROM fix workflow, the tool automatically saves a copy of
the chip's D-Flash before it touches anything; that copy is the only
backup you need to roll back the operation.

For full-firmware operations (program-pflash, program-dflash, restore),
make your own backups of P-Flash and D-Flash before writing. The tool
will not do this for you.

This tool reads and writes manufacturer firmware. You are responsible
for confirming that you have the right to do so on the specific
hardware in question.

## Requirements

Hardware

* Arduino Uno R4 WiFi (or Minima; pin mapping differs and is handled
  in the firmware automatically).
* Two 1 kΩ series resistors.
* Bench 5 V or 12 V supply for the FRM3. Do not power the FRM3 from
  the Arduino's USB rail.
* Three wires from Arduino to FRM3 BDM test points:
  * D2 to BKGD (through 1 kΩ)
  * D3 to RESET (through 1 kΩ)
  * GND to GND

Software

* `arduino-cli` with the `arduino:renesas_uno` core installed.
* Python 3.10 or later with `pyserial`.
* `tkinter` (usually bundled with Python).

## How to use

Build and upload the firmware:

```
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi arduino_frm3_bdm.ino
arduino-cli upload  --fqbn arduino:renesas_uno:unor4wifi -p /dev/ttyACM0 arduino_frm3_bdm.ino
```

Connect the FRM3 power, BKGD, RESET, and GND. Then either:

GUI workflow

```
python3 frm3_gui.py
```

* **Quick (EEPROM)** tab: one-click "Run EEPROM fix" reads D-Flash,
  decodes it to a 4 KB EEPROM, then reprograms via the EEE engine.
  Both backup files are saved with a timestamp in a folder of your
  choice.
* **Advanced (raw flash)** tab: dump, program, and verify P-Flash and
  D-Flash directly.
* **Diagnostics** tab: probe, sync, flash registers, EEE query, chip
  identity, boot test.

CLI workflow

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

`restore` runs program + verify for both P-Flash and D-Flash in one
shot.

## Accreditation

Chip-side details come from the NXP MC9S12XE-Family Reference Manual
(Rev. 1.21), in particular Chapter 7 (BDM) and Chapter 26
(S12XFTM384K2V1 flash module).

The EEE-log to EEPROM decoder in `frm3.py` is a inspired by the algorithm published by Tom van Leeuwen at
https://gitlab.com/tomvleeuwen/dflash_to_eeprom.

The wider FRM3 recovery workflow was assembled from publicly available
write-ups, repair notes, and community discussion across various
automotive ECU and BMW repair forums.

## License

This project is licensed under the MIT License. See the `LICENSE` file
for the full text.

In short: you may use, modify, and redistribute this software for any
purpose, including commercially, provided the copyright notice and
permission notice are preserved. The software is provided as-is, with
no warranty.
