"""Keyboard-teachbox — drive the ROB3 firmware's keypad from your keyboard.

An INPUT driver (separate from the viewers): it injects teachbox keypad events
into the running firmware so the arm moves; watch the motion in a viewer
(``rob3-viz``). Two modes:

* **own-engine** — boots its own ucSim and drives it directly (``eng.press``).
* **attach** (``--console-port N``) — connects to a *running* ucSim's command
  console (opened by ``rob3-viz --console-port N``) and injects keypresses via
  ``set hardware teachbox <row> <group>`` over that socket, so one sim is shared
  between the viewer and this driver.

Keys (verified ROB3 keypad row/group, from hardware/teachbox/test.md):

    0-9   numeric (POSITION mode: 2..7 select axis 0..5)
    + -   jog selected axis up / down
    P     POS    E ENT    N NOP    I INS    O OUT    D DEL    C ERR    R RUN
    q     quit

Raw single-key mode (default on a TTY): press a key and it fires immediately —
good for live jogging next to a 3D viewer.
"""

from __future__ import annotations

import socket
import sys
import time

#: char -> (label, row, group). group 1/2/3 = P1.5/6/7; row = 74LS138 /Y0../Y7.
#: Verified key->(row,group) (mirrors the GUI/CLI KEYMAP).
KEYMAP = {
    "0": ("0", 0, 2), "1": ("1", 1, 2), "2": ("2", 2, 2), "3": ("3", 3, 2),
    "4": ("4", 4, 2), "5": ("5", 5, 2), "6": ("6", 6, 2), "7": ("7", 7, 2),
    "8": ("8", 0, 1), "9": ("9", 1, 1),
    "+": ("UP/RIGHT", 3, 1), "-": ("DOWN/LEFT", 4, 1),
    "P": ("POS", 2, 3), "E": ("ENT", 5, 1), "N": ("NOP", 2, 1),
    "D": ("DEL", 7, 1), "C": ("ERR", 6, 1), "R": ("RUN", 7, 3),
    "I": ("INS", 0, 3), "O": ("OUT", 1, 3),
}

PRESS_CYCLES = 8000   # interactive ucSim budget per keypress


class _OwnEngineSink:
    """Drive a ucSim engine this process owns, using the VERIFIED teachbox
    bring-up: load loopback+teachbox+adc, reach the main-loop keypad poll, then
    dispatch keys via the release→hold→release debounce cadence (the same
    sequence the vetted test_teachbox_axis_select.py uses).

    Axis-select (keys 0..5) is [SIM]-verified end-to-end through the scanner.
    Jog (+/-) uses the firmware's verified kh_jog routine on the selected axis
    (the key→jog scanner path is not yet black-box-mapped, so we invoke the
    verified jog directly — same net effect: the selected axis's 0x50+N moves).
    """

    JOG = 0x0E26          # kh_jog entry
    KBD_HANDLE = 0x0C80   # key dispatch
    POS_SLOT = 0x50       # per-axis position base

    def __init__(self, hex_path=None, console_port=None):
        import os
        from rob3_ucsim import UCSimEngine, default_hex, find_ucsim, MAIN_LOOP
        rom = hex_path or default_hex()
        if not rom:
            raise SystemExit("set ROB3_HEX=/path/to/rom.hex (or pass --hex)")
        base = os.environ.get("ROB3_MODS")
        if not base:
            here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            for c in (os.path.join(here, "..", "..", "simulator", "ucsim-modules"),
                      os.path.expanduser("~/github/eurobtec/rob3_ucsim/simulator/ucsim-modules")):
                if os.path.isdir(c):
                    base = c
                    break
        mods = []
        if base:
            for name in ("loopback", "teachbox", "adc"):
                p = os.path.join(base, name, f"{name}.so")
                if os.path.exists(p):
                    mods.append(p)
        self.eng = UCSimEngine(hex_path=rom, binary=find_ucsim(),
                               console_port=console_port, load_hw=mods or None)
        self.eng.reset(fixed_baud=False)
        self.eng.command("set mem sfr 0xb0 0x00")   # de-assert emergency-off (P3.2)
        self._main = MAIN_LOOP
        self._live = self.eng.has_modules
        if not self._live:
            print("WARNING: teachbox/adc cl_hw not loaded — keypresses will not "
                  "reach the firmware. Build them / set ROB3_MODS.", file=sys.stderr)
        else:
            self.eng.run_to(MAIN_LOOP)
        self.selected = None

    def _settle(self, n):
        for _ in range(n):
            self.eng.run_cycles(8000)

    def _tap(self, row, group, hold=6):
        """Release→press→(step to kbd_handle)→hold→release — the debounce cadence."""
        self.eng.release(); self._settle(3)
        self.eng.press(row, group)
        self.eng.command("break 0x%04x" % self.KBD_HANDLE)
        for _ in range(30):
            out = self.eng.command("step 8000", timeout=15)
            if ("0x%04x" % self.KBD_HANDLE) in out.lower() or \
               ("0x%06x" % self.KBD_HANDLE) in out.lower():
                break
        self.eng.command("clear 0x%04x" % self.KBD_HANDLE)
        self._settle(hold)
        self.eng.release(); self._settle(2)

    def press(self, row, group):
        if not self._live or group not in (1, 2, 3):
            return
        # index = row + 1 + (group-1)*8; axis-select is index 0x02..0x07
        # (group 1, rows 1..6) -> axis 0..5.
        if group == 1 and 1 <= row <= 6:
            self.selected = row - 1
        self._tap(row, group)

    def jog(self, direction: int) -> None:
        """Jog the selected axis via the verified kh_jog routine (ACC.0: 0=+,1=-)."""
        if not self._live or self.selected is None:
            return
        self.eng.command("set mem sfr 0xd0 0x00")                 # PSW bank 0
        self.eng.command("set mem sfr 0xe0 0x%02x" % (direction & 1))
        self.eng.command("set mem iram 0x29 0x40")                # POSITION mode
        self.eng.command("set mem iram 0x01 0x%02x" % (self.POS_SLOT + self.selected))  # R1
        self.eng.command("pc 0x%04x" % self.JOG)
        self.eng.command("break 0x0e40"); self.eng.command("break 0x0e2f")
        self.eng.command("break 0x0e36"); self.eng.run(timeout=10)
        self.eng.command("clear 0x0e40"); self.eng.command("clear 0x0e2f")
        self.eng.command("clear 0x0e36")

    def close(self):
        self.eng.close()


class _ConsoleSink:
    """Inject keypresses into a *running* ucSim via its command-console socket."""

    def __init__(self, host: str, port: int):
        self.sock = socket.create_connection((host, port), timeout=5.0)

    def _send(self, line: str):
        self.sock.sendall((line + "\n").encode())
        time.sleep(0.02)

    def press(self, row, group):
        if group in (1, 2, 3):
            self._send(f"set hardware teachbox {row} {group}")
            self._send("set hardware teachbox 0")   # release

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


def _read_key():
    """Read one keypress raw (no Enter). Returns '' on EOF."""
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def run(sink, *, raw: bool = True) -> None:
    print("keyboard-teachbox: keys 0-9 + - P E N I O D C R, 'q' to quit")
    while True:
        if raw and sys.stdin.isatty():
            ch = _read_key()
        else:
            line = sys.stdin.readline()
            if not line:
                break
            ch = line.strip()[:1]
        if not ch or ch in ("q", "\x03", "\x04"):   # q / Ctrl-C / Ctrl-D
            break
        k = ch.upper() if ch.upper() in KEYMAP else ch
        if k not in KEYMAP:
            continue
        label, row, group = KEYMAP[k]
        if k in ("+", "-") and hasattr(sink, "jog"):
            sink.jog(0 if k == "+" else 1)      # ACC.0: 0=+ (increment), 1=- (decrement)
            print(f"  jog {label}")
        else:
            sink.press(row, group)
            print(f"  {label} (row={row} group={group})")


def main() -> int:
    import argparse

    ap = argparse.ArgumentParser(description="Keyboard-teachbox: drive the ROB3 firmware keypad")
    ap.add_argument("--console-port", type=int, default=None,
                    help="attach to a running ucSim console on localhost:PORT "
                         "(opened by `rob3-viz --console-port PORT`)")
    ap.add_argument("--host", default="localhost", help="console host (attach mode)")
    ap.add_argument("--hex", default=None, help="ROM image for own-engine mode (default $ROB3_HEX)")
    ap.add_argument("--line", action="store_true", help="line mode (type keys + Enter) instead of raw")
    args = ap.parse_args()

    if args.console_port:
        sink = _ConsoleSink(args.host, args.console_port)
    else:
        sink = _OwnEngineSink(hex_path=args.hex)
    try:
        run(sink, raw=not args.line)
    except KeyboardInterrupt:
        pass
    finally:
        sink.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
