"""Persistent ucSim engine for the ROB3 firmware — a thin ROB3 layer on pyucsim.

Subclasses :class:`pyucsim.UCSimEngine` (the generic, CPU-agnostic ucSim-over-pty
driver) and adds everything ROB3-specific:

  * sensible defaults — finds a ``ucsim_51``/``s51`` binary and the ROB3 ROM,
    forces ``-t 51``, optionally ``loadhw``s the ROB3 cl_hw modules;
  * firmware landmarks (per-axis target/position/feedback IRAM, 8255 motor
    ports, the main-loop entry);
  * ROB3 control/readback helpers — ``reset(fixed_baud=…)``, ``run_cycles``,
    ``run_to``, ``press``/``release``, ``push_pot``/``push_pots``,
    ``read_positions``/``read_targets``/``read_ports``, ``set_iram``.

The generic transport (pty lifecycle, ``command``/``step``/``run``/``load_hw``,
the ``@``-filename and ``run``-vs-``step`` gotchas, ``close``, context manager)
is inherited from pyucsim.

If a ``ucsim_51`` with the cl_hw modules is not available the engine still runs
on stock ``s51`` minus ``set hardware`` (``has_modules`` is then ``False`` and
the teachbox/adc helpers are no-ops).
"""
from __future__ import annotations

import os
import re
import shutil

from pyucsim import UCSimEngine as _BaseEngine

# Verified firmware landmarks (see the rob3-firmware-map skill / annotated asm)
IRAM_TARGET = 0x40        # 0x40+N per-axis target
IRAM_CURPOS = 0x50        # 0x50+N per-axis current position
IRAM_FEEDBK = 0x58        # 0x58+N per-axis feedback (ADC copy)
XRAM_PORT_A = 0x5000      # 8255 Port A (motor bits, axes 0..3)
XRAM_PORT_C = 0x5200      # 8255 Port C (motor bits, axes 4..5)
MAIN_LOOP = 0x074D


def find_ucsim() -> str:
    """Return a ucSim binary path. Resolution order:

    1. ``UCSIM_51`` env (the override), else
    2. the standard sibling checkout
       ``$HOME/github/eurobtec/ucsim/src/sims/s51.src/ucsim_51`` (the
       plugin-enabled build — preferred over a stock PATH binary that cannot
       ``loadhw`` the cl_hw modules), else
    3. ``ucsim_51`` on PATH, else ``s51``.
    """
    cand = os.environ.get("UCSIM_51")
    sibling = os.path.expanduser(
        "~/github/eurobtec/ucsim/src/sims/s51.src/ucsim_51")
    paths = [cand] if cand else []
    paths += [sibling, shutil.which("ucsim_51")]
    for p in paths:
        if p and os.path.exists(os.path.expanduser(p)) and os.access(os.path.expanduser(p), os.X_OK):
            return os.path.expanduser(p)
    s51 = shutil.which("s51")
    if s51:
        return s51
    raise FileNotFoundError("no ucsim_51 or s51 on PATH (set UCSIM_51)")


def default_hex() -> str | None:
    """The ROB3 ROM path. Resolution order:

    1. the ``ROB3_HEX`` env var (any image — the override), else
    2. the standard sibling checkout
       ``$HOME/github/eurobtec/rob3/firmware/legacy/hex/M2764A@DIP28.HEX``
       (so a normal checkout needs neither ``ROB3_HEX`` nor ``--hex``), else
    3. ``None`` (pass ``hex_path``/``--hex``).

    The ``@`` in the filename is handled by pyucsim's ``safe_image``."""
    env = os.environ.get("ROB3_HEX")
    if env:
        return os.path.expanduser(env)
    cand = os.path.expanduser(
        "~/github/eurobtec/rob3/firmware/legacy/hex/M2764A@DIP28.HEX")
    return cand if os.path.exists(cand) else None


class UCSimEngine(_BaseEngine):
    """ROB3-aware ucSim engine (subclass of :class:`pyucsim.UCSimEngine`)."""

    def __init__(
        self,
        xtal: str = "11.0592M",
        hex_path: str | None = None,
        console_port: int | None = None,
        load_hw: list[str] | None = None,
        binary: str | None = None,
    ):
        self.console_port = console_port
        extra_args: list[str] = []
        if console_port:
            # -z: command console on localhost:<port> AND stdio (so the CLI can
            # boot over its pty while a terminal attaches with `nc`); -b: no
            # ANSI colour so the nc stream is clean.
            extra_args += ["-b", "-z", str(console_port)]

        image = hex_path if hex_path is not None else default_hex()

        super().__init__(
            binary=binary or find_ucsim(),
            cpu="51",
            xtal=xtal,
            image=image,
            load_hw=load_hw,
            extra_args=extra_args,
        )

        # ROB3 cl_hw present? (set hardware adc recognised, e.g. via a loaded
        # adc.so or a compiled-in module).
        probe = self.command("set hardware adc")
        self.has_modules = "adc[" in probe or "adc " in probe.lower()

    # -- `cmd` alias (kept for the GUI/CLI/tests that used the old API) -------
    def cmd(self, line: str, timeout: float = 5.0) -> str:
        return self.command(line, timeout=timeout)

    # -- firmware control -----------------------------------------------------
    def reset(self, fixed_baud: bool = True) -> None:  # type: ignore[override]
        self.command("reset")
        if fixed_baud:
            self.command("set mem sfr 0xb0 0x00")   # P3.0=0 -> fixed-baud path

    def run_cycles(self, n: int) -> None:
        """Advance a bounded number of instructions (ucSim ``step N``).

        `step` (not `run N`) is the reliable bounded-advance primitive on this
        build — `run N` free-runs until interrupted. Inherited from pyucsim."""
        self.step(n, timeout=20.0)

    def run_to(self, addr: int, cycles: int = 3_000_000) -> bool:
        """Free-run to a breakpoint at ``addr``; return True if it was hit."""
        self.command("break 0x%04x" % addr)
        out = self.run(timeout=30.0)
        self.command("clear 0x%04x" % addr)
        return ("stop at 0x%06x" % addr) in out.lower() \
            or ("stop at 0x%04x" % addr) in out.lower()

    # -- teachbox / adc (need the cl_hw modules) ------------------------------
    def press(self, row: int, group: int) -> None:
        if self.has_modules:
            self.command(f"set hardware teachbox {row} {group}")

    def release(self) -> None:
        if self.has_modules:
            self.command("set hardware teachbox 0")

    def push_pot(self, ch: int, value: int) -> None:
        if self.has_modules:
            self.command("set hardware adc %d 0x%02x" % (ch, value & 0xFF))

    def push_pots(self, values) -> None:
        """Push several ADC channels. Safe to issue back-to-back (no run/step
        between them, so none is swallowed by ucSim's resUSER pipelining trap)."""
        if not self.has_modules:
            return
        for ch, v in enumerate(values):
            self.command("set hardware adc %d 0x%02x" % (ch, v & 0xFF))

    # -- state readback -------------------------------------------------------
    def _dump_byte(self, space: str, addr: int) -> int:
        out = self.command("dump %s 0x%04x 0x%04x" % (space, addr, addr))
        m = re.search(r"0x0*%x\b[^\n]*?\s([0-9a-fA-F]{2})\b" % addr, out, re.I)
        return int(m.group(1), 16) if m else -1

    def read_positions(self) -> list[int]:
        return self._row6(self.command("dump iram 0x50 0x55"), 0x50)

    def read_targets(self) -> list[int]:
        return self._row6(self.command("dump iram 0x40 0x45"), 0x40)

    def read_ports(self) -> tuple[int, int]:
        return self._dump_byte("xram", XRAM_PORT_A), self._dump_byte("xram", XRAM_PORT_C)

    @staticmethod
    def _row6(out: str, addr: int) -> list[int]:
        m = re.search(r"0x%02x\b([^\n]*)" % (addr & 0xff), out, re.I)
        if not m:
            return [-1] * 6
        hexes = re.findall(r"\b([0-9a-fA-F]{2})\b", m.group(1))
        vals = [int(h, 16) for h in hexes[:6]]
        return (vals + [-1] * 6)[:6]

    def set_iram(self, addr: int, value: int) -> None:
        self.command("set mem iram 0x%02x 0x%02x" % (addr, value & 0xFF))


# ---- smoke test --------------------------------------------------------------
if __name__ == "__main__":
    eng = UCSimEngine()
    print(f"binary={eng.binary}  modules={eng.has_modules}")
    eng.reset()
    print("reached main loop:", eng.run_to(MAIN_LOOP))
    print("targets  :", ["0x%02x" % v for v in eng.read_targets()])
    print("positions:", ["0x%02x" % v for v in eng.read_positions()])
    print("ports A/C:", ["0x%02x" % v for v in eng.read_ports()])
    eng.close()
