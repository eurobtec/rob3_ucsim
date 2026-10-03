"""rob3_ucsim — ROB3-specific ucSim simulation harness (built on pyucsim).

Drives the ROB3 8031 firmware in Daniel Drotos' µCsim (``ucsim_51`` / ``s51``)
and exposes the firmware's state through a small, ROB3-aware API. The engine is
a **subclass of :class:`pyucsim.UCSimEngine`** — it inherits the generic
ucSim-over-pty transport and adds ROB3 knowledge: this firmware's memory
landmarks (per-axis targets/positions/feedback, the 8255 motor ports, the
main-loop entry) and its ``cl_hw`` peripherals (teachbox, adc).

Designed to be used *alongside* :mod:`rob3` (the RS-232 protocol library): bring
the ROM up in ucSim with :class:`UCSimEngine`, then drive/inspect the controller
with ``rob3``'s codec, calibration, and ``Rob3Client``.

Components:
  * :class:`UCSimEngine` — ROB3-aware ucSim engine (subclass of
    ``pyucsim.UCSimEngine``): ``reset``/``run_to``/``run_cycles``,
    ``press``/``push_pot``, ``read_positions``/``read_targets``/``read_ports``.
  * :class:`UCSimBatch`  — one-shot batch driver (run a script, parse output).
  * :class:`Plant`       — the motor/pot physics that live outside ucSim
    (no ucSim dependency; unit-testable, reusable behind a socket bridge).

The ROM image and the ``cl_hw`` ``.so`` modules live in the ROB3 firmware repo;
point ``ROB3_HEX`` / ``UCSIM_51`` at them (or pass ``hex_path`` / ``load_hw``).

Typical use::

    from rob3_ucsim import UCSimEngine, MAIN_LOOP

    eng = UCSimEngine(hex_path="…/M2764A@DIP28.HEX")   # @ handled by pyucsim
    eng.reset()
    eng.run_to(MAIN_LOOP)
    print(eng.read_positions())
    eng.close()
"""
from __future__ import annotations

from .batch import UCSimBatch
from .engine import (
    IRAM_CURPOS,
    IRAM_FEEDBK,
    IRAM_TARGET,
    MAIN_LOOP,
    XRAM_PORT_A,
    XRAM_PORT_C,
    UCSimEngine,
    default_hex,
    find_ucsim,
)
from .plant import N_AXES, MotorCommand, Plant

__all__ = [
    "UCSimEngine",
    "UCSimBatch",
    "Plant",
    "MotorCommand",
    "N_AXES",
    "find_ucsim",
    "default_hex",
    "MAIN_LOOP",
    "IRAM_TARGET",
    "IRAM_CURPOS",
    "IRAM_FEEDBK",
    "XRAM_PORT_A",
    "XRAM_PORT_C",
]

__version__ = "0.1.0"
