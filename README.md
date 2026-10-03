# rob3_ucsim

ROB3-specific **ucSim simulation harness** (pure Python, built on
[`pyucsim`](https://github.com/eurobtec/pyucsim)). Drives the ROB3 8031 firmware
in Daniel Drotos' µCsim (`ucsim_51` / `s51`) and exposes the firmware's state
through a small, ROB3-aware API — it knows this firmware's memory landmarks
(per-axis targets/positions/feedback, the 8255 motor ports, the main-loop entry)
and its `cl_hw` peripherals (teachbox, adc).

`UCSimEngine` is a **subclass of `pyucsim.UCSimEngine`**: the generic
ucSim-over-pty transport (process lifecycle, `command`/`step`/`run`/`load_hw`,
the `@`-filename and `run`-vs-`step` gotchas, `close`, context manager) is
inherited; this package adds only the ROB3 layer.

Meant to be used **alongside** the [`rob3`](https://github.com/eurobtec/rob3_py)
RS-232 protocol library: bring the ROM up in ucSim with `UCSimEngine`, then
drive/inspect it with `rob3`'s codec, calibration, and `Rob3Client`.

## Install

```bash
pip install -e .            # pulls pyucsim from git
pip install -e ".[dev]"     # + pytest
```

Requires Python ≥ 3.10 (via `pyucsim`), a `ucsim_51` / `s51` binary on `PATH`
(or `UCSIM_51`), and — for the ROM — the ROB3 firmware image (point `ROB3_HEX`
at `firmware/hex/M2764A@DIP28.HEX` in the firmware repo, or pass `hex_path=`).
The interactive teachbox/adc helpers also need the ROB3 `cl_hw` `.so` modules
(pass them via `load_hw=[...]`).

## API

| Symbol | What it is |
| :----- | :--------- |
| `UCSimEngine` | ROB3-aware ucSim engine (subclass of `pyucsim.UCSimEngine`): `reset(fixed_baud=)`, `run_to(addr)`, `run_cycles(n)`, `press(row,group)`/`release()`, `push_pot(ch,v)`/`push_pots()`, `read_positions()`/`read_targets()`/`read_ports()`, `set_iram()`, plus everything inherited from pyucsim (`command`, `step`, `run`, `load_hw`, `close`, `with` support). |
| `UCSimBatch` | One-shot batch driver: run a script of ucSim commands, parse the output. |
| `Plant` | The motor/pot physics that live *outside* ucSim (no ucSim dependency): decode the 8255 Port A/C motor bits and integrate pot positions. Unit-testable; reusable behind a socket bridge to ROS 2 / Gazebo. |
| `MAIN_LOOP`, `IRAM_TARGET`, `IRAM_CURPOS`, `IRAM_FEEDBK`, `XRAM_PORT_A`, `XRAM_PORT_C` | Verified firmware landmarks. |
| `find_ucsim()`, `default_hex()` | Locate the simulator binary / the ROM (`ROB3_HEX`). |

## Usage

### Drive the firmware in ucSim

```python
from rob3_ucsim import UCSimEngine, MAIN_LOOP

eng = UCSimEngine(hex_path="/path/to/rob3/firmware/hex/M2764A@DIP28.HEX")
eng.reset()
eng.run_to(MAIN_LOOP)               # run past the init gates to the main loop
print("positions:", eng.read_positions())
eng.close()
```

With the cl_hw modules (teachbox / adc), pass them to `load_hw`:

```python
eng = UCSimEngine(
    hex_path="…/M2764A@DIP28.HEX",
    load_hw=["…/ucsim-modules/adc/adc.so", "…/ucsim-modules/teachbox/teachbox.so"],
)
```

### Together with the `rob3` protocol library

```python
from rob3_ucsim import UCSimEngine, MAIN_LOOP
from rob3 import protocol as P, Calibration

eng = UCSimEngine(hex_path="…/M2764A@DIP28.HEX")
eng.reset(); eng.run_to(MAIN_LOOP)
counts = [c & 0xFF for c in eng.read_positions()]
joints = Calibration().counts_to_joints(counts)   # counts -> rad/m
frame  = P.set_all_positions(counts)               # the RS-232 frame
eng.close()
```

### Environment variables

| Var | Meaning | Default |
| :-- | :------ | :------ |
| `UCSIM_51` | path to a loader-enabled `ucsim_51` | PATH lookup, then stock `s51` |
| `ROB3_HEX` | ROB3 ROM image | none (pass `hex_path=` or set this) |

## Relationship to the other repos

- **[pyucsim](https://github.com/eurobtec/pyucsim):** the generic, CPU-agnostic
  ucSim Python client this package subclasses.
- **[rob3](https://github.com/eurobtec/rob3):** the firmware repo — the ROM,
  annotated source, `cl_hw` modules, and the behavioral `tests/*.sh`.
- **[rob3_py](https://github.com/eurobtec/rob3_py):** the RS-232 protocol
  library — use it with this harness.

## License

MIT.
