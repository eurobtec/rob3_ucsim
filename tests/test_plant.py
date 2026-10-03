"""Unit tests for the Plant motor/pot physics (no ucSim needed)."""
from rob3_ucsim import N_AXES, Plant


def test_axis0_drive_up_and_down():
    p = Plant(pot=[100] * N_AXES, step=4)
    p.step_from_ports(port_a=0b00000001, port_c=0)   # axis 0 up
    assert p.pot[0] == 104
    p.step_from_ports(port_a=0b00000010, port_c=0)   # axis 0 down
    assert p.pot[0] == 100


def test_axis5_in_port_c():
    p = Plant(pot=[100] * N_AXES, step=4)
    p.step_from_ports(port_a=0, port_c=0b00000100)   # axis 5 up (shift (5-4)*2)
    assert p.pot[5] == 104


def test_clamp_to_range():
    p = Plant(pot=[100] * N_AXES, step=4)
    p.pot[1] = 254
    p.step_from_ports(port_a=0b00000100, port_c=0)   # axis 1 up (shift 2)
    assert p.pot[1] == 255


def test_move_toward_fallback():
    p = Plant(pot=[10] * N_AXES, step=4)
    while p.move_toward(2, 20):
        pass
    assert p.pot[2] == 20


def test_hold_when_no_bits():
    p = Plant(pot=[128] * N_AXES, step=4)
    p.step_from_ports(port_a=0, port_c=0)
    assert p.pot == [128] * N_AXES
