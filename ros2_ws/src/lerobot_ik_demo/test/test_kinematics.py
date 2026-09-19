"""ROS-free tests for scripts/so101_kin.py.  Run:  /usr/bin/python3 -m pytest test/ -q

Needs `xacro` + the lerobot_description package (sourced workspace) to read the URDF.
"""
import math
import os
import subprocess
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from so101_kin import Kin, densify, solve_path, time_parameterize  # noqa: E402

HOME = [0.015343553863686413, -0.015343553863686413, 0.23475637411440212, -0.01227484309094913, -0.006137421545474565]


@pytest.fixture(scope="module")
def kin():
    try:
        from ament_index_python.packages import get_package_share_directory
        xacro = os.path.join(get_package_share_directory("lerobot_description"), "urdf", "so101.urdf.xacro")
        urdf = subprocess.run(["xacro", xacro], capture_output=True, text=True, check=True).stdout
    except Exception as exc:  # workspace not sourced / xacro missing
        pytest.skip(f"cannot load URDF: {exc}")
    return Kin(urdf)


def test_fk_home_matches_moveit(kin):
    # value reported by MoveIt's /compute_fk for the SRDF "home" pose (gripper link origin)
    assert np.allclose(kin.position(HOME), [0.01679, -0.27230, 0.22522], atol=1e-4)


def test_ik_round_trip(kin):
    rng = np.random.default_rng(1)
    solved = 0
    for _ in range(100):
        q = [rng.uniform(-0.8, 0.8), rng.uniform(-1.2, 1.0), rng.uniform(-1.2, 1.2), rng.uniform(-1, 1), 0.0]
        sol = kin.ik(kin.position(q), Kin.pitch(q), HOME)
        if sol is None:
            continue
        solved += 1
        assert np.linalg.norm(kin.position(sol) - kin.position(q)) < 1e-3
        assert abs(Kin.pitch(sol) - Kin.pitch(q)) < 1e-9
        assert kin.in_limits(sol)
    assert solved > 70


def test_ik_rejects_unreachable_and_axis(kin):
    assert kin.ik((0.0, -1.0, 0.1), 0.0, HOME) is None               # far beyond reach
    assert kin.ik((kin.pan_xy[0], kin.pan_xy[1], 0.2), 0.0, HOME) is None  # on the pan axis


def test_pan_sign_positive_goes_right(kin):
    # +q1 swings the tool toward -x (the arm's right, seen from behind)
    a = kin.position([0.3, 0.0, 0.3, 0.0, 0.0])
    b = kin.position([-0.3, 0.0, 0.3, 0.0, 0.0])
    assert a[0] < b[0]


def test_densify_spacing():
    pts = densify([((0, 0, 0), 0.0), ((0.1, 0, 0), 0.5)], 0.01)
    assert len(pts) == 11
    assert pts[-1][1] == pytest.approx(0.5)
    assert all(np.linalg.norm(np.subtract(a[0], b[0])) <= 0.0101 for a, b in zip(pts, pts[1:]))


def test_solve_path_lift_is_continuous(kin):
    start = kin.position(HOME)
    dense = densify([(tuple(start), Kin.pitch(HOME)), ((start[0], start[1], start[2] + 0.05), Kin.pitch(HOME))], 0.01)
    qs, why = solve_path(kin, dense, HOME)
    assert qs is not None, why
    assert max(abs(a - b) for q1, q2 in zip(qs, qs[1:]) for a, b in zip(q1, q2)) < 0.3


def test_time_parameterize_profile():
    qs = [[0, 0, 0, 0, 0], [0.1, 0, 0, 0, 0], [0.3, 0, 0, 0, 0], [0.4, 0, 0, 0, 0]]
    prof = time_parameterize(qs, v_max=0.2)
    times = [t for t, _, _ in prof]
    assert times == sorted(times) and len(set(times)) == len(times)
    assert np.allclose(prof[0][2], 0) and np.allclose(prof[-1][2], 0)   # starts and ends at rest
    assert max(abs(qd[0]) for _, _, qd in prof) <= 0.2 + 1e-9            # never above v_max
    assert time_parameterize([[0.0] * 5, [0.0] * 5], 0.2) is None        # no motion -> nothing to run
