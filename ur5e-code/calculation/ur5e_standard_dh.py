#!/usr/bin/env python3
"""UR5e nominal standard-DH forward kinematics, using only Python's stdlib.

Official nominal parameters (lengths in metres, angles in radians):
https://www.universal-robots.com/developer/hardware-and-motion/robot-motion-dh-parameters/

Convention: ^{i-1}T_i = Rz(q_i) Tz(d_i) Tx(a_i) Rx(alpha_i).
^{0}T_6 maps a point expressed in DH frame 6 to base DH frame 0.
There are no joint-angle offsets in this nominal table. Frame 6 is the
DH end frame; no separate tool/TCP transform is included. Factory
calibration and installation/world-frame transforms are not included.

Run:
    python ur5e_standard_dh.py
    python ur5e_standard_dh.py --q 30 -60 90 -30 45 60
    python ur5e_standard_dh.py --json path/to/calculations.json
The default run prints both q = 0 and the teaching example.
"""

import argparse
import json
import math
from pathlib import Path

SOURCE_URL = "https://www.universal-robots.com/developer/hardware-and-motion/robot-motion-dh-parameters/"
A = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
D = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
ALPHA = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]
EXAMPLE_DEG = [30.0, -60.0, 90.0, -30.0, 45.0, 60.0]


def eye():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def mm(left, right):
    return [[sum(left[i][k] * right[k][j] for k in range(len(right)))
             for j in range(len(right[0]))] for i in range(len(left))]


def standard_dh(q, a, d, alpha):
    c, s = math.cos(q), math.sin(q)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [[c, -s * ca, s * sa, a * c],
            [s, c * ca, -c * sa, a * s],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0]]


def elementary_dh(q, a, d, alpha):
    """Independent construction from four elementary homogeneous transforms."""
    c, s = math.cos(q), math.sin(q)
    ca, sa = math.cos(alpha), math.sin(alpha)
    rz = [[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0],
          [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    tz = eye()
    tz[2][3] = d
    tx = eye()
    tx[0][3] = a
    rx = [[1.0, 0.0, 0.0, 0.0], [0.0, ca, -sa, 0.0],
          [0.0, sa, ca, 0.0], [0.0, 0.0, 0.0, 1.0]]
    return mm(mm(mm(rz, tz), tx), rx)


def max_difference(left, right):
    return max(abs(x - y) for lr, rr in zip(left, right) for x, y in zip(lr, rr))


def determinant3(r):
    return (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
            - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
            + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))


def evaluate(q_degrees):
    if len(q_degrees) != 6:
        raise ValueError("Exactly six joint angles are required.")
    q = [math.radians(angle) for angle in q_degrees]
    local = [standard_dh(q[i], A[i], D[i], ALPHA[i]) for i in range(6)]
    independent = [elementary_dh(q[i], A[i], D[i], ALPHA[i]) for i in range(6)]
    cumulative = [eye()]
    independent_total = eye()
    for local_t, alternate_t in zip(local, independent):
        cumulative.append(mm(cumulative[-1], local_t))
        independent_total = mm(independent_total, alternate_t)
    total = cumulative[-1]
    r = [row[:3] for row in total[:3]]
    rt = list(map(list, zip(*r)))
    orthogonality_error = max_difference(mm(rt, r), [row[:3] for row in eye()[:3]])
    determinant = determinant3(r)
    local_error = max(max_difference(x, y) for x, y in zip(local, independent))
    total_error = max_difference(total, independent_total)
    assert local_error < 1e-12
    assert total_error < 1e-12
    assert orthogonality_error < 1e-12
    assert abs(determinant - 1.0) < 1e-12
    return {
        "q_degrees": list(q_degrees),
        "q_radians": q,
        "A_i": local,
        "T_0i": cumulative,
        "origins_0_to_6_m": [[t[row][3] for row in range(3)] for t in cumulative],
        "T_03": cumulative[3],
        "T_36": mm(mm(local[3], local[4]), local[5]),
        "T_06": total,
        "validation": {
            "max_elementary_local_error": local_error,
            "max_elementary_total_error": total_error,
            "max_RtR_minus_I_error": orthogonality_error,
            "det_R": determinant,
        },
    }


def print_matrix(matrix):
    for row in matrix:
        print("  [" + "  ".join(f"{(0.0 if abs(x) < 5e-10 else x): .9f}" for x in row) + "]")


def print_result(label, result):
    print(f"\n{label}: q in degrees = {result['q_degrees']}")
    for i, transform in enumerate(result["A_i"], start=1):
        print(f"A_{i} = ^{i-1}T_{i}:")
        print_matrix(transform)
    print("T_03 = A_1 A_2 A_3:")
    print_matrix(result["T_03"])
    print("T_36 = A_4 A_5 A_6:")
    print_matrix(result["T_36"])
    print("T_06 = A_1 A_2 A_3 A_4 A_5 A_6:")
    print_matrix(result["T_06"])
    print("Origins O_0 ... O_6 in base DH frame, metres:")
    for i, point in enumerate(result["origins_0_to_6_m"]):
        print(f"  O_{i}: ({point[0]: .9f}, {point[1]: .9f}, {point[2]: .9f})")
    print("Independent elementary-matrix and rotation checks:", result["validation"])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--q", nargs=6, type=float, metavar="DEG", help="Joint angles q1 ... q6 in degrees")
    parser.add_argument("--json", type=Path, help="Optional output path for full calculation data")
    args = parser.parse_args()
    results = {"custom": evaluate(args.q)} if args.q is not None else {
        "zero": evaluate([0.0] * 6), "example": evaluate(EXAMPLE_DEG)}
    print("UR5e nominal standard DH; all translations are in metres.")
    print("Source:", SOURCE_URL)
    for label, result in results.items():
        print_result(label, result)
    if args.json:
        payload = {
            "source_url": SOURCE_URL,
            "length_unit": "m",
            "angle_unit": "rad (unless explicitly labelled degrees)",
            "convention": "A_i = Rz(q_i) Tz(d_i) Tx(a_i) Rx(alpha_i)",
            "a_m": A, "d_m": D, "alpha_radians": ALPHA,
            "alpha_degrees": [90, 0, 0, 90, -90, 0],
            "joint_offsets_radians": [0.0] * 6,
            **results,
        }
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print("JSON saved to", args.json.resolve())


if __name__ == "__main__":
    main()
