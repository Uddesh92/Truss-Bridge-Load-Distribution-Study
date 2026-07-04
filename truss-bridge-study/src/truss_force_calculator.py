"""
Truss Bridge Force Calculator
==============================
Method-of-joints solver for parallel-chord Pratt and Howe trusses.

Given a truss type, number of panels, span, height and applied load(s),
this script builds the joint geometry, assembles the static equilibrium
equations (sum Fx = 0, sum Fy = 0 at every joint), solves for every
member's internal axial force, and reports which member is predicted
to fail first (highest compression -> buckling risk for thin members,
highest tension -> snapping risk for brittle joints).

Author: Uranium's Aerospace Project Portfolio
Project: Truss Bridge Load Distribution Study (Project-2)

Usage:
    python truss_force_calculator.py

Or import and use programmatically:
    from truss_force_calculator import solve_truss
    result = solve_truss("pratt", n_panels=6, span_m=0.60, height_m=0.10,
                          load_n=500*9.81/1000, load_joint=3)
"""

import numpy as np


# ---------------------------------------------------------------------
# 1. GEOMETRY BUILDER
# ---------------------------------------------------------------------
def build_truss(truss_type: str, n_panels: int, span_m: float, height_m: float):
    """
    Build node coordinates and member connectivity for a parallel-chord
    Pratt or Howe truss.

    Bottom chord nodes:  B0 ... Bn   (y = 0)
    Top chord nodes:     T0 ... Tn   (y = height)
    Verticals:           Bi - Ti for every i
    Diagonals:           one per panel, direction depends on truss type
                          and which half of the span the panel sits in
                          (diagonals always point toward mid-span, which
                          is the classic Pratt arrangement; Howe is the
                          mirror image of Pratt).

    Returns:
        nodes: dict {name: (x, y)}
        members: list of (node_a, node_b) tuples
    """
    truss_type = truss_type.lower()
    if truss_type not in ("pratt", "howe"):
        raise ValueError("truss_type must be 'pratt' or 'howe'")

    n = n_panels
    w = span_m / n  # panel width

    nodes = {}
    for i in range(n + 1):
        nodes[f"B{i}"] = (i * w, 0.0)
        nodes[f"T{i}"] = (i * w, height_m)

    members = []

    # Chords
    for i in range(n):
        members.append((f"B{i}", f"B{i+1}"))  # bottom chord
        members.append((f"T{i}", f"T{i+1}"))  # top chord

    # Verticals
    for i in range(n + 1):
        members.append((f"B{i}", f"T{i}"))

    # Diagonals: alternate direction at mid-span so diagonals always
    # slope "uphill" toward the center of the bridge (Pratt), or
    # "downhill" toward the center (Howe = mirror of Pratt).
    for i in range(n):
        left_half = i < n / 2
        if truss_type == "pratt":
            diag = (f"B{i}", f"T{i+1}") if left_half else (f"T{i}", f"B{i+1}")
        else:  # howe
            diag = (f"T{i}", f"B{i+1}") if left_half else (f"B{i}", f"T{i+1}")
        members.append(diag)

    return nodes, members


# ---------------------------------------------------------------------
# 2. METHOD-OF-JOINTS SOLVER
# ---------------------------------------------------------------------
def solve_truss(truss_type, n_panels, span_m, height_m, load_n, load_joint=None):
    """
    Solve member forces for a simply-supported truss (pin at B0,
    roller at Bn) carrying a single downward point load.

    load_joint defaults to the bottom-chord joint nearest mid-span.

    Returns a dict with:
        'forces'        : {member_name: force_N}  (+ tension, - compression)
        'max_tension'   : (member, force)
        'max_compression': (member, force)
        'nodes', 'members'
    """
    nodes, members = build_truss(truss_type, n_panels, span_m, height_m)
    joint_names = list(nodes.keys())
    n_joints = len(joint_names)
    n_members = len(members)

    if load_joint is None:
        load_joint = f"B{n_panels // 2}"

    # Supports: pin at B0 (Rx0, Ry0), roller at Bn (Ry_n only)
    support_dofs = ["Rx_B0", "Ry_B0", "Ry_last"]
    unknowns = [f"F_{a}_{b}" for a, b in members] + support_dofs
    n_unknowns = len(unknowns)
    n_eq = 2 * n_joints

    A = np.zeros((n_eq, n_unknowns))
    b = np.zeros(n_eq)

    joint_index = {name: idx for idx, name in enumerate(joint_names)}

    # External load (downward, negative y) applied at load_joint
    b[2 * joint_index[load_joint] + 1] -= load_n

    # Member contributions: unit vector from joint -> other end
    for m_idx, (a, b_node) in enumerate(members):
        xa, ya = nodes[a]
        xb, yb = nodes[b_node]
        dx, dy = xb - xa, yb - ya
        length = np.hypot(dx, dy)
        ux, uy = dx / length, dy / length

        ia, ib = joint_index[a], joint_index[b_node]
        # Force F_ab pulls joint a toward b, and joint b toward a
        A[2 * ia, m_idx] += ux
        A[2 * ia + 1, m_idx] += uy
        A[2 * ib, m_idx] += -ux
        A[2 * ib + 1, m_idx] += -uy

    # Support reactions
    i_rx0 = n_members + 0
    i_ry0 = n_members + 1
    i_ryn = n_members + 2
    A[2 * joint_index["B0"], i_rx0] = 1.0
    A[2 * joint_index["B0"] + 1, i_ry0] = 1.0
    A[2 * joint_index[f"B{n_panels}"] + 1, i_ryn] = 1.0

    # Solve (least-squares handles the statically-determinate system
    # cleanly even with floating point round-off)
    solution, *_ = np.linalg.lstsq(A, b, rcond=None)

    forces = {f"{a}-{b_node}": solution[i] for i, (a, b_node) in enumerate(members)}

    max_tension = max(forces.items(), key=lambda kv: kv[1])
    max_compression = min(forces.items(), key=lambda kv: kv[1])

    return {
        "forces": forces,
        "reactions": {
            "Rx_B0": solution[i_rx0],
            "Ry_B0": solution[i_ry0],
            "Ry_last": solution[i_ryn],
        },
        "max_tension": max_tension,
        "max_compression": max_compression,
        "nodes": nodes,
        "members": members,
        "load_joint": load_joint,
        "load_n": load_n,
    }


# ---------------------------------------------------------------------
# 3. REPORTING
# ---------------------------------------------------------------------
def print_report(result, truss_type):
    print(f"\n{'='*55}")
    print(f" {truss_type.upper()} TRUSS  —  FORCE ANALYSIS")
    print(f"{'='*55}")
    print(f"Load applied at joint {result['load_joint']}: "
          f"{result['load_n']:.3f} N\n")

    print(f"{'Member':<12}{'Force (N)':>12}   Type")
    print("-" * 40)
    for member, f in sorted(result["forces"].items(), key=lambda kv: -abs(kv[1])):
        ftype = "Tension" if f > 1e-9 else ("Compression" if f < -1e-9 else "Zero-force")
        print(f"{member:<12}{f:>12.2f}   {ftype}")

    mt_name, mt_val = result["max_tension"]
    mc_name, mc_val = result["max_compression"]
    print("\nPredicted first-failure member (buckling risk, max compression):")
    print(f"   {mc_name}  ->  {mc_val:.2f} N (compression)")
    print("Predicted highest-tension member:")
    print(f"   {mt_name}  ->  {mt_val:.2f} N (tension)")


def compare_pratt_and_howe(n_panels=6, span_m=0.60, height_m=0.10,
                            load_n=500 * 9.81 / 1000):
    """Convenience wrapper: run both truss types with identical
    geometry/load and print a side-by-side style report — mirrors
    the 'Results' table format used in the project logbook."""
    for t in ("pratt", "howe"):
        result = solve_truss(t, n_panels, span_m, height_m, load_n)
        print_report(result, t)


# ---------------------------------------------------------------------
# 4. DEMO / ENTRY POINT
# ---------------------------------------------------------------------
if __name__ == "__main__":
    # Example matching a typical popsicle-stick bridge test:
    # ~60 cm span, ~10 cm height, 6 panels, 500 g test load.
    TEST_LOAD_N = 500 * 9.81 / 1000  # 500 g -> Newtons ≈ 4.905 N

    compare_pratt_and_howe(
        n_panels=6,
        span_m=0.60,
        height_m=0.10,
        load_n=TEST_LOAD_N,
    )

    print("\nNote: this idealised model assumes pin joints and a single")
    print("point load. Real popsicle-stick bridges also fail from glue-")
    print("joint weakness and out-of-plane sway, which this 2D model")
    print("does not capture — matching the 'Deflection, not member")
    print("fracture' failure mode observed in the physical test.")
