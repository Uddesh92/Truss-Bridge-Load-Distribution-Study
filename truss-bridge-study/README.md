# Truss Bridge Load Distribution Study

**Project 2 — Comparative structural analysis of Pratt vs. Howe truss bridges**

A hands-on structural engineering project comparing the load-carrying behavior
of Pratt and Howe truss configurations, built from popsicle sticks and tested
under static point loading. Includes a Python method-of-joints force
calculator used to predict critical members before physical testing.

## 📌 Project Overview

Two truss bridges — one Pratt, one Howe — were built to the same span using
the same materials and joining method (Fevicol + tape), then tested under an
identical 500 g static load to compare failure behavior, deflection, and
structural efficiency.

## 🔧 Methodology

1. Predicted the critical (highest-force) member in each truss using
   method-of-joints hand calculations.
2. Built both bridges from popsicle sticks with identical span and height.
3. Applied a 500 g static test load to each bridge.
4. Recorded actual failure load, first-failure member, and failure mode.
5. Compared predicted vs. actual failure member to evaluate the accuracy of
   the idealized (pin-jointed, 2D) structural model.

## 📊 Results

### Prediction vs. Actual

| Truss Type | Mass (g) | Predicted Failure Member | Predicted Force (N) | Actual Failure Load (N) | Actual First-Fail Member |
|---|---|---|---|---|---|
| Pratt | 100 | Bottom chord (outer) | 7.29 | 6.8 | Top chord outer |
| Howe  | 85  | Bottom chord (outer) | 7.29 | 4.90 | Diagonals |

### Failure Mode Accuracy

| Truss Type | Failure Mode | Prediction Correct? |
|---|---|---|
| Pratt | Deflection | No |
| Howe  | Deflection | No |

### Final Report

| Metric | Pratt | Howe |
|---|---|---|
| Bridge Weight (g) | 100 | 85 |
| Number of Sticks | 98 | 78 |
| Joining Method | Fevicol + Tape | Fevicol + Tape |
| Test Load Applied (g) | 500 | 500 |
| Failure | No (structure intact) | No (structure intact) |
| Observed Behaviour | Sway/deflection | Sway/deflection |
| Deflection | Present (without fracture) | Present (without fracture) |
| Load Capacity (g) | 500 | 500 |
| **Efficiency Ratio** | **5.0** | **~5.9** |

> Efficiency ratio = load capacity ÷ bridge weight. Howe achieved a **~18%
> higher efficiency ratio** than Pratt despite carrying the same 500 g load,
> because it did so with 15 g less material.

## 🔑 Key Takeaway

Both bridges survived the test load without fracturing — the limiting
factor in both cases was **sway/deflection**, not member failure as the
idealized method-of-joints model predicted. This highlights a real gap
between 2D pin-jointed truss theory and physical popsicle-stick structures,
where glue-joint stiffness, out-of-plane buckling, and lateral bracing
dominate real-world failure — not just axial member force.

## 🐍 Force Calculator

[`src/truss_force_calculator.py`](src/truss_force_calculator.py) is a
method-of-joints solver that:
- Builds Pratt or Howe truss geometry parametrically (any span/height/panel count)
- Assembles and solves the static equilibrium equations with NumPy
- Reports the axial force in every member
- Flags the predicted critical member (max compression / max tension)

Run it directly:

```bash
python src/truss_force_calculator.py
```

Or use it programmatically:

```python
from truss_force_calculator import solve_truss, print_report

result = solve_truss("howe", n_panels=6, span_m=0.60, height_m=0.10,
                      load_n=500 * 9.81 / 1000)
print_report(result, "howe")
```

## 📁 Repository Structure

```
truss-bridge-study/
├── README.md
├── data/
│   ├── results.json          # structured results + final report data
│   └── final_report.csv      # spreadsheet-friendly version
└── src/
    └── truss_force_calculator.py
```

## 🎓 About

Built as part of an aerospace engineering hands-on structures portfolio
(sub-₹5,000 budget, household-tool build constraints), targeting practical
structural intuition relevant to ISRO/HAL-style structures work.

## 📄 License

MIT
