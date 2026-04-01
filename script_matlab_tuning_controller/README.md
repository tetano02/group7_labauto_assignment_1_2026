# PI Controller Tuning via Frequency-Domain Optimisation

A MATLAB toolkit for tuning a PI controller using constrained optimisation in the frequency domain. The optimiser shapes the open-loop transfer function `L(jω) = C(jω)·G(jω)` to meet robustness, stability, and performance specifications simultaneously.

---

## Table of Contents

- [PI Controller Tuning via Frequency-Domain Optimisation](#pi-controller-tuning-via-frequency-domain-optimisation)
  - [Table of Contents](#table-of-contents)
  - [Background](#background)
    - [Loop Shaping](#loop-shaping)
    - [The PI Controller](#the-pi-controller)
  - [File Overview](#file-overview)
  - [Function Reference](#function-reference)
    - [`pi_frequency_response.m`](#pi_frequency_responsem)
    - [`find_cutting_frequency.m`](#find_cutting_frequencym)
    - [`fitness.m`](#fitnessm)
    - [`constraints.m`](#constraintsm)
  - [Main Script](#main-script)
    - [Plant and Notch Filter](#plant-and-notch-filter)
    - [Constraint Parameters](#constraint-parameters)
    - [Initial Guess](#initial-guess)
    - [Optimisation](#optimisation)
    - [Verification Plots](#verification-plots)
  - [Constraint Summary](#constraint-summary)
  - [Tuning Guide](#tuning-guide)
  - [Dependencies](#dependencies)

---

## Background

### Loop Shaping

The design philosophy is **loop shaping**: rather than placing closed-loop poles directly, we shape the frequency response of the open-loop transfer function:

```
L(jω) = C(jω) · G(jω)
```

where `G(jω)` is the fixed plant (motor velocity response) and `C(jω)` is the PI controller to be designed. Good loop shape means:

| Frequency region | Requirement | Reason |
|---|---|---|
| Low (`ω ≤ ω_low`) | High gain | Reject load disturbances |
| At crossover `ωc` | `\|L\| = 1`, sufficient phase | Bandwidth and stability margin |
| High (`ω ≥ ω_high`) | Low gain | Suppress sensor noise |

### The PI Controller

The controller has the transfer function:

```
C(s) = Kp · (1 + 1/(Ti·s))
```

The two tunable parameters are:
- **Kp** — proportional gain (scales overall loop gain)
- **Ti** — integral time constant (position of the integrator zero at `ω = 1/Ti`)

The optimiser searches for the `[Ti, Kp]` pair that minimises the distance between the actual crossover frequency and the target, subject to all frequency-domain constraints.

---

## File Overview

```
.
├── pi_frequency_response.m     % Builds the PI frequency response function handle
├── find_cutting_frequency.m    % Finds the 0 dB gain crossover frequency
├── fitness.m                   % Optimisation cost function
├── constraints.m               % Nonlinear inequality constraints for fmincon
├── taratura_ottima.mlx         % Top-level script: setup, optimise, plot
└── model_joint1.mat            % Identified plant model (required external file)
```

---

## Function Reference

---

### `pi_frequency_response.m`

**Purpose:** Given a PI parameter vector, returns a function handle that evaluates the controller's complex frequency response `C(jω)` at any set of frequencies.

**Signature:**
```matlab
controller = pi_frequency_response(controller_parameters)
```

**Arguments:**

| Name | Type | Description |
|---|---|---|
| `controller_parameters` | `1×2 vector` | `[Ti, Kp]` — integral time and proportional gain |

**Returns:**

| Name | Type | Description |
|---|---|---|
| `controller` | `function handle @(w)` | Evaluates `C(jω)` at frequency vector `w`, returns complex column vector |

**How it works:**

1. Extracts `Kp = controller_parameters(2)` and `Ti = controller_parameters(1)`.
2. Builds the transfer function `C(s) = Kp*(1 + 1/(Ti*s))` using MATLAB's `tf` object.
3. Returns `@(w) reshape(freqresp(C, w), length(w), 1)` — a lazy evaluator that computes the frequency response on demand.

**Why a function handle?** The optimiser calls this function thousands of times with different parameter vectors. Returning a handle rather than a pre-evaluated array means the response is always computed for the current candidate parameters.

**Example:**
```matlab
controller = pi_frequency_response([0.1, 5.0]);   % Ti=0.1, Kp=5
w = logspace(-2, 4, 500)';
C_jw = controller(w);   % complex column vector, length 500
```

---

### `find_cutting_frequency.m`

**Purpose:** Finds the gain crossover frequency `ωc` — the frequency at which `|L(jωc)| = 1` (0 dB).

**Signature:**
```matlab
wc = find_cutting_frequency(frequency_response, frequency_vector)
```

**Arguments:**

| Name | Type | Description |
|---|---|---|
| `frequency_response` | `function handle @(w)` | Returns complex `L(jω)` at frequencies `w` |
| `frequency_vector` | `N×1 vector` | Frequency grid [rad/s] for the initial coarse scan |

**Returns:**

| Name | Type | Description |
|---|---|---|
| `wc` | `scalar` or `[]` | Crossover frequency [rad/s], or empty if no crossing exists |

**Algorithm — three stages:**

```
Stage 1 — Coarse scan
    Evaluate |L(jω)| over the full frequency grid.
    If both values > 1 and < 1 exist → crossing is inside the grid (Case a).

Stage 2 — Bracket (only if Stage 1 finds no crossing)
    Case b: |L| < 1 everywhere → start at ω = 1e-80, multiply by 10
            until |L| > 1 (crossing is at a lower frequency than the grid).
    Case c: |L| > 1 everywhere → start at ω = 1e+80, divide by 10
            until |L| < 1 (crossing is at a higher frequency than the grid).
    If no crossing found after bracketing → issue warning, return [].

Stage 3 — Refine
    Call fzero(@(w) |L(jw)| - 1, bracket_point) for machine-precision result.
    If fzero returns a negative ω (rare edge case), reflect and re-solve.
```

**Why geometric steps in Stage 2?** Crossover frequencies can span many decades. Multiplying/dividing by 10 covers the full range in at most ~160 steps regardless of where the crossing lies. Arithmetic steps would require millions of iterations for the same coverage.

**Why `fzero` in Stage 3?** `fzero` uses bisection + secant + inverse quadratic interpolation, converging superlinearly. Starting from a bracketed point (within one decade of the true root) it typically needs fewer than 10 function evaluations to reach full precision.

**Returns `[]` when:**
- `|L(jω)| < 1` for all `ω` (loop gain never reaches 0 dB — system has very low bandwidth)
- `|L(jω)| > 1` for all `ω` (system is not strictly proper — does not roll off)

---

### `fitness.m`

**Purpose:** Cost function minimised by the optimiser. Measures how close the actual crossover frequency is to the designer's target.

**Signature:**
```matlab
J = fitness(controller_parameters, controller_frequency_response, ...
            process_frequency_response, desired_wc, frequency_vector)
```

**Arguments:**

| Name | Type | Description |
|---|---|---|
| `controller_parameters` | `1×2 vector` | `[Ti, Kp]` candidate parameters |
| `controller_frequency_response` | `function handle` | Factory: params → `@(w)` for `C(jω)` |
| `process_frequency_response` | `function handle @(w)` | Plant `G(jω)` |
| `desired_wc` | `scalar` | Target crossover frequency [rad/s] |
| `frequency_vector` | `N×1 vector` | Frequency grid for crossover search |

**Returns:**

| Name | Type | Description |
|---|---|---|
| `J` | `scalar ≥ 0` | Cost value (lower = better) |

**Cost definition:**

```
J = (actual_wc - desired_wc)²    if a crossover exists
J = 1 000 000                    if no crossover found (penalty)
```

The squared error is always non-negative and penalises large deviations more heavily than small ones — desirable properties for a gradient-based optimiser. The large penalty for the no-crossover case acts as a soft wall, steering the optimiser away from parameter regions where the loop gain never reaches 0 dB.

**Alternative cost (not implemented here):** `J = -actual_wc` to maximise bandwidth rather than target a specific value.

---

### `constraints.m`

**Purpose:** Computes the four nonlinear inequality constraints passed to `fmincon` as `nonlcon`. Each constraint is formulated as `cin(i) ≤ 0` (satisfied when the actual value does not exceed its limit).

**Signature:**
```matlab
[cin, ceq] = constraints(controller_parameters, controller_frequency_response, ...
    process_frequency_response, desired_maximum_sensitivity, desired_phase_margin, ...
    low_freq_disturbance_attenuation, low_freq_threshold, ...
    high_freq_noise_attenuation, high_freq_threshold, frequency_vector)
```

**Arguments:**

| Name | Type | Description |
|---|---|---|
| `controller_parameters` | `1×2 vector` | `[Ti, Kp]` candidate |
| `controller_frequency_response` | `function handle` | Controller factory |
| `process_frequency_response` | `function handle @(w)` | Plant `G(jω)` |
| `desired_maximum_sensitivity` | `scalar` | Upper bound on `max\|S(jω)\|` |
| `desired_phase_margin` | `scalar` | Lower bound on phase margin [deg] |
| `low_freq_disturbance_attenuation` | `scalar` | Upper bound on `\|G/(1+L)\|` for `ω ≤ ω_low` |
| `low_freq_threshold` | `scalar` | `ω_low` [rad/s] |
| `high_freq_noise_attenuation` | `scalar` | Upper bound on `\|L/(1+L)\|` for `ω ≥ ω_high` |
| `high_freq_threshold` | `scalar` | `ω_high` [rad/s] |
| `frequency_vector` | `N×1 vector` | Evaluation grid |

**Returns:**

| Name | Type | Description |
|---|---|---|
| `cin` | `1×4 vector` | Inequality constraints (feasible when all `≤ 0`) |
| `ceq` | `[]` | No equality constraints used |

**The four constraints:**

| Index | Expression | Physical meaning |
|---|---|---|
| `cin(1)` | `max\|G/(1+L)\| - limit ≤ 0` for `ω ≤ ω_low` | Disturbance rejection at low frequencies |
| `cin(2)` | `max\|L/(1+L)\| - limit ≤ 0` for `ω ≥ ω_high` | Noise attenuation at high frequencies |
| `cin(3)` | `max\|1/(1+L)\| - Ms_limit ≤ 0` | Sensitivity peak — robustness to uncertainty |
| `cin(4)` | `PM_desired - PM_actual ≤ 0` | Phase margin — sign reversed (lower bound) |

**Phase margin computation:**

```matlab
L_phase    = unwrap(angle(L(frequency_vector)));   % remove 2π jumps
phase_wc   = L_phase(i_wc);                        % phase at crossover
actual_PM  = rad2deg(pi + phase_wc);               % PM in degrees
```

`unwrap()` is essential: without it, a 2π phase jump below `ωc` would give a falsely optimistic (or pessimistic) phase margin reading.

**NaN guard:** If any `cin(i)` is `NaN` (e.g. due to a degenerate parameter set), the entire vector is replaced with `1e5`. This keeps `fmincon` from crashing and treats the candidate as deeply infeasible.

---

## Main Script

### Plant and Notch Filter

```matlab
load ../model_joint1.mat
sys_vel_motore = tf(modello_continuo);   % torque-to-velocity plant G(s)
```

A notch filter is cascaded with the plant to suppress a mechanical resonance at `ωn = 205 rad/s` (~33 Hz):

```
         s² + 2·ξz·ωn·s + ωn²
N(s) = ──────────────────────────
         s² + 2·ξp·ωn·s + ωn²
```

with `ξz = 0.2` (sharp zeros) and `ξp = 0.7` (wide poles). The augmented plant seen by the controller is `Plant = G(s)·N(s)`.

> **Important:** The notch filter must be implemented identically in the real-time controller. The designed `C(s)` assumes `N(s)` is present in the loop.

---

### Constraint Parameters

```matlab
desired_wc                       = 30;       % Target crossover [rad/s]
desired_phase_margin             = 50;       % Min phase margin [deg]
desired_maximum_sensitivity      = 2;        % Max |S(jω)| peak
low_freq_disturbance_attenuation = 0.1;      % Max |G/(1+L)| for ω ≤ 0.001
low_freq_threshold               = 0.001;    % [rad/s]
high_freq_noise_attenuation      = 0.1;      % Max |L/(1+L)| for ω ≥ 50000
high_freq_threshold              = 50000;    % [rad/s]
```

**Effect of tightening each parameter:**

| Parameter | Tighter value | Trade-off |
|---|---|---|
| `desired_phase_margin` | > 50° | More robust, lower achievable bandwidth |
| `desired_maximum_sensitivity` | < 2.0 | More robust, harder to satisfy at high bandwidth |
| `low_freq_disturbance_attenuation` | < 0.1 | Better disturbance rejection, needs higher low-freq gain |
| `high_freq_noise_attenuation` | < 0.1 | Less noise amplification, limits controller gain at high freq |

---

### Initial Guess

A physically motivated starting point is computed analytically:

```matlab
Ti = 1 / (0.01 * wc_des);            % zero two decades below wc
C0 = 1 + 1 / (Ti * s);               % unit-gain PI shape
Kp = 1 / abs(freqresp(C0*Plant, wc_des));  % scale to achieve |L(jwc)| = 1
x0 = [Kp, Ti];
```

This places the first iterate close to (or inside) the feasible region, which is critical for `fmincon`'s gradient-based search to find a good solution quickly.

---

### Optimisation

Two modes are available, selected by the flag `LOCAL_OPTIMIZATION`:

**Local (`fmincon`):**
```matlab
[x, fval, exitflag] = fmincon(optim_fitness, x0, [], [], [], [], ...
    lower_bound, upper_bound, optim_constraints);
```
- Fast (seconds), deterministic, finds nearest local minimum.
- Sufficient when the analytical `x0` is in a good basin of attraction.

**Global (`GlobalSearch` + `fmincon`):**
```matlab
gs      = GlobalSearch('Display', 'iter');
problem = createOptimProblem('fmincon', 'x0', x0, 'objective', optim_fitness, ...
    'lb', lower_bound, 'ub', upper_bound, 'nonlcon', optim_constraints);
[x, fval, exitflag] = run(gs, problem);
```
- Slower (minutes), explores many starting points, reduces risk of poor local minima.
- Use when the local result looks physically suspicious.

**Exit flag interpretation:**

| `exitflag` | Meaning |
|---|---|
| `> 0` | Converged successfully |
| `= 0` | Iteration limit reached — result may still be usable |
| `< 0` | Failed — check constraints or initial guess |

---

### Verification Plots

Three figures are generated after optimisation:

**Figure 1 — Open-loop Bode magnitude `|L(jω)|`**
Shows the shaped loop gain with constraint markers overlaid:
- Black `*` and L-shaped line: minimum required gain at `ω_low` (disturbance constraint)
- Red `*` and L-shaped line: maximum allowed gain at `ω_high` (noise constraint)

**Figure 2 — Stability margins**
`margin(C * Plant)` annotates the Bode plot with actual gain margin (dB) and phase margin (°). Compare these against `desired_phase_margin`.

**Figure 3 — Sensitivity function `|S(jω)|`**
`bodemag(1/(1+C*Plant))` with a dashed horizontal line at `20·log10(Ms)`. The sensitivity curve must stay entirely below this line for `cin(3) ≤ 0`.

---

## Constraint Summary

| `cin(i)` | Constraint | Closed-loop interpretation |
|---|---|---|
| `cin(1) ≤ 0` | `\|G/(1+L)\|` small at low `ω` | Load disturbances at low frequency are attenuated |
| `cin(2) ≤ 0` | `\|L/(1+L)\|` small at high `ω` | Sensor noise is not passed to the plant input |
| `cin(3) ≤ 0` | `max\|1/(1+L)\| ≤ Ms` | System is robust to gain/phase uncertainty |
| `cin(4) ≤ 0` | `PM_actual ≥ PM_desired` | Closed loop is sufficiently damped |

All four are evaluated numerically over `frequency_vector = logspace(-5, 5, 100000)`.

---

## Tuning Guide

**Problem: optimiser exits with `exitflag ≤ 0`**
- The feasible region may be empty — try relaxing `desired_maximum_sensitivity` (increase to 2.5) or `desired_phase_margin` (reduce to 40°).
- Try a different `x0`. A rough rule: `Ti ≈ 1/(0.01·wc_des)`, `Kp` chosen for unit gain at `wc_des`.
- Switch to `LOCAL_OPTIMIZATION = false` for a global search.

**Problem: result has very low phase margin despite `cin(4) ≤ 0`**
- Check for `exitflag = 0` (iteration limit) — the constraint may have been marginally satisfied but not reliably.
- Tighten the `desired_phase_margin` to give more margin in the optimisation.

**Problem: notch filter causes unexpected behaviour**
- Verify `ωn` matches the actual resonance frequency in the identified model.
- Increase `xci_p` (wider notch poles) to reduce the phase loss introduced by the filter near `ωc`.
- Check that the same notch coefficients are used in the real-time implementation.

**Increasing bandwidth (`desired_wc`)**
- A higher `wc` generally requires a larger `Kp` and may push the sensitivity peak above `Ms`.
- Relax `desired_maximum_sensitivity` slightly, or increase `desired_phase_margin` to ensure the solver does not settle for a marginally stable solution.

---

## Dependencies

| Toolbox | Used for |
|---|---|
| Control System Toolbox | `tf`, `freqresp`, `margin`, `bodemag` |
| Optimisation Toolbox | `fmincon` |
| Global Optimisation Toolbox | `GlobalSearch`, `createOptimProblem` (optional) |

**External file required:** `../model_joint1.mat` — must contain a variable `modello_continuo` representing the identified continuous-time plant model (state-space or transfer function).