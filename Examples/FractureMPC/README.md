# ROSCO gain-table fracture MPC

Controller release: `ROSCO 2.10.5+fracture.mpc.1`, based on upstream ROSCO
`v2.10.5`. The same optional solver modules are turbine-independent; turbine
configuration and validation artifacts are supplied separately.

This is an optional ROSCO-native research controller. Normal ROSCO pitch and
torque control is unchanged until the fracture transition is latched. All
configuration uses named `value ! FMPC_Parameter` entries in DISCON.IN.
The 5 MW positional configuration format is not used.

## Module ownership

| Module | Responsibility |
|---|---|
| `FractureMPC_Types.f90` | Turbine-independent parameter, gain-table and controller-state types |
| `FractureMPC_Table.f90` | Validated table loading, circular phase coordinates, normalized four-neighbour interpolation and coverage guard |
| `FractureMPC_Model.f90` | Reduced physical prediction model, actuator-state augmentation, finite-horizon condensation and constraints |
| `FractureMPC_QP.cpp` | Small C ABI adapter to qpOASES; matrix-layout conversion and solution feasibility check |
| `FractureMPC.f90` | ROSCO-style input parsing, measurement handling, fault latch, cadence, actuator commands and fallback |
| `DISCON.F90` | Optional integration hooks, guarded by the build feature |

Solver and scheduling code contains no 5/10/15 MW turbine physical constants.
The controller follows the five-MW MPC's constrained receding-horizon idea,
but does not copy its controller implementation or parameter layout.

## Build and run

Run `build_windows.ps1` from PowerShell. It detects neither new toolchains nor
installs global packages: paths to existing CMake, Intel ifx and qpOASES source
can be supplied as arguments. qpOASES is built into a separate ignored build
directory with a matching static C/C++ runtime. Existing 5 MW libraries are untouched.

The built DLL is `rosco/controller/build-fracture-mpc/Release/discon.dll`.
The existing `rosco/lib/discon.dll` remains the original ROSCO baseline.
On other platforms configure CMake with `ROSCO_FRACTURE_MPC=ON`,
`QPOASES_INCLUDE_DIR` and `QPOASES_LIBRARY`. The default build feature is OFF.
qpOASES 3.2.2 is an external LGPL-2.1-or-later dependency; retain its license and
relinking requirements when distributing binaries. Its source is not vendored here.

`IEA10/DISCON.IN` and `IEA10/FMPC_Gains.txt` are a concrete 10 MW configuration.
Set ServoDyn's `DLL_FileName` to the new DLL and `DLL_InFile` to this DISCON.IN.
Input table paths are relative to DISCON.IN. Enable a tower measurement node
(`NTwGages=1`, `TwrGagNd=20` for the current 20-node 10 MW tower) and verify
the extended tower displacement/velocity signals before running control comparisons.

Run `test_controller.py` in the bladefracture environment for synthetic DLL
contract tests, or `run_openfast_smoke.py` for an isolated actual-fracture run.
The latter preserves all original turbine input directories. Its 12 s test uses
5 m/s wind, fracture at 5 s and a 2 s structural-property ramp. It is not an
instantaneous-fracture validation or a complete shutdown performance study.

## Fracture contract

`FMPC_Mode=0` disables the feature; absence of the parameter also disables it.
`FMPC_Mode=1` enables it. `FMPC_Trigger=1` uses the modified OpenFAST interface:

| Record | Quantity |
|---|---|
| 129 | Advertised avrSWAP extent; checked before reading extended records |
| 1019, 1021 | Tower fore-aft velocity (m/s), displacement (m) |
| 1023 | Rotor-only inertia (blades plus hub), kg m^2 |
| 1025 | Fracture location, matched to gain-table metadata |
| 1027 | 0 healthy, 1 property ramp, 2 final fractured properties |

The table's blade ID must match the plant configuration. The current packet
does not transmit blade ID, so it cannot be automatically checked against the plant.
The runtime rotor inertia is augmented with generator inertia times gearbox
ratio squared exactly once, consistent with the 5 MW source's inertia boundary.
The normal unmodified OpenFAST executable used to generate LIN files does not
publish this fracture packet. Nonlinear fault runs need the modified executable.

`FMPC_Trigger=2` is an explicitly timed test switch using `FMPC_TriggerTime` and
configured fallback inertia. It does not itself fracture the plant.
No automatic detection of physical blade loss from vibration is implemented.
The transition is latched: losing the packet after a fault cannot return to normal generation.
Unsupported packet geometry or inertia invokes fallback rather than healthy control.

## Prediction and command boundaries

The augmented state is `[Omega, tower_x, tower_v, Tg/Tg_max, beta, phase_error]`.
The six table coordinates are `[Omega, beta, wind, alpha, sin(psi_d), cos(psi_d)]`,
all in SI units. All twelve aerodynamic torque/thrust derivatives are used.
The alpha dependence is eliminated using `J_eff = J - G_T_alpha`, with a
positive-inertia guard. Phase is predicted on the circle with a nominal phase
advance and a local phase-error correction. Wind is held over the horizon;
tower fore-aft velocity enters as relative-wind perturbation.

Measured rotor and tower acceleration anchor the local affine model. They
absorb current gravity, structural and load-offset mismatch at each update;
there is no independently calibrated future imbalance-gravity model. The short
frozen-model horizon and six-coordinate gain interpolation remain approximations.

qpOASES optimizes torque and pitch commands with absolute bounds, command-rate
constraints, monotonic post-fault pitch commands, and nonnegative predicted
rotor speed. The objective penalizes speed relative to a configurable deceleration
reference, tower motion, torque and departure from feather pitch. Commands are
held between MPC updates. The post-fault branch explicitly owns first-order
pitch/torque actuator states and enforces ROSCO physical magnitude/rate limits
again at each plant callback. The normal ROSCO actuator behavior applies before
the switch; post-fault actuator time constants are explicit FMPC parameters.

`FMPC_ExtendedMotion=1` uses the custom displacement and velocity records.
Mode 0 instead uses bounded leaky integration of tower acceleration; this observer
does not recover absolute static displacement and requires separate calibration.
Restart/checkpoint continuation with FMPC enabled is not supported. Use fresh
simulations; original ROSCO restart behavior remains available with FMPC disabled.

## Guard statuses in `<root>.FMPC.csv`

1 = MPC applied; 2 = property-ramp guard; 3 = outside table coverage;
4 = QP/model failure; 5 = low/reversed rotor speed; 6 = native ROSCO shutdown;
7 = unsupported/missing extended feedback after a fault.
Fallback commands feather pitch and unload torque at bounded rates. Native
ROSCO shutdown has priority and retains its own command and actuator handling.
The trace
separates demanded commands from applied actuator outputs and records the
lookup distance, QP status, feasibility residual and current equivalent inertia.

## Moving to 15 MW

Use `export_turbine.py` with the 15 MW local derivative CSV, base ROSCO DISCON,
ElastoDyn/tower files and healthy/fractured BeamDyn blade properties, plus
`--rated-rpm 7.56` and a different output directory. No controller source change
or turbine-specific compilation is required.

Changing turbines also requires the appropriate physical parameters and ROSCO
actuator limits, not just a CSV filename. The exporter derives a documented
rotor-inertia fallback and rigid-base first-tower-mode mass/stiffness/damping
estimate. These estimates do not include calibrated monopile/soil flexibility
or tower gravity stiffening. Identify the reduced tower dynamics and validate
closed-loop performance before claiming safety/load benefits for either turbine.
This implementation and its smoke tests do not establish superiority over
ROSCO shutdown, full-envelope stability, or 15 MW runtime performance.

## Git workflow

Work is isolated on `feature/fracture-table-mpc` in this ROSCO repository.
Open that branch in GitHub Desktop to inspect the source and example configuration.
The separate 5 MW controller repository is read-only in this work. No remote
push or changes to the original deployed ROSCO DLL are performed.
