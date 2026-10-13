# Roadmap

This roadmap prioritizes trust in the simulation before adding more visual or mission complexity. Dates are intentionally omitted; milestones should be promoted only when acceptance criteria pass.

## Current baseline: 1.0

- Five class-specific ROV models and dynamics profiles
- Matching visual and physical thruster channels
- Nonlinear 6-DOF browser simulation
- Three cameras, operator assist, adaptive telemetry, and autonomous demo
- Reproducible Blender-to-GLB fleet build

## 1.1 — Verification and repeatability

Goal: make physics and asset regressions detectable without manual inspection.

- Add unit tests for rotations, Coriolis terms, restoring forces, RK4, and matrix allocation
- Add pure-axis allocation tests for every supported and unsupported degree of freedom
- Add deterministic scenario tests with stored tolerances for state histories
- Validate every GLB's rotor count and `channel_index` metadata against `ROV_PROFILES`
- Add browser smoke tests for class switching, cameras, demo mode, and keyboard controls
- Add visual regression views for propeller clearance and responsive layouts
- Run build and tests in continuous integration

Acceptance criteria:

- All five profiles complete deterministic 60-second scenarios with finite state
- Model/channel mismatches fail the build
- UI smoke tests pass at desktop, tablet, and mobile breakpoints

## 1.2 — Hydrodynamic and propulsion fidelity

Goal: move from representative motion toward calibratable vehicle behavior.

- Support full coupled rigid-body, added-mass, and damping matrices
- Add configurable thrust curves, deadband, asymmetric forward/reverse output, and voltage effects
- Model advance ratio, local inflow, wake interaction, and optional cavitation limits
- Add vertical and horizontal current shear rather than a single planar current vector
- Introduce quaternion attitude propagation while retaining Euler telemetry
- Add parameter import/export with units and schema validation
- Document a calibration workflow against sea-trial or tank-test data

Acceptance criteria:

- A profile can be calibrated from an external parameter file without source edits
- Thruster and free-decay test cases can be compared against reference measurements
- Energy and sign-convention checks are automated

## 1.3 — Mission operations

Goal: support repeatable piloting and mission-analysis workflows.

- Add gamepad mapping with dead zones and remapping
- Add waypoint editing, mission save/load, and route validation
- Add CSV/JSON telemetry recording and deterministic replay
- Add simulated DVL, depth, compass, IMU, and camera measurements with configurable noise
- Add tether length, drag, payout, and snag warnings
- Add seabed and structure collision/contact response
- Add failure injection for a disabled, degraded, or stuck thruster

Acceptance criteria:

- A mission can be authored, run, exported, and replayed with the same initial conditions
- Failure states are visible in both the allocator and operator console

## 2.0 — Validated training platform

Goal: establish a versioned, evidence-backed configuration suitable for structured training and engineering studies.

- Validate at least one profile against documented vehicle data
- Add scenario packs, scoring, event logs, and instructor controls
- Version vehicle and mission schemas independently from the application
- Publish uncertainty bounds and validation reports
- Complete accessibility testing and localization support

This milestone must not be described as safety-critical or hardware-certified without an appropriate independent validation process.

## Maintenance backlog

- Split Three.js and scene code into cacheable chunks to remove the current Vite bundle advisory
- Decide whether to remove the unused `one-person-submarine.mjs` legacy asset
- Move inline console markup into small tested view components if UI complexity grows
- Add release automation and generated artifact checksums
