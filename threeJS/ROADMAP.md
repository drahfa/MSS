# Roadmap

This roadmap prioritizes trust in the simulation before adding more visual or mission complexity. Dates are intentionally omitted; milestones should be promoted only when acceptance criteria pass.

## Current baseline: 1.0

- Five class-specific ROV models and dynamics profiles
- Matching visual and physical thruster channels
- Nonlinear 6-DOF browser simulation
- Three cameras, operator assist, adaptive telemetry, and autonomous demo
- Reproducible Blender-to-GLB fleet build

## 1.1 — Verification and repeatability (Complete)

Completed items & acceptance criteria:
- Line-for-line port of core MSS functions (`rbody`, `m2c`, `gRvect`, `eulerang`, `Smtrx`, `Hmtrx`, `rk4`, `ssa`)
- Coupled 6x6 mass and Coriolis solving in `src/mss/`
- Node unit test suite covering kinematics, matrix operations, and numerical solvers (`test/mss-library.test.js`)
- Pure-axis allocation tests for all supported and unsupported degrees of freedom across all 5 profiles (`test/allocation.test.js`)
- Model metadata automated verification ensuring rotor count and channel indices match `ROV_PROFILES` (`test/glb-metadata.test.js`)
- Deterministic 60-second scenario tests across all 5 vehicle profiles with finite state and decaying velocities (`test/rov-plant.test.js`)
- Fixed FPV camera look-at pitch direction, base URL path resolution, trail resets, paused rotor freeze, and 2D planar current drift

## 1.2 — Hydrodynamic and propulsion fidelity (Complete)

Goal: move from representative motion toward calibratable vehicle behavior.

Completed in 1.2:
- [x] External Vehicle Parameter Schema (`public/schemas/rov-profile.schema.json`) and standalone JSON profiles (`public/vehicles/*.json`)
- [x] Nonlinear thruster mechanics: deadband cutoff and asymmetric forward/reverse bollard thrust curves (`effectiveThrustRatio`)
- [x] Propeller advance ratio ($J_a$) and through-water axial inflow degradation (`advanceRatioFactor`)
- [x] Depth-dependent ocean current shear layer modeling with boundary-layer attenuation (`currentShear`)
- [x] Unit quaternion attitude propagation on $S^3$ (`euler2q`, `q2euler`, `Rquat`, `qdot`), eliminating gimbal lock singularities
- [x] Free-decay and step-response verification tests against analytical damped frequency, terminal velocity, and Lyapunov passivity (`test/hydrodynamics.test.js`)
- [x] Parameter import/export UI with real-time JSON validation (`src/mss/profileValidator.js`, `test/profile-validator.test.js`, and console dialog in `index.html`)
- [x] Documented calibration workflow against sea-trial or tank-test data (`CALIBRATION_WORKFLOW.md`)
- [x] Automated test suite expanded to 27/27 unit tests covering kinematics, quaternions, propulsion, schema, hydrodynamics, and hot-reloading
- [x] IEEE Transactions-style technical report and architectural specification (`to-publish/IEEE_TECHNICAL_REPORT.md`) with five benchmark case studies

Acceptance criteria (All Passed):
- A profile can be calibrated from an external parameter file without source edits (Validated via `test/profile-validator.test.js` & cockpit modal)
- Thruster and free-decay test cases can be compared against reference measurements (Validated via `test/hydrodynamics.test.js`, Table V)
- Energy and sign-convention checks are automated (Validated via Lyapunov passivity tests)

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
