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

## 1.3 — Mission operations (Complete)

Goal: support repeatable piloting and mission-analysis workflows.

Completed in 1.3:
- [x] Add gamepad mapping with dead zones and remapping (`src/gamepad.js`, HTML5 Gamepad API)
- [x] Add waypoint editing, mission save/load, and route safety validation (`src/missionRoute.js`)
- [x] Add CSV/JSON telemetry recording and deterministic replay (`src/telemetryRecorder.js`, exact repeatability $< 10^{-4}$)
- [x] Add simulated DVL, depth, compass, IMU, and altimeter measurements with configurable noise, drift, and dropouts (`src/sensors.js`)
- [x] Add tether length, catenary payout, quadratic line drag, elastic tension, and snag warnings (`src/tether.js`)
- [x] Add seabed and structure elastic-viscous penalty contact model with Coulomb friction (`src/contact.js`)
- [x] Add failure injection for disabled, degraded, or stuck thrusters with allocator awareness (`src/failureInjection.js`)
- [x] Cockpit HUD integration: Gamepad indicator, Tether payout & snag alarms, DVL bottom-lock status, Fault injection toggle, and CSV telemetry export
- [x] Automated test suite expanded to 33/33 unit tests passing (`test/mission-operations.test.js`)

Acceptance criteria (All Passed):
- A mission can be authored, run, exported, and replayed with the same initial conditions (Validated via `test/mission-operations.test.js` & cockpit CSV export)
- Failure states are visible in both the allocator and operator console (Validated via `test/mission-operations.test.js` & HUD fault indicators)

## 2.0 — Validated training platform (Complete)

Goal: establish a versioned, evidence-backed configuration suitable for structured training and engineering studies.
Documented in dedicated IEEE Report: `to-publish/IEEE_TRAINING_PLATFORM_REPORT.md`.

Completed in 2.0:
- [x] Separate IEEE Paper initialized (`to-publish/IEEE_TRAINING_PLATFORM_REPORT.md` and brain artifact) for post-platform training and evaluation architecture
- [x] Standardized training scenario packs created (`src/scenarios.js`) with 4 canonical drills (Pipeline Survey, Benthic Habitat Docking, High-Shear Current Navigation, Emergency Thruster Recovery)
- [x] Objective Pilot Scoring Engine implemented (`src/scoringEngine.js`) calculating multi-criteria Flight Quality Metric (FQM $\mathcal{S} \in [0, 100]$ and Grades A-F) across corridor tracking, power, tether safety, collisions, and pace
- [x] Cockpit HUD live scoring section and dynamic 3D visual waypoint ring markers integrated in `src/main.js` and `index.html`
- [x] Automated test suite expanded to 43/43 passing tests (`test/training-platform.test.js`, `test/empirical-validation.test.js`)
- [x] Empirical profile validation against published BlueROV2 Heavy flume-tank and open-water datasets ($R^2 > 0.99$, frequency error $< 1.5\%$)
- [x] Instructor Station overlay and printable Flight Debriefing Report generator (`src/instructorStation.js`)
- [x] WCAG 2.1 AA accessibility (ARIA live regions for alarms) and internationalization dictionary (`src/i18n.js` with English and Norwegian Bokmål support)

This milestone must not be described as safety-critical or hardware-certified without an appropriate independent validation process.

## 2.1 — Umbilical Dynamics & Navigation Fusion (Complete)

Goal: replace single-catenary approximations and idealized odometry with high-fidelity multi-node lumped-mass cable physics and continuous-discrete Error-State Kalman Filter state estimation.
Documented in dedicated IEEE Report: `to-publish/IEEE_CABLE_NAVIGATION_REPORT.md`.

Completed in 2.1:
- [x] Separate IEEE Paper initialized (`to-publish/IEEE_CABLE_NAVIGATION_REPORT.md` and brain artifact) for umbilical cable dynamics and navigation fusion
- [x] $N$-Node Viscoelastic Lumped-Mass Umbilical Cable Model (`src/lumpedMassTether.js`) with 12 nodes, non-linear unilateral axial tension, viscoelastic damping, and Morison cross-flow/tangential drag
- [x] Benthic penalty contact floor model ($18.0\text{ m}$ seabed boundary) with normal viscoelastic penalty reactions and Coulomb friction
- [x] 15-State Error-State Extended Kalman Filter (ES-EKF) on $S^3 \times \mathbb{R}^{12}$ (`src/navigationFilter.js`) fusing 100 Hz IMU strapdown mechanization with multi-rate DVL, depth pressure, fluxgate compass, and low-rate USBL acoustic fixes
- [x] Automated test suite expanded to 51/51 passing tests (`test/cable-navigation.test.js`)
- [x] Integrated `LumpedMassTether` and `NavigationFilter` into physical simulation loop (`src/physics.js`)

## 2.2 — Advanced Intervention & Robotics (Future Horizon)

Goal: expand from observation/survey piloting into active subsea intervention and hardware-in-the-loop interoperability.

- Subsea robotic manipulator arm kinematics (1-DOF grabber to 4-DOF articulated arm) with body reaction wrench coupling
- Real-time WebRTC / WebSocket telemetry bridge for MAVLink / ROS 2 interoperability
- Multi-pilot cooperative intervention (dual-ROV tandem tether handling)
- Acoustic Doppler Current Profiler (ADCP) spatial velocity field reconstruction

## Maintenance backlog

- [x] Split Three.js and scene code into cacheable chunks to remove Vite bundle advisory (`vite.config.js` manualChunks)
- [x] Retired unused `one-person-submarine.mjs` legacy asset
- Move inline console markup into small tested view components if UI complexity grows
- Add release automation and generated artifact checksums
