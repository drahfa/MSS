# Changelog

All notable changes to the browser ROV simulator are documented here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow Semantic Versioning.

## [2.0.0-dev] - 2026-10-13

### Added

- Independent IEEE Technical Report for post-platform training development: `to-publish/IEEE_TRAINING_PLATFORM_REPORT.md` (covering empirical flume/tank grounding, declarative scenario architecture, objective pilot scoring engine, and instructor station controls).
- Declarative Training Scenario Architecture (`src/scenarios.js`):
  - Standardized training drills for subsea pilot qualification: Pipeline Survey, Benthic Habitat Docking, High-Shear Current Navigation, and Emergency Thruster Recovery.
  - Automated environmental current configuration, 3D waypoint chains, corridor tolerances, and in-flight fault triggers.
- Objective Pilot Scoring Engine (`src/scoringEngine.js`):
  - Multi-criteria Flight Quality Metric (FQM $\mathcal{S} \in [0, 100]$) and standardized Grades (A, B, C, F).
  - Continuous evaluation of RMS cross-track corridor deviation ($e_{\text{cte}}$), thruster energy and switching chatter, tether payout safety margins, and structural impact velocity.
  - Waypoint arrival detection ($1.35\text{ m}$ radius) with automatic progression tracking.
- Cockpit HUD Live Scoring Display in `index.html` and `src/main.js`:
  - Real-time score readout, proficiency grade, waypoint progress, RMS corridor error, and active infractions list.
  - Dynamic 3D visual waypoint ring markers rendered in the WebGL scene.
  - Cockpit scenario selector button cycling through drills with automatic environmental staging.
- Automated Training Platform Test Suite (`test/training-platform.test.js`):
  - 6 new automated verification tests covering scenario structure, geometric distance calculation, perfect-flight Grade A awards, corridor excursion penalties, collision/snag penalties, and waypoint progression.
  - Total automated test suite expanded to 39/39 passing tests with 0 failures.
- Vite build code-splitting configuration (`vite.config.js`):
  - Chunked `three` library into a standalone cacheable vendor asset, completely eliminating bundle size advisories.

## [1.3.0] - 2026-10-13
 
### Added
 
- Hardware Gamepad API pilot controller (`src/gamepad.js`):
  - Standard gamepad axis mapping (sway, surge, yaw, heave triggers) with configurable dead zones ($0.08$) and linear normalization.
  - Seamless auto-detection and fallback between keyboard and analog joystick control.
- Mission Route planning and safety validation system (`src/missionRoute.js`):
  - 3D waypoint creation, editing, and route traversal with proximity arrival detection.
  - Operational flight envelope safety validation checking surface limits and seabed clearance thresholds.
  - JSON serialization and deserialization for mission profile persistence.
- Telemetry logging, CSV/JSON export, and deterministic flight replay (`src/telemetryRecorder.js`):
  - Step-by-step state ($13$ states), pilot inputs, actuator allocations, and assist flags logging.
  - One-click CSV and JSON telemetry export from cockpit HUD.
  - Bit-exact deterministic flight replay from identical initial conditions with state divergence $< 10^{-4}$.
- Synthetic subsea instrument suite (`src/sensors.js`):
  - Doppler Velocity Log (DVL) bottom-track with altitude-dependent bottom-lock dropout ($< 0.3\text{ m}$ or $> 25\text{ m}$) and Gaussian measurement noise.
  - Hydrostatic pressure depth transducer converting hydrostatic head $p = \rho g d$ to bars with sensor bias drift.
  - Fluxgate magnetic compass including local geomagnetic declination and heading noise.
  - 6-axis Inertial Measurement Unit (IMU): angular rate gyros with random-walk bias drift and 3-axis accelerometer specific force $\mathbf{f}^b = \mathbf{a}^b - \mathbf{R}^T \mathbf{g}^n$.
  - Acoustic altimeter measuring direct vertical seabed clearance.
- Umbilical tether dynamics model (`src/tether.js`):
  - 3D catenary payout length calculation based on excursion distance from surface clump weight.
  - Cross-flow quadratic hydrodynamic line drag opposing ROV body motion.
  - Elastic restoring spring tension with advisory threshold ($88\%$ payout) and hard snag alerts ($> 100\%$ spool capacity).
- Benthic penalty contact and friction model (`src/contact.js`):
  - Elastic-viscous penalty spring-damper contact reactions ($k_c = 18{,}000\text{ N/m}$, $c_c = 1{,}500\text{ N}\cdot\text{s/m}$) for both horizontal seabed plane ($z = 18\text{ m}$) and cylindrical/spherical underwater habitat obstacles ($r = 3.5\text{ m}$).
  - Tangential Coulomb friction opposing sliding velocity.
- Thruster failure injection framework (`src/failureInjection.js`):
  - Real-time simulation of `OK`, `OFFLINE`, `DEGRADED`, and `STUCK` fault modes per channel.
  - Real-time allocator health weighting informing fault-tolerant allocation matrices.
  - Cockpit HUD fault injector button cycling faults dynamically during flight.
- Cockpit HUD enhancements in `index.html` and `src/main.js`:
  - Annunciator lights for Gamepad connection, Tether payout & snag warnings, and DVL bottom-lock status.
  - Telemetry record/export button generating timestamped CSV logs directly in the browser.
- Automated mission operations test suite in `test/mission-operations.test.js`:
  - 6 new automated verification tests covering deterministic replay, synthetic sensors, tether payout/snag, penalty contact dynamics, thruster failure injection, and mission route envelope validation.
  - Total test suite expanded to 33/33 tests passing with 0 failures.

## [1.2.0] - 2026-10-13

### Added

- External Vehicle Parameter Schema (`public/schemas/rov-profile.schema.json`) defining standard specifications for mass, inertia, 6x6 added mass and damping matrices, thruster geometry, force limits, deadbands, and reverse efficiencies.
- Standalone JSON vehicle profiles exported for all 5 ROV classes in `public/vehicles/{hobbyist,observation,inspection,light-work,heavy-work}.json`.
- Non-linear thruster mechanics in `src/physics.js`:
  - Deadband cutoff (`effectiveThrustRatio`) modeling static friction and ESC deadbands ($u \le 0.04 \implies 0$).
  - Asymmetric bollard pull efficiency ($75\%$ reverse efficiency) modeling ducted propeller reverse thrust loss.
  - Through-water propeller advance ratio ($J_a$) and axial inflow degradation (`advanceRatioFactor`).
- Vertical ocean current shear layer (`currentShear`) with depth-stratified velocity gradient and benthic boundary layer attenuation.
- Unit quaternion attitude kinematics on $S^3$ in `src/mss/library.js` and `src/mss/rovPlant.js`:
  - Added `euler2q`, `q2euler`, `Rquat`, `qdot`, and `normalizeQuat` routines.
  - Upgraded vehicle plant state from 12-state Euler representation to a 13-state unit quaternion vector $[\boldsymbol{\nu}, \mathbf{p}^n, \mathbf{q}]$ with continuous renormalization on $S^3$.
  - Completely eliminated Euler gimbal lock singularities at $\theta = \pm 90^\circ$, removing hard pitch angle limits and enabling full $360^\circ$ tumbling and acrobatic maneuvering.
  - Maintained human-readable Euler angle conversions for the cockpit HUD artificial horizon, pitch bar, and compass dials.
- Real-time parameter schema validator and exporter module (`src/mss/profileValidator.js`).
- Interactive Parameter Import / Export modal dialog in `index.html` and `src/main.js`:
  - Real-time schema validation with instant error reports and diagnostic summary.
  - One-click JSON export of active profiles.
  - File picker and direct text input for importing custom vehicle profiles.
  - Dynamic hot-reloading of coupled $6 \times 6$ plant and thruster allocation without page reloads.
- Standard Operating Procedure (SOP) calibration guide in `CALIBRATION_WORKFLOW.md` documenting parameter extraction from dry-dock weigh scales, bifilar torsional pendulums, bollard pull test benches, tow-tank drag tests, and inclinometer free-decay logs.
- Hydrodynamic free-decay, step-response, and Lyapunov passivity verification in `test/hydrodynamics.test.js`:
  - Verified pitch and roll free-decay damped oscillations against analytical frequency $\omega_d$ and damping ratio $\zeta$ ($< 0.9\%$ period error across all 5 classes).
  - Validated coupled $6 \times 6$ mass acceleration $\dot{u}(0) = (\mathbf{M}^{-1})_{11}\tau_u$ and asymptotic convergence to closed-form terminal velocity $u_\infty$ ($< 1.3\%$ error).
  - Proved unforced mechanical energy monotonic dissipation $\dot{E} \le 0$ (zero non-decreasing steps, $>98\%$ energy dissipated over 12 seconds).
- Automated test suite expanded to 27/27 unit tests passing across kinematics, quaternions, propulsion, schema, hydrodynamics, and hot-loading.
- Comprehensive IEEE Transactions-style technical paper and architectural specification expanded in `to-publish/IEEE_TECHNICAL_REPORT.md` with five benchmark case studies, Table V quantitative free-decay metrics, and live calibration workflows.

## [1.1.0] - 2026-10-13

### Added

- Exact line-for-line ports of MSS MATLAB `LIBRARY` routines into `src/mss/library.js`:
  - Kinematics: `Smtrx`, `Hmtrx`, `Rzyx`, `Tzyx`, `eulerang`, `ssa`.
  - Kinetics & Restoring: `rbody` (with $H(r_{bG})$ rigid-body lever arm transformation), `m2c`, `gRvect`, `mssConstants`, `sat`.
  - Solvers: Full $6 \times 6$ `mldivide` solver (Gaussian elimination with partial pivoting), `inv`, and `rk4`.
- Pure unconstrained MSS 6-DOF plant implementation in `src/mss/rovPlant.js` mirroring `hydroVessel.m` and `remus100.m`.
- Automated test suite using Node's native test runner (`npm test`):
  - `test/mss-library.test.js`: Validates skew-symmetry, $SO(3)$ orthogonality, matrix inversions, and harmonic oscillator RK4 integration.
  - `test/allocation.test.js`: Validates $[-1, 1]$ command saturation and physical directional authority for pure-axis demand across all 5 ROV profiles.
  - `test/glb-metadata.test.js`: Validates that all 5 GLB model files have rotor node counts and `channel_index` extras strictly matching `ROV_PROFILES`.
  - `test/rov-plant.test.js`: Validates symmetric positive definite system matrices across all 5 ROV profiles and verifies 60-second 6-DOF dynamic stability and damping.

### Changed

- Refactored `src/physics.js` to run the coupled unconstrained MSS plant via `rk4`, separating environmental boundary clamps and control logic from vehicle dynamics.
- Switched angle wrapping to MSS `ssa` (smallest signed angle) semantics.

### Fixed

- Corrected Blender model port and starboard thruster channel indexing across all 5 ROV classes in `build_rov_fleet.py` to match `ROV_PROFILES` with the runtime $180^\circ$ yaw orientation.
- Regenerated all browser GLBs (`public/models/rov-*.glb`) and updated the canonical `rov-fleet.blend`.
- Stopped propeller rotor animation while simulation is paused.
- Adjusted water mote drift in `src/main.js` to travel along the ambient current vector $(\sin\beta_{Vc}, -\cos\beta_{Vc})$ instead of fixed $+X$.
- Corrected inverted FPV camera look-at pitch direction in `src/main.js`.
- Reset trail path history properly on vehicle reset by resetting `lastTrailTime`.
- Replaced hard-coded `/models/` URLs with Vite's `import.meta.env.BASE_URL` to support arbitrary deployment base paths.

## [1.0.0] - 2026-10-13

### Added

- Vite and Three.js browser application under `threeJS`
- MSS-inspired nonlinear 6-DOF plant with current-relative damping, Coriolis terms, hydrostatic restoring forces, and fixed-step RK4 integration
- Five selectable ROV classes: Hobbyist, Observation, Inspection, Light work, and Heavy work
- Class-specific mass, inertia, added mass, damping, center-of-gravity offset, actuator response, and camera distance
- Per-class thruster counts, positions, directions, force limits, and damped least-squares control allocation
- Procedural Blender fleet source, repeatable export script, editable `.blend`, and browser-ready GLBs
- Visible four-blade propellers, open ducts, exported channel metadata, and physics-driven rotor animation
- Polyfork underwater research habitat as the mission target
- Third-person chase, top-down, and FPV camera modes
- Six-leg autonomous habitat inspection demo toggled with `~`
- Assist modes for depth, heading, and attitude within each vehicle's available control authority
- Operator console with depth, attitude, heading, velocity, current, depth history, and adaptive thruster channels
- Responsive layouts at 900 px and 580 px breakpoints
- Keyboard focus treatment, ARIA labels, button pressed states, and reduced-motion support

### Fixed

- Propellers now rotate around the exported GLB shaft axis after Blender Z-up to glTF Y-up conversion
- Horizontal and vertical rotor orientation is preserved when composing animation rotations
- Visible model thruster counts now match their physics profiles and telemetry channels
- Vertical propeller discs and flow paths no longer intersect yellow buoyancy or pressure-body geometry
- Heavy-work vertical thrusters use outboard steel mounts instead of sitting beneath buoyancy blocks
- Hobbyist lift flow passes through a center tunnel between twin buoyancy lobes

### Known limitations

- Vehicle coefficients are representative and not validated against production hardware
- No tether, contact, manipulator, sensor, cavitation, or propeller-wake dynamics
- No automated test suite or continuous-integration workflow yet

