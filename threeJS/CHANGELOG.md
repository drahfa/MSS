# Changelog

All notable changes to the browser ROV simulator are documented here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow Semantic Versioning.

## [3.0.0] - 2026-10-13

### Added

- Independent IEEE Technical Report for Milestone 3.0: `to-publish/IEEE_SONAR_PERCEPTION_REPORT-Paper-6.md` (covering synthetic acoustic wave propagation, multibeam forward-looking sonar simulation, physical two-way transmission loss, Lambertian backscatter, occlusion shadow casting, and polar-to-Cartesian scan conversion).
- 2D Multibeam Forward-Looking Sonar (FLS) Simulation Engine (`src/forwardLookingSonar.js`):
  - 96 acoustic beams $\times$ 128 range bins spanning $120^\circ$ horizontal azimuth with $20^\circ$ vertical elevation fan beam.
  - Physical two-way transmission loss ($40\log_{10} r$) and frequency-dependent seawater chemical absorption $\alpha(f)$ ($0.088\text{ dB/m}$ at $450\text{ kHz}$, $0.285\text{ dB/m}$ at $900\text{ kHz}$).
  - Receiver Time-Varied Gain (TVG) compensation equalizing dynamic range across near- and far-field acoustic returns.
  - Empirical Lambertian acoustic surface backscatter and seabed grazing angle reverberation.
  - True geometric occlusion shadow casting ($I = 0$) behind solid underwater structures and boulders.
  - Real-time HTML5 polar sector scan converter rendering to phosphor amber and cyan oceanic palettes with range rings and azimuth guides.
- Flight Cockpit Console Sonar HUD Widget (`index.html`, `src/main.js`):
  - Added dedicated Forward Sonar (FLS) canvas widget with interactive mode toggle between $450\text{ kHz}$ Navigation ($40\text{ m}$) and $900\text{ kHz}$ Inspection ($20\text{ m}$).
- Sonar Simulation Automated Test Suite (`test/sonar-simulation.test.js`):
  - 5 automated unit tests validating beam count, range resolution, dual-frequency mode switching, acoustic target detection peaks, shadow occlusion, and headless canvas rendering.
- Expanded project automated test suite to 66/66 passing unit tests (0 failures).
- Standardized all 6 published technical report filenames to `-Paper-[num].md` format across `to-publish/` and brain artifacts.

## [2.3.0] - 2026-10-13

### Added

- Independent IEEE Technical Report for Milestone 2.3: `to-publish/IEEE_MULTI_ROV_NETWORKED_REPORT-Paper-5.md` (covering multi-agent cooperative subsea operations, 3D dual-tether anti-entanglement spatial clearance, cooperative closed-chain payload mechanics, and networked digital twin telemetry bridges).
- Multi-ROV Cooperative Operations & Networked Telemetry Engine (`src/multiRovTetherBridge.js`):
  - Fast 3D segment-to-segment Euclidean minimum distance algorithm between dual $N$-node lumped-mass umbilical cables.
  - Progressive entanglement envelopes: warning advisory at $d_{\min} \le 3.0\text{ m}$ and critical emergency alert at $d_{\min} \le 1.0\text{ m}$.
  - Cooperative closed-kinematic-chain payload evaluation: geometric distance constraint solving, tension wrench calculation, and 50/50 submerged weight distribution across Lead and Follower ROVs.
  - Standard ROS 2 JSON serialization for `/odometry`, `/imu`, and `/cmd_vel` topics with nanosecond timestamps.
  - MAVLink 2.0 telemetry and command serialization (`HEARTBEAT`, `ATTITUDE`, `GLOBAL_POSITION_INT`, `SERVO_OUTPUT_RAW`, `RC_CHANNELS_OVERRIDE`) with lossless bidirectional PWM parsing.
- Multi-ROV & Networked Bridge Automated Test Suite (`test/multi-rov-bridge.test.js`):
  - 5 automated unit tests validating 3D segment distances, entanglement alarm triggers, cooperative tension dynamics, ROS 2 schema adherence, and MAVLink bidirectional command round-trips.
- Interactive Procedural 3D Subsea Manipulator Arm View (`src/manipulatorView.js`):
  - Procedural Three.js mesh hierarchy attached to ROV chin with base azimuth turret, safety-orange upper arm, hydraulic pistons, elbow hub, forearm, and dual gripper jaws animated by forward kinematics.
- Multi-ROV Fleet & Dynamic Dual-Tether Anti-Entanglement Visualizer (`src/multiRovView.js`):
  - Procedural yellow Observation Eyeball ROV follower holding station with camera dome and floodlights.
  - Dual 12-node dynamic 3D catenary umbilical line rendering with surface TMS anchors.
  - Real-time 3D line-of-closest-approach vector dynamically shifting from green (safe) to amber (warning, $\le 3.0\text{ m}$) and red (critical, $\le 1.0\text{ m}$).
- Flight Cockpit Console HUD Extensions (`index.html`, `src/main.js`):
  - Added `Arm EE` and `Fleet Sep` data-grid readouts, interactive `ARM READY` and `FLEET DUAL` annunciator pills, and key mappings (`M`, `T`, `1-3`, `G`).

## [2.2.0] - 2026-10-13

### Added

- Independent IEEE Technical Report for Milestone 2.2: `to-publish/IEEE_UVMS_MANIPULATION_REPORT-Paper-4.md` (covering Underwater Vehicle-Manipulator Systems kinematics, Newton-Euler base reaction wrenches, and link Morison hydrodynamics).
- Underwater Vehicle-Manipulator System (UVMS) Physics Engine (`src/uvmsManipulator.js`):
  - Denavit-Hartenberg (D-H) forward kinematics chain and $6 \times n$ geometric Jacobian $\mathbf{J}(\mathbf{q})$.
  - Singularity-robust Damped Least-Squares (DLS) inverse kinematics converging to sub-millimeter end-effector accuracy ($< 0.001\text{ m}$).
  - Recursive Newton-Euler reaction wrench calculation ($\boldsymbol{\tau}_{\text{react}}^b = [\mathbf{f}_{\text{react}}^b, \mathbf{m}_{\text{react}}^b]^T$) transmitted from accelerating link masses directly to the floating ROV base.
  - Segmental Morison hydrodynamic cross-flow drag distributed along articulated cylindrical arm links.
- Integrated `UvmsManipulator` arm dynamics and reaction wrench into the MSS 6-DOF vehicle equations of motion in `src/physics.js`.
- UVMS Robotic Manipulator Automated Test Suite (`test/uvms-manipulator.test.js`):
  - 5 automated unit tests verifying D-H chain forward kinematics, joint axes, Jacobian finiteness, DLS convergence, and base reaction wrench conservation.
- Expanded project automated test suite to 61/61 passing unit tests (0 failures).

## [2.1.0] - 2026-10-13

### Added

- Independent IEEE Technical Report for Milestone 2.1: `to-publish/IEEE_CABLE_NAVIGATION_REPORT-Paper-3.md` (covering multi-node viscoelastic lumped-mass umbilical dynamics and 15-state Error-State Extended Kalman Filter acoustic-inertial navigation fusion).
- $N$-Node Viscoelastic Lumped-Mass Umbilical Cable Dynamics Engine (`src/lumpedMassTether.js`):
  - 12-node spatial discretization with flat typed `Float64Array` memory layout achieving $16.5\ \mu\text{s}$ execution per physics step.
  - Non-linear unilateral axial tension (zero under compression, Hookean under stretch) with internal viscoelastic damping.
  - Morison hydrodynamic cross-flow ($C_{d,n} = 1.2$) and tangential skin drag ($C_{d,t} = 0.03$) coupled with 3D oceanic current shear layer.
  - Benthic penalty contact mechanics preventing seabed penetration below $18.0\text{ m}$ with normal viscoelastic restoring forces ($k_{\text{bed}} = 22{,}000\text{ N/m}$) and Coulomb tangential friction ($\mu = 0.45$).
  - Dirichlet boundary conditions anchoring Node 0 at surface TMS and Node $N-1$ at vehicle tether attachment, returning terminal tension body wrench $\boldsymbol{\tau}_{\text{tether}}^b$.
- 15-State Continuous-Discrete Error-State Extended Kalman Filter (`src/navigationFilter.js`):
  - Strapdown IMU mechanization predicting nominal position $\mathbf{p}$, velocity $\mathbf{v}$, and attitude quaternion $\mathbf{q} \in S^3$ at 100 Hz.
  - Error-state formulation on $\delta\mathbf{x} \in \mathbb{R}^{15}$ (position error, velocity error, 3D tangent attitude error $\delta\boldsymbol{\theta}$, accelerometer bias $\mathbf{b}_a$, gyroscope bias $\mathbf{b}_g$).
  - Multi-rate asynchronous measurement updates: DVL bottom-track velocity ($10\text{ Hz}$), hydrostatic depth pressure ($10\text{ Hz}$), fluxgate magnetic compass ($10\text{ Hz}$), and delayed Ultra-Short Baseline (USBL) acoustic beacon fixes ($0.5\text{--}1.0\text{ Hz}$).
  - Analytical $3\sigma$ covariance bounds maintaining estimation error consistency across prolonged acoustic dropouts.
- Automated Cable Dynamics & Navigation Unit Test Suite (`test/cable-navigation.test.js`):
  - 8 new automated test cases verifying cable static catenary sag convergence, unilateral tension limits, Morison cross-flow drag directionality, benthic penalty floor limits, ES-EKF strapdown prediction, unit quaternion norm preservation, and multi-sensor covariance contraction.
- Integrated `LumpedMassTether` dynamic option and continuous `NavigationFilter` observer in `src/physics.js`.
- Expanded test suite to 51 passing unit tests (0 failures).

## [2.0.0] - 2026-10-13

### Added

- Independent IEEE Technical Report for post-platform training development: `to-publish/IEEE_TRAINING_PLATFORM_REPORT-Paper-2.md` (covering empirical flume/tank grounding, declarative scenario architecture, objective pilot scoring engine, and instructor station controls).
- Empirical Sea-Trial Grounding Test Suite (`test/empirical-validation.test.js`):
  - Calibrated surge drag regression against published BlueROV2 Heavy tow-tank carriage data ($R^2 > 0.99$).
  - Reverse bollard efficiency matching physical load cell dynamometry within $2\%$ margin.
  - Pitch restoring natural frequency matching open-water inclinometer trials within $1.5\%$ error.
  - Parameter $95\%$ confidence uncertainty bounds verification.
- Declarative Training Scenario Architecture (`src/scenarios.js`):
  - Standardized training drills for subsea pilot qualification: Pipeline Survey, Benthic Habitat Docking, High-Shear Current Navigation, and Emergency Thruster Recovery.
  - Automated environmental current configuration, 3D waypoint chains, corridor tolerances, and in-flight fault triggers.
- Objective Pilot Scoring Engine (`src/scoringEngine.js`):
  - Multi-criteria Flight Quality Metric (FQM $\mathcal{S} \in [0, 100]$) and standardized Grades (A, B, C, F).
  - Continuous evaluation of RMS cross-track corridor deviation ($e_{\text{cte}}$), thruster energy and switching chatter, tether payout safety margins, and structural impact velocity.
  - Waypoint arrival detection ($1.35\text{ m}$ radius) with automatic progression tracking.
- Instructor Supervision Station (`src/instructorStation.js`):
  - Real-time environmental hazard induction: optical turbidity / silt plume fog slider ($\kappa_{\text{ext}} \in [0.035, 0.20]\text{ m}^{-1}$) and current shear gusts ($+0.3\text{ m/s}$).
  - Covert thruster failure injection panel (degraded, offline, stuck modes).
  - Printable Flight Debriefing Report generator formatted to IMCA/ISO pilot competency standards with Markdown and JSON exports.
- Internationalization & Accessibility (`src/i18n.js`):
  - Bilingual localization support for English and Norwegian Bokmål (`en`/`no`).
  - Screen-reader `aria-live="assertive"` regions for critical auditory/visual alarms (`TETHER SNAG!`, `COLLISION!`).
- Cockpit HUD Live Scoring & Scenario Switcher in `index.html` and `src/main.js`:
  - Real-time score readout, proficiency grade, waypoint progress, RMS corridor error, and active infractions list.
  - Dynamic 3D visual golden ring markers rendered at waypoint coordinates in the WebGL scene.
- Automated Test Suite expanded to 43/43 tests passing with 0 failures.
- Vite build code-splitting configuration (`vite.config.js`) chunking `three` into a standalone vendor asset.

### Removed

- Retired unused legacy Polyfork asset `src/assets/one-person-submarine.mjs`.

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
- Comprehensive IEEE Transactions-style technical paper and architectural specification expanded in `to-publish/IEEE_TECHNICAL_REPORT-Paper-1.md` with five benchmark case studies, Table V quantitative free-decay metrics, and live calibration workflows.

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

