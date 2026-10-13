# Changelog

All notable changes to the browser ROV simulator are documented here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow Semantic Versioning.

## [1.2.0] - In Progress

### Added

- External Vehicle Parameter Schema (`public/schemas/rov-profile.schema.json`) defining standard specifications for mass, inertia, 6x6 added mass and damping matrices, thruster geometry, force limits, deadbands, and reverse efficiencies.
- Standalone JSON vehicle profiles exported for all 5 ROV classes in `public/vehicles/{hobbyist,observation,inspection,light-work,heavy-work}.json`.
- Non-linear thruster mechanics in `src/physics.js`:
  - Deadband cutoff (`effectiveThrustRatio`) modeling static friction and ESC deadbands ($u \le 0.04 \implies 0$).
  - Asymmetric bollard pull efficiency ($75\%$ reverse efficiency) modeling ducted propeller reverse thrust loss.
- Unit quaternion attitude kinematics on $S^3$ in `src/mss/library.js` and `src/mss/rovPlant.js`:
  - Added `euler2q`, `q2euler`, `Rquat`, `qdot`, and `normalizeQuat` routines.
  - Upgraded vehicle plant state from 12-state Euler representation to a 13-state unit quaternion vector $[\boldsymbol{\nu}, \mathbf{p}^n, \mathbf{q}]$ with continuous renormalization on $S^3$.
  - Completely eliminated Euler gimbal lock singularities at $\theta = \pm 90^\circ$, removing hard pitch angle limits and enabling full $360^\circ$ tumbling and acrobatic maneuvering.
  - Maintained human-readable Euler angle conversions for the cockpit HUD artificial horizon, pitch bar, and compass dials.
- Unit test suite expanded in `test/mss-library.test.js` covering unit quaternion round-trips, direction cosine orthogonality, and singularity-free RK4 rotation past $90^\circ$ (20/20 tests passing).
- Comprehensive IEEE Transactions-style technical paper and architectural specification expanded in `to-publish/IEEE_TECHNICAL_REPORT.md` with in-depth derivations of local inflow velocity fields, depth-stratified current transport acceleration, and Case Study 4 evaluating quaternion stability across steep vertical pitch.

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

