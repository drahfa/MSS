# MSS ROV simulator

An interactive browser simulator for remotely operated underwater vehicles, built from the Marine Systems Simulator (MSS) 6-DOF formulation. It combines a nonlinear vehicle plant, class-specific thruster allocation, five Blender-authored ROVs, and an operator console rendered with Three.js.

The simulator is useful for dynamics demonstrations and interface prototyping. Its vehicle parameters are representative and have not been validated against a specific production ROV.

## Quick start

Requirements:

- Node.js `^20.19.0` or `>=22.12.0`
- A current browser with WebGL support
- Blender 4.2 LTS or newer only when rebuilding the ROV models

From this directory:

```powershell
npm ci
npm run dev
```

Open the local address printed by Vite. The default is `http://127.0.0.1:5173`.

Create and inspect a production build with:

```powershell
npm run build
npm run preview
```

## Controls

| Input | Action |
| --- | --- |
| `W` / `S` | Forward / reverse surge |
| `A` / `D` | Port / starboard sway where the selected allocation supports it |
| `R` / `F` | Rise / dive |
| `Q` / `E` | Yaw left / right |
| `C` | Cycle Third person, Top down, and FPV cameras |
| `~` | Enter or exit the autonomous habitat demo |
| `Space` | Pause or resume |
| `X` | Reset the vehicle |

The Assist control holds depth, heading, and attitude within the authority available from the selected thruster layout. Small vehicles are intentionally under-actuated, so unsupported axes are not synthesized by invisible motors.

## What is included

- Pure MSS 6-DOF plant formulation (`src/mss/rovPlant.js`) and core mathematics library (`src/mss/library.js`)
- Nonlinear propulsion dynamics with thruster deadband thresholding and asymmetric forward/reverse bollard pull curves
- External JSON parameter schema (`public/schemas/rov-profile.schema.json`) and modular vehicle definitions (`public/vehicles/*.json`)
- Current-relative velocity, irrotational current acceleration corrections, Coriolis, damping, buoyancy, and restoring forces
- Fixed-step fourth-order Runge-Kutta integration
- Per-class damped least-squares thruster allocation and actuator lag
- Hobbyist, Observation, Inspection, Light work, and Heavy work ROVs
- Animated, channel-matched propellers with unobstructed vertical flow paths
- Third-person chase, top-down, and vehicle FPV cameras
- Autonomous six-leg inspection demo, enabled with `~`
- Depth history, attitude, heading, velocity, current, and dynamic thruster telemetry
- Responsive operator-console layout based on a subsea flight-screen visual language
- IEEE Transactions-style technical paper and architecture specification in [`to-publish/IEEE_TECHNICAL_REPORT.md`](../to-publish/IEEE_TECHNICAL_REPORT.md)

## System overview

```text
keyboard / demo route
        |
        v
assist controllers ---- selected ROV profile
        |                 | mass, damping, thrusters
        v                 v
6-DOF demand ------> control allocator
                          |
                          v
                 actuator lag + saturation
                          |
                          v
             MSS-inspired nonlinear plant
                          |
                          v
          Three.js scene + telemetry console
```

The render loop uses a variable display timestep while the dynamics accumulator advances in fixed `0.01 s` RK4 steps.

## ROV fleet

The models are original, manufacturer-neutral interpretations of the broad class silhouettes in the [ROV class reference sheet](../references/typesOfROV.jpeg).

| Class | Thruster allocation | Force limit per unit | Simulated mass |
| --- | --- | --- | ---: |
| Hobbyist | 2 horizontal + 1 center vertical | 5 N / 6 N | 4.2 kg |
| Observation | 2 horizontal + 2 vertical | 18 N / 21 N | 22 kg |
| Inspection | 4 vectored horizontal + 4 vertical | 65 N / 72 N | 85 kg |
| Light work | 4 vectored horizontal + 2 vertical | 420 N / 470 N | 650 kg |
| Heavy work | 4 vectored horizontal + 4 vertical | 2,100 N / 2,350 N | 4,200 kg |

Each channel in `ROV_PROFILES` defines an identifier, operator label, body-fixed position, unit thrust direction, and bidirectional force limit. The same channel index is exported in the corresponding GLB and drives its visible propeller.

### Rebuild the models

The editable source is `blender/rov-fleet.blend`. The deterministic build script recreates and exports the fleet:

```powershell
blender --background --python blender/build_rov_fleet.py
```

Generated browser models are written to `public/models/rov-*.glb`. `rov-fleet.blend1` is Blender's automatic backup and is not the canonical source.

When changing a thruster, update both its Blender channel in `build_rov_fleet.py` and its physical definition in `src/physics.js`. See [HANDOFF.md](HANDOFF.md) for the synchronization checklist.

## Dynamics lineage

The implementation in `src/physics.js` follows the state and sign conventions used by MSS `CRAFT/AUV/models/remus100.m` and `CRAFT/hydroVessel.m`:

- State: `x = [u v w p q r north east down phi theta psi]`
- Body frame: front-starboard-down
- Navigation frame: North-East-Down
- Relative velocity: `nu_r = nu - nu_c`
- Plant: `M nu_dot + C(nu) nu + D(nu) nu + g(eta) = tau`
- Rigid-body and added-mass Coriolis terms based on `rbody.m` and `m2c.m`
- Hydrostatic restoring terms based on `gRvect.m`
- ZYX Euler kinematics based on `eulerang.m`
- RK4 integration based on `rk4.m`

The control allocator constructs a physical wrench column `[force; position x force]` for every thruster. It normalizes available authority by axis, solves a damped least-squares inverse, and scales saturated commands into `[-1, 1]`.

## Coordinate mapping

MSS navigation coordinates are mapped into Three.js as:

```text
Three X = East
Three Y = -Down
Three Z = -North
```

Blender authors models Z-up and exports glTF Y-up. Propeller blades are authored in the rotor's local XY plane; after export, the shaft is the GLB node's local Y axis. Preserve this distinction when changing rotor animation.

## Assets

The underwater habitat is a Polyfork asset stored locally as `src/assets/underwater-research-habitat.mjs`, allowing Vite to resolve its Three.js imports and produce a self-contained build.

`src/assets/one-person-submarine.mjs` is a retained legacy Polyfork asset and is not loaded by the current simulator. Active vehicle visuals come from `public/models/rov-*.glb`.

## Project structure

```text
threeJS/
|-- blender/
|   |-- build_rov_fleet.py   # Procedural fleet generator and GLB exporter
|   `-- rov-fleet.blend      # Editable canonical Blender scene
|-- public/models/           # Runtime GLB files
|-- src/
|   |-- assets/              # Locally stored Polyfork modules
|   |-- main.js              # Scene, controls, cameras, demo, and telemetry
|   |-- physics.js           # Profiles, allocation, and 6-DOF dynamics
|   `-- style.css            # Console design system and responsive layout
|-- index.html               # Accessible console structure
|-- CHANGELOG.md
|-- HANDOFF.md
|-- ROADMAP.md
`-- package.json
```

## Verification

Run the automated test suite:

```powershell
npm test
```

Create and verify a production build:

```powershell
npm run build
```

Also exercise the following in a browser:

1. Select all five vehicle classes and confirm the model and channel count change together.
2. Command forward, reverse, depth, and yaw; confirm the matching propellers rotate in the correct direction and around their shaft.
3. Check vertical propellers from Top down and Third person views for clear rotor discs.
4. Toggle Assist, Trail, Pause, and the `~` demo.
5. Check the layout above 900 px, below 900 px, and below 580 px.

## Known limitations

- Mass, inertia, hydrodynamic coefficients, and force limits are representative rather than vehicle-certified data.
- Ambient-current flow is implemented through relative velocity and hydrodynamic damping. Propeller wake, advance ratio, cavitation, and thruster-thruster interaction are not modeled.
- Euler angles are retained for readability; this is not a high-attitude maneuvering model.
- Tether forces, seabed contact, collision response, manipulator physics, and sensor noise are not simulated.
- Vite reports a bundle-size advisory because Three.js and the scene ship in one JavaScript chunk.

See [ROADMAP.md](ROADMAP.md) for planned verification and fidelity work.

## Documentation

- [CHANGELOG.md](CHANGELOG.md) — released behavior and fixes
- [ROADMAP.md](ROADMAP.md) — prioritized future work
- [HANDOFF.md](HANDOFF.md) — engineering, design, and maintenance specifications

## License

This project is part of MSS and follows the repository's root [MIT License](../LICENSE).
