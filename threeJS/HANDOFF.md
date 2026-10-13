# ROV simulator handoff

This document is the implementation contract for engineers and designers extending `threeJS`. Start with [README.md](README.md) to run the application and [ROADMAP.md](ROADMAP.md) for priorities.

## Product intent

The primary job is to make class-dependent underwater vehicle behavior visible and operable in a browser. The interface uses the visual language of a subsea flight screen: persistent state, restrained cyan instrumentation, amber command cues, and a large live vehicle view.

This is an educational and prototyping simulator, not a certified piloting or vehicle-design tool.

## Architecture ownership

| Area | Canonical source | Responsibilities |
| --- | --- | --- |
| Vehicle dynamics | `src/physics.js` | Profiles, force limits, allocation, equations of motion, integration |
| Runtime orchestration | `src/main.js` | Input, demo route, cameras, GLB loading, animation, telemetry updates |
| Console structure | `index.html` | Semantic regions, labels, controls, instrument markup |
| Visual system | `src/style.css` | Tokens, layout, states, breakpoints, focus and reduced motion |
| Fleet geometry | `blender/build_rov_fleet.py` | Procedural meshes, thruster placement, metadata, GLB export |
| Editable 3D source | `blender/rov-fleet.blend` | Generated canonical Blender scene for inspection and manual refinement |
| Runtime models | `public/models/rov-*.glb` | Browser-delivered artifacts; do not edit directly |
| Mission environment | `src/assets/underwater-research-habitat.mjs` | Polyfork habitat factory |

## Runtime data flow

1. Keyboard input or the demo route produces normalized surge, sway, heave, and yaw demand.
2. Assist controllers add depth, heading, roll, and pitch corrections.
3. The selected `ROV_PROFILES` entry supplies mass properties and physical thruster columns.
4. The damped least-squares allocator maps six-axis demand onto available channels.
5. First-order actuator lag and `[-1, 1]` saturation produce channel commands.
6. `RovDynamics.step()` advances the 12-state plant in fixed `0.01 s` RK4 increments.
7. `main.js` maps North-East-Down state into Three.js, animates indexed rotors, and refreshes telemetry at approximately 12.5 Hz.

## Vehicle and thruster contract

Every profile thruster has this shape:

```js
{
  id: 'H1',
  label: 'Front port vector',
  r: [front, starboard, down],
  f: [front, starboard, down],
  force: 65,
  reverseEfficiency: 0.75, // optional, defaults to 0.75 (75% bollard pull in reverse)
  deadband: 0.04,          // optional, defaults to 0.04 (4% deadband threshold)
}
```

- `r` is a profile-relative body-fixed position multiplied by the profile's `leverScale`.
- `f` is a unit direction in the front-starboard-down body frame.
- `force` is the nominal forward bollard pull limit in newtons.
- `reverseEfficiency` scales negative thrust commands (default 0.75) to model ducted propeller reverse thrust loss.
- `deadband` filters commands below $|u| \le 0.04$ to emulate static friction and ESC deadband.
- Array order is the channel order used by dynamics, telemetry, and GLB rotor metadata.
- `id` should remain short enough for the 310 px telemetry rail; use `H1`–`H4` and `V1`–`V4` unless a different functional family is required.
- `label` should describe location and function in plain operator language.
- Profiles can be externalized and validated against `public/schemas/rov-profile.schema.json`. See standalone vehicle profiles under `public/vehicles/`.
- For formal mathematical derivations and lineage, refer to `to-publish/IEEE_TECHNICAL_REPORT.md`.

Under-actuated vehicles are valid. Do not add hidden forces to make every keyboard axis work. An unavailable allocation row should remain unavailable and be handled by the damped inverse and passive restoring terms.

### Synchronizing a thruster change

1. Update the profile entry in `src/physics.js`.
2. Update the corresponding `thruster()` call in `blender/build_rov_fleet.py`.
3. Keep the zero-based Blender `channel` equal to the profile array index.
4. Ensure the entire propeller disc and its inlet/outlet column clear buoyancy, frame, and tooling geometry.
5. Regenerate the fleet with Blender.
6. Run `npm run build`.
7. Select every class in the browser and compare visible rotors with telemetry channels.
8. Test positive and negative commands; rotor direction and amber reverse telemetry must agree.
9. Inspect vertical rotors in Top down and Third person cameras.

The exporter writes `channel_index`, Blender-axis metadata, and glTF-axis metadata into node extras. `main.js` sorts rotor nodes by `channel_index` before matching them to `sim.thrusters`.

## Coordinate and rotor conventions

### MSS and Three.js

| Quantity | MSS | Three.js |
| --- | --- | --- |
| Longitudinal navigation | North | `-Z` |
| Lateral navigation | East | `+X` |
| Vertical navigation | Down | `-Y` |
| Body axes | Front, starboard, down | Converted in `updateVisuals()` |

### Blender and glTF

- Blender scene: Z-up, vehicle nose authored toward $-Y$.
- glTF/Three.js scene: Y-up, vehicle nose points North ($-Z$) after applying a $180^\circ$ runtime Y rotation.
- Blender $+X$ is vehicle port, and $-X$ is vehicle starboard to account for this $180^\circ$ runtime orientation.
- Propeller geometry is authored in the Blender rotor's local XY plane.
- Blender local Z is the authored shaft.
- After the export conversion, the runtime rotor shaft is local Y.
- Rotate with `rotateOnAxis(rotorShaftAxis, angle)`; do not increment an imported Euler component.

## Dynamics notes

- State order is `[u, v, w, p, q, r, north, east, down, phi, theta, psi]`.
- System mass is a fully coupled $6 \times 6$ matrix solved using Gaussian elimination (`mldivide`).
- Current is transformed into the body frame and subtracted before Coriolis and damping calculations.
- The allocator uses physical wrench columns and row authority normalization before applying a damped pseudoinverse.
- The dynamics timestep is fixed; do not replace it with display-frame `dt` without revalidating stability.
- Depth is clamped to the current scene envelope and pitch is constrained to avoid the Euler singularity.

When changing coefficients, document provenance and units. Representative guesses should remain explicitly labeled as such.

## MSS compliance boundary

The current JavaScript plant is an MSS-inspired reduced-order implementation. It is not a 100% numerical port of `hydroVessel.m`, `remus100.m`, or any other single MSS vehicle model. Product copy, code comments, and releases must not claim full MSS compliance until the validation gate below is satisfied.

| Area | Current parity | Implementation |
| --- | --- | --- |
| State order and frames | Complete | Uses MSS 12-state order `[u v w p q r n e d phi theta psi]` with FSD body axes and NED navigation axes |
| RK4 | Complete | MSS `rk4.m` ported line-for-line in `src/mss/library.js` |
| Euler kinematics | Complete | MSS `eulerang.m`, `Rzyx.m`, `Tzyx.m` in `src/mss/library.js` |
| Restoring vector | Complete | MSS `gRvect.m` with weight and buoyancy moments about CO |
| Relative current | Complete | Planar irrotational current with complete `nu_c_dot = [-Smtrx(omega) * v_c; 0]` derivative |
| Rigid-body Coriolis & Mass | Complete | MSS `rbody.m` transforming `MRB` and `CRB` using the CG lever arm $H(r_{bG})$ |
| Added-mass Coriolis | Complete | MSS `m2c.m` 6-DOF formulation |
| Mass solve | Complete | Full coupled $6 \times 6$ solve $M \setminus \text{RHS}$ using Gaussian elimination with partial pivoting (`mldivide`) |
| Plant separation | Complete | Pure unconstrained plant `rovPlant` isolated from browser clamps and UI controls |
| Test suite | Active | `npm test` runs Node test runner validating MSS library and plant dynamics |

There is also a known modeling inconsistency: profiles use a nonzero `cg` value in hydrostatic restoring moments, while their diagonal rigid-body mass and Coriolis representation assumes the body origin is at the CG. MSS `rbody.m` transforms both `MRB` and `CRB` using the CG lever arm. Resolve this before using the model for quantitative engineering comparisons.

### Gate for an MSS numerical-compliance claim

1. Select and name one exact MSS reference model and revision; “MSS” as a whole is not a single numerical model.
2. Port its complete mass, added-mass, Coriolis, damping, restoring, lift/drag, cross-flow, current, and propulsion calculations without substituting representative coefficients.
3. Support the required full matrix operations and CG/body-origin transformations.
4. Match its state constraints and input saturation instead of applying browser-specific state clamps inside the plant.
5. Run MATLAB or GNU Octave and JavaScript derivative comparisons over documented randomized states, currents, and inputs.
6. Run identical time histories through both implementations and publish tolerances, fixtures, MSS commit/revision, and results.
7. Keep custom ROV allocation and UI control outside the parity-tested plant as an explicit external generalized-force layer.

Until all seven items pass, use “MSS-inspired,” “based on MSS conventions,” or “reduced-order MSS formulation.” Do not use “MSS compliant,” “identical to MSS,” or “validated against MSS.”

## Visual specification

### Tokens

| CSS token | Value | Use |
| --- | --- | --- |
| `--abyss` | `#03131c` | Application background |
| `--panel` | `#08232c` | Telemetry surfaces |
| `--panel-2` | `#0b2b34` | Secondary surface |
| `--line` | `#1b4650` | Structural rules and grid gaps |
| `--ink` | `#d8eef0` | Primary text |
| `--muted` | `#84a7ac` | Labels and secondary readings |
| `--cyan` | `#50d5ce` | Live, active, and forward output |
| `--amber` | `#efb454` | Commands, setpoints, and reverse output |
| `--coral` | `#f07262` | Reserved warning/error accent |

Typography uses the local `Bahnschrift` or `DIN Alternate` family with `Arial Narrow` and Arial fallbacks. Numeric telemetry uses tabular figures where available. Labels are compact because this is an instrument panel, not a marketing page.

### Desktop layout

```text
+---------------------------------------------------+----------+
| identity             mission state        elapsed |          |
+---------------------------------------------------+ telemetry|
|                                                   | 310 px   |
|                  live 3D viewport                 | scrolls  |
|                                                   |          |
+---------------------------------------------------+          |
| pilot input | keys | current | assist/actions     |          |
+---------------------------------------------------+----------+
```

- Header height: 66 px.
- Control deck height: 150 px.
- Telemetry rail width: 310 px.
- Major regions are separated by a 1 px `--line` grid gap.
- The viewport remains the visual priority; telemetry scrolls independently when necessary.

### Responsive behavior

- Above 900 px: desktop grid with fixed telemetry rail.
- At 900 px and below: viewport, two-column telemetry, then controls; the page scrolls vertically.
- At 580 px and below: single-column telemetry and controls, hidden depth ruler, condensed header.
- Do not hide primary vehicle controls solely to preserve the desktop composition.

## Component and interaction states

| Component | Default | Active / pressed | Disabled / loading | Error |
| --- | --- | --- | --- | --- |
| Vehicle selector | Current class | New class selected | Disabled during GLB load | Status text reports model load failure |
| Demo button | `Demo ~` | Amber border; `Exit demo ~` | Not applicable | Route falls back to timeout progression |
| Camera button | Current mode | Label changes on cycle | Not applicable | Not applicable |
| Trail button | Visible | Amber pressed state | Not applicable | Not applicable |
| Assist button | Cyan inset rule | `aria-pressed=true` | Ignores clicks during demo | Not applicable |
| Pause button | Amber fill | Text changes to Resume | Not applicable | Not applicable |
| Annunciator | Dim ring | Cyan lamp; demo uses amber | Dim ring | Reserve coral for future fault state |
| Thruster bar | Cyan forward load | Amber reverse load | Empty at zero | Future fault state should retain channel row |

Interactive transitions are functional and short. Reduced-motion users receive no CSS transitions. Avoid adding autonomous decorative movement; propellers, particles, instruments, and vehicle motion already communicate system state.

## Content rules

- Vehicle names and channel labels come from `ROV_PROFILES`.
- Channel labels truncate with an ellipsis in the telemetry rail; preserve the full meaning near the start of the label.
- Force limits use newtons and locale-aware thousands separators.
- Depth and velocity include units in their readings.
- Loading and failure messages name the affected vehicle class.
- Do not call the representative profiles validated, certified, or hardware-accurate.

## Accessibility

- All actions are native buttons or form controls and must remain keyboard reachable.
- Preserve the visible 2 px white focus outline.
- Keep `aria-pressed` synchronized on toggle buttons.
- Update the camera button's accessible label when its mode changes.
- The viewport has an image role and descriptive label; critical state must also remain available as text telemetry.
- Color is not the only state signal: controls also use labels, fill direction, button text, or pressed semantics.
- Maintain the `prefers-reduced-motion` override.
- When adding faults, include text status rather than relying on coral alone.

## Loading, failure, and edge cases

- GLBs are loaded asynchronously and cached by class. Keep the selector disabled until the request completes.
- A model-load failure must leave the previous model usable and place a specific message in `statusText`.
- The telemetry list must accept 1–12 channels without fixed channel markup.
- Long localized labels should truncate in the rail without changing its width.
- `ResizeObserver` owns canvas sizing; do not derive it from window size alone.
- Model scale changes must be accompanied by chase-camera and light-placement review.
- Slow devices may accumulate display lag, but physics catch-up is bounded by the clamped frame delta.

## Validation checklist

### Code

- `npm run build` completes.
- No state or channel command becomes `NaN` or infinite in a 60-second scenario.
- Positive and negative allocation commands remain within `[-1, 1]`.
- Unsupported degrees of freedom do not create hidden forces.

### Models

- Rotor count equals profile thruster count.
- Channel indices are unique and contiguous from zero.
- Horizontal ducts match their physical vector orientation.
- Vertical ducts have unobstructed discs and flow columns.
- Propellers rotate about runtime local Y, not an Euler/world axis.

### Interface

- All five classes switch without stale channels.
- The `~` key enters and exits demo mode and restores prior camera/assist settings.
- Camera, trail, assist, pause, reset, and current controls work by mouse and keyboard.
- Desktop, 900 px, and 580 px layouts remain readable.
- Browser console contains no load or render errors.

## Known technical debt

- Automated tests cover core MSS math and unforced free decay; automated batch comparison against live Octave execution in CI remains to be wired.
- `main.js` combines rendering, input, orchestration, and telemetry responsibilities.
- The Three.js bundle triggers Vite's 500 kB chunk advisory.
- The retained Polyfork submarine module is unused.
- Vehicle data is embedded in JavaScript rather than loaded from a versioned schema.

Address verification debt before increasing physical fidelity; see [ROADMAP.md](ROADMAP.md).
