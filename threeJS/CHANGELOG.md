# Changelog

All notable changes to the browser ROV simulator are documented here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow Semantic Versioning.

## Unreleased

### Planned

- Automated dynamics, model-metadata, and browser regression tests

## 1.0.0 - 2026-10-13

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

