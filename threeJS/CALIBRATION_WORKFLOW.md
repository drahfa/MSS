# ROV Hydrodynamic & Propulsion Calibration Workflow

**Standard Operating Procedure (SOP) for Empirical Parameter Identification & Schema Generation**  
*Applicable to: Marine Systems Simulator (MSS) 6-DOF Web Simulator & `rov-profile.schema.json`*

---

## 1. Overview and Objective

This document defines the step-by-step engineering methodology for calibrating six-degree-of-freedom (6-DOF) hydrodynamic, inertial, and propulsive parameters from empirical tow-tank, test-bench, and sea-trial measurements.

By following this workflow, marine roboticists and offshore operators can generate a verified, publication-grade vehicle profile conforming to `public/schemas/rov-profile.schema.json` and hot-load it directly into the simulator without source code modification.

```
+--------------------------------------------------------------------------------+
|                          EMPIRICAL DATA SOURCES                                |
|  - Dry Weigh Scales       - Inclinometer / IMU        - Tow-Tank Load Cell     |
|  - Torsional Pendulum     - Bollard Pull Test Bench   - DVL Speed-Run Logs     |
+--------------------------------------------------------------------------------+
                                       |
                                       v
+--------------------------------------------------------------------------------+
|                       PARAMETER IDENTIFICATION PHASES                         |
|  Phase 1: Mass & Hydrostatics (m, I_G, z_G, B/W)                               |
|  Phase 2: Bollard Pull & Deadbands (F_max, eta_rev, u_db)                       |
|  Phase 3: Hydrodynamic Drag Decomposition (D_L, D_n)                           |
|  Phase 4: Inclinometer Free-Decay Identification (M_A, D_L55, omega_n, zeta)   |
|  Phase 5: Through-Water Advance Inflow Tuning (k_adv, v_max)                   |
+--------------------------------------------------------------------------------+
                                       |
                                       v
+--------------------------------------------------------------------------------+
|                         JSON SCHEMA PACKAGING & VALIDATION                     |
|  - Export/Validate against rov-profile.schema.json                             |
|  - Live Hot-Reloading into WebGL Simulator Cockpit                             |
+--------------------------------------------------------------------------------+
```

---

## Phase 1: Dry-Dock Physical & Hydrostatic Mass Calibration

### 1.1 Total Vehicle Mass ($m$)
- **Apparatus**: Multi-point crane scale or calibrated industrial platform scale.
- **Procedure**: Suspend the fully outfitted ROV (including payload skid, tooling, and oil-filled housings) in dry dock.
- **Metric**: Record total dry mass $m$ in kilograms.

### 1.2 Principal Moments of Inertia ($\mathbf{I}_G = [I_x, I_y, I_z]$)
- **Apparatus**: Bifilar or trifilar torsional pendulum suspension rig.
- **Procedure**: Suspend the vehicle horizontally from twin vertical cables of length $L$ separated by distance $2R$. Induce small angular torsional oscillations about the roll, pitch, and yaw axes respectively.
- **Formula**:
  $$I = \frac{m g R^2 T_{\text{torsion}}^2}{4 \pi^2 L}$$
  where $T_{\text{torsion}}$ is the measured oscillation period over 10 cycles.

### 1.3 Center of Gravity Offsets ($\mathbf{r}_{bG} = [x_G, y_G, z_G]^T$)
- **Apparatus**: Four-point reaction weigh scale balance or dry tilt table.
- **Procedure**: Measure reaction forces across lifting lugs. For subsea vehicles designed with positive metacentric stability, ballast lead weights are positioned along the bottom frame so that $x_G \approx 0$, $y_G \approx 0$, and $z_G > 0$ (measured downward from Coordinate Origin CO in meters).

### 1.4 Hydrostatic Buoyancy Ratio ($B/W$)
- **Apparatus**: Calm freshwater/seawater ballast tank with submerged load cell.
- **Procedure**: Submerge the trimmed vehicle. Measure net submerged weight $W_{\text{sub}} = W - B$.
- **Formula**:
  $$\frac{B}{W} = 1 - \frac{W_{\text{sub}}}{m g}$$
  Nominal trim targets slightly positive buoyancy ($B/W \approx 1.005\text{--}1.010$) to ensure passive recovery during power blackout.

---

## Phase 2: Static Bollard Pull Bench Calibration

### 2.1 Forward Bollard Thrust ($F_{\max, i}$)
- **Apparatus**: In-line load cell anchored to test-tank bulkhead, connected via tether to thruster mounting bracket.
- **Procedure**: Command $100\%$ full forward thrust ($u = +1.0$) in static ambient water ($v = 0$).
- **Metric**: Record steady-state forward bollard pull in Newtons ($F_{\max}$).

### 2.2 Reverse Thrust Efficiency ($\eta_{\text{rev}}$)
- **Procedure**: Command $100\%$ full reverse thrust ($u = -1.0$) on the same test bench.
- **Metric**: Compute reverse efficiency ratio:
  $$\eta_{\text{rev}} = \frac{|F_{\text{rev}}|}{F_{\max}} \in [0.65, 0.85]$$

### 2.3 ESC Command Deadband ($u_{\text{db}}$)
- **Procedure**: Sweep command $u$ upward from $0.00$ in increments of $0.005$ until detectable load cell tension ($> 0.5\text{ N}$) is registered.
- **Metric**: Record threshold $u_{\text{db}}$ (typically $0.02\text{--}0.05$, default $0.04$).

---

## Phase 3: Tow-Tank / Constant Speed Drag Calibration

### 3.1 Velocity-Decoupled Drag Separation ($D_L$ and $D_n$)
- **Apparatus**: Carriage-mounted tow-tank dynamometer or straight-line bollard towline.
- **Procedure**: Tow the unpowered vehicle along surge ($u$), sway ($v$), and heave ($w$) at constant speeds ranging from $0.2\text{ m/s}$ to $2.0\text{ m/s}$ in $0.2\text{ m/s}$ increments. Record steady tow force $F_{\text{drag}}$.
- **Mathematical Decomposition**:
  $$F_{\text{drag}}(u) = D_L u + D_n u^2$$
  Dividing both sides by towing speed $u$:
  $$\frac{F_{\text{drag}}(u)}{u} = D_L + D_n u$$
- **Linear Regression**:
  Perform linear regression of $\frac{F_{\text{drag}}}{u}$ versus $u$:
  - **Slope** $= D_n$ (quadratic vortex-shedding drag coefficient in $\text{N}\cdot\text{s}^2/\text{m}^2$).
  - **Intercept** $= D_L$ (linear skin friction coefficient in $\text{N}\cdot\text{s}/\text{m}$).

---

## Phase 4: Inclinometer Free-Decay Calibration

### 4.1 Submerged Angular Oscillation Test
- **Apparatus**: Onboard 6-axis IMU / precision inclinometer logging at $\ge 100\text{ Hz}$.
- **Procedure**: Submerge the vehicle in calm water without tether tension. Tilt the vehicle in pitch to initial angle $\theta_0 \approx 8^\circ\text{--}10^\circ$ ($0.15\text{ rad}$) and release freely with thrusters disabled ($\boldsymbol{\tau} = \mathbf{0}$).
- **Measurements**:
  1. Record successive positive peak times $t_1, t_2, \dots, t_N$ and amplitudes $\theta_1, \theta_2, \dots, \theta_N$.
  2. Compute measured damped oscillation period:
     $$T_d = \frac{t_N - t_1}{N - 1}$$
  3. Compute logarithmic decrement:
     $$\delta = \frac{1}{N - 1} \ln\left(\frac{\theta_1}{\theta_N}\right)$$

### 4.2 Parameter Extraction
1. **Damping Ratio**:
   $$\zeta = \frac{\delta}{\sqrt{4 \pi^2 + \delta^2}}$$
2. **Undamped Natural Frequency**:
   $$\omega_n = \frac{2\pi}{T_d \sqrt{1 - \zeta^2}}$$
3. **Total Pitch System Inertia ($M_{55}$)**:
   $$M_{55} = \frac{z_G W}{\omega_n^2}$$
4. **Hydrodynamic Added Inertia ($M_{A, 55} = M_{\dot{q}}$)**:
   $$M_{A, 55} = M_{55} - I_y - m z_G^2$$
5. **Linear Pitch Damping ($D_{L, 55}$)**:
   $$D_{L, 55} = 2 \zeta \sqrt{M_{55} z_G W}$$

Repeat the same protocol for roll ($\phi$) to calibrate $M_{A, 44} = K_{\dot{p}}$ and $D_{L, 44} = K_p$.

---

## Phase 5: Sea-Trial Advance Ratio Inflow Calibration

### 5.1 Open-Water Full Throttle Speed-Run
- **Apparatus**: Doppler Velocity Log (DVL) or acoustic tracking transponder (USBL) in calm sea conditions.
- **Procedure**: Command sustained $100\%$ forward surge ($u = 1.0$) until acceleration stabilizes to steady cruising speed $u_{\text{cruise}}$.
- **Analysis**:
  At steady cruising speed, the degraded physical forward thrust balances total hydrodynamic drag:
  $$F_{\text{steady}} = D_{L, 1} u_{\text{cruise}} + D_{n, 1} u_{\text{cruise}}^2$$
  Comparing $F_{\text{steady}}$ against static bollard pull $F_{\text{static}} = \sum_{i} F_{\max, i} f_{x, i}$:
  $$\eta_{\text{advance}}(u_{\text{cruise}}) = \frac{F_{\text{steady}}}{F_{\text{static}}} = 1 - k_{\text{adv}} \frac{u_{\text{cruise}}}{V_{\max}}$$
- **Tuning Formula**:
  $$k_{\text{adv}} = \frac{V_{\max}}{u_{\text{cruise}}} \left( 1 - \frac{D_{L, 1} u_{\text{cruise}} + D_{n, 1} u_{\text{cruise}}^2}{F_{\text{static}}} \right)$$
  Typically $k_{\text{adv}} \approx 0.25\text{--}0.35$ for reference speed $V_{\max} = 2.0\text{ m/s}$.

---

## Phase 6: JSON Profile Formatting & Live Simulator Import

Once all parameters are extracted, format the values into JSON adhering to `rov-profile.schema.json`:

```json
{
  "$schema": "../schemas/rov-profile.schema.json",
  "name": "Calibrated Intervention ROV",
  "role": "Offshore Inspection · 1,500 m",
  "mass": 92.5,
  "inertia": [9.4, 14.1, 15.8],
  "addedMass": [39.0, 56.0, 65.0, 2.3, 6.2, 6.8],
  "linearDamping": [41.0, 58.0, 72.0, 8.6, 13.8, 15.0],
  "quadraticDamping": [70.0, 94.0, 112.0, 6.5, 10.8, 11.6],
  "leverScale": 1.35,
  "cg": 0.048,
  "buoyancyRatio": 1.007,
  "thrusters": [
    {
      "id": "H1", "label": "Front Port Vector",
      "r": [0.34, -0.48, 0], "f": [0.7071, 0.7071, 0],
      "force": 70.0, "reverseEfficiency": 0.75, "deadband": 0.04,
      "kAdvance": 0.30, "vMax": 2.0
    }
  ]
}
```

### Loading into Simulator
1. Open the simulator in any browser.
2. Click **Profile JSON** in the top viewport toolbar.
3. Paste the formatted JSON into the editor pane or click **Import File**.
4. Observe real-time validation: verify that the badge displays **Schema Valid ✓**.
5. Click **Apply to Simulator**. The 6-DOF plant, thruster allocation matrix, and HUD telemetry will immediately hot-reload to the calibrated profile without browser refreshes.
