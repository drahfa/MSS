import { buildVehicle, rovPlant } from './mss/rovPlant.js';
import { rk4, ssa, euler2q, q2euler, Rquat, normalizeQuat } from './mss/library.js';
import { TetherModel } from './tether.js';
import { ContactModel } from './contact.js';
import { FailureInjector } from './failureInjection.js';
import { SensorSuite } from './sensors.js';
import { LumpedMassTether } from './lumpedMassTether.js';
import { NavigationFilter } from './navigationFilter.js';

export const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
export const wrapPi = angle => ssa(angle);

const scale = (a, s) => a.map(v => v * s);
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];

const C45 = Math.SQRT1_2;
const unit = (id, label, r, f, force, reverseEfficiency = 0.75, deadband = 0.04) => ({
  id, label, r, f, force, reverseEfficiency, deadband
});

export const ROV_PROFILES = {
  hobbyist: {
    name: 'Hobbyist', role: 'Recreation · 100 m', mass: 4.2,
    inertia: [.16, .24, .28], addedMass: [2.0, 3.1, 3.6, .04, .08, .09],
    linearDamping: [4.0, 6.2, 7.4, .45, .72, .78], quadraticDamping: [6, 9, 11, .32, .5, .55],
    leverScale: .58, cg: .018, cameraDistance: 4.2, actuatorTau: .10,
    buoyancyRatio: 1.006,
    thrusters: [
      unit('H1', 'Port drive', [0, -.43, 0], [1, 0, 0], 5, 0.75, 0.04),
      unit('H2', 'Starboard drive', [0, .43, 0], [1, 0, 0], 5, 0.75, 0.04),
      unit('V1', 'Center depth', [0, 0, -.05], [0, 0, 1], 6, 0.75, 0.04),
    ],
  },
  observation: {
    name: 'Observation', role: 'Visual survey · 300 m', mass: 22,
    inertia: [.72, 1.05, 1.2], addedMass: [8.5, 13.5, 16, .14, .42, .48],
    linearDamping: [11, 16, 20, 1.6, 2.5, 2.8], quadraticDamping: [18, 25, 30, 1.1, 1.8, 2],
    leverScale: .9, cg: .028, cameraDistance: 5.1, actuatorTau: .14,
    buoyancyRatio: 1.006,
    thrusters: [
      unit('H1', 'Port drive', [0, -.56, .16], [1, 0, 0], 18, 0.75, 0.04),
      unit('H2', 'Starboard drive', [0, .56, .16], [1, 0, 0], 18, 0.75, 0.04),
      unit('V1', 'Port lift', [.12, -.88, -.18], [0, 0, 1], 21, 0.75, 0.04),
      unit('V2', 'Starboard lift', [.12, .88, -.18], [0, 0, 1], 21, 0.75, 0.04),
    ],
  },
  inspection: {
    name: 'Inspection', role: 'Survey + intervention · 1,000 m', mass: 85,
    inertia: [8.5, 12.8, 14.2], addedMass: [36, 52, 61, 2.1, 5.8, 6.3],
    linearDamping: [38, 54, 68, 8, 13, 14], quadraticDamping: [65, 88, 106, 6, 10, 11],
    leverScale: 1.35, cg: .045, cameraDistance: 6.3, actuatorTau: .18,
    buoyancyRatio: 1.006,
    thrusters: [
      unit('H1', 'Front port vector', [.34, -.48, 0], [C45, C45, 0], 65, 0.75, 0.04),
      unit('H2', 'Front starboard vector', [.34, .48, 0], [C45, -C45, 0], 65, 0.75, 0.04),
      unit('H3', 'Aft port vector', [-.34, -.48, 0], [C45, -C45, 0], 65, 0.75, 0.04),
      unit('H4', 'Aft starboard vector', [-.34, .48, 0], [C45, C45, 0], 65, 0.75, 0.04),
      unit('V1', 'Front port lift', [.43, -.86, -.43], [0, 0, 1], 72, 0.75, 0.04),
      unit('V2', 'Front starboard lift', [.43, .86, -.43], [0, 0, 1], 72, 0.75, 0.04),
      unit('V3', 'Aft port lift', [-.43, -.86, -.43], [0, 0, 1], 72, 0.75, 0.04),
      unit('V4', 'Aft starboard lift', [-.43, .86, -.43], [0, 0, 1], 72, 0.75, 0.04),
    ],
  },
  'light-work': {
    name: 'Light work', role: 'Manipulation + tooling · 3,000 m', mass: 650,
    inertia: [145, 230, 260], addedMass: [285, 410, 475, 44, 116, 128],
    linearDamping: [250, 360, 430, 92, 146, 160], quadraticDamping: [390, 530, 650, 68, 112, 125],
    leverScale: 2.1, cg: .09, cameraDistance: 8.3, actuatorTau: .28,
    buoyancyRatio: 1.006,
    thrusters: [
      unit('H1', 'Front port vector', [.46, -.86, 0], [C45, C45, 0], 420, 0.75, 0.04),
      unit('H2', 'Front starboard vector', [.46, .86, 0], [C45, -C45, 0], 420, 0.75, 0.04),
      unit('H3', 'Aft port vector', [-.46, -.86, 0], [C45, -C45, 0], 420, 0.75, 0.04),
      unit('H4', 'Aft starboard vector', [-.46, .86, 0], [C45, C45, 0], 420, 0.75, 0.04),
      unit('V1', 'Port lift', [.08, -1.30, -.42], [0, 0, 1], 470, 0.75, 0.04),
      unit('V2', 'Starboard lift', [.08, 1.30, -.42], [0, 0, 1], 470, 0.75, 0.04),
    ],
  },
  'heavy-work': {
    name: 'Heavy work', role: 'Construction + recovery · 6,000 m', mass: 4200,
    inertia: [3100, 5450, 5900], addedMass: [1900, 2800, 3300, 860, 2100, 2300],
    linearDamping: [1500, 2300, 2800, 690, 1150, 1260], quadraticDamping: [2400, 3400, 4200, 520, 880, 980],
    leverScale: 3.4, cg: .18, cameraDistance: 13.2, actuatorTau: .42,
    buoyancyRatio: 1.006,
    thrusters: [
      unit('H1', 'Front port vector', [.58, -1.72, 0], [C45, C45, 0], 2100, 0.75, 0.04),
      unit('H2', 'Front starboard vector', [.58, 1.72, 0], [C45, -C45, 0], 2100, 0.75, 0.04),
      unit('H3', 'Aft port vector', [-.58, -1.72, 0], [C45, -C45, 0], 2100, 0.75, 0.04),
      unit('H4', 'Aft starboard vector', [-.58, 1.72, 0], [C45, C45, 0], 2100, 0.75, 0.04),
      unit('V1', 'Front port lift', [.72, -2.12, -.62], [0, 0, 1], 2350, 0.75, 0.04),
      unit('V2', 'Front starboard lift', [.72, 2.12, -.62], [0, 0, 1], 2350, 0.75, 0.04),
      unit('V3', 'Aft port lift', [-.72, -2.12, -.62], [0, 0, 1], 2350, 0.75, 0.04),
      unit('V4', 'Aft starboard lift', [-.72, 2.12, -.62], [0, 0, 1], 2350, 0.75, 0.04),
    ],
  },
};

/**
 * Maps normalized actuator command u in [-1, 1] to effective thrust ratio,
 * taking into account thruster deadband and reverse asymmetry.
 */
export function effectiveThrustRatio(u, reverseEfficiency = 0.75, deadband = 0.04) {
  const absU = Math.abs(u);
  if (absU <= deadband) return 0;
  // Rescale range [deadband, 1] linearly to [0, 1]
  const activeFraction = (absU - deadband) / (1 - deadband);
  return u > 0 ? activeFraction : -activeFraction * reverseEfficiency;
}

/**
 * Propeller advance ratio degradation factor:
 * As local axial inflow velocity increases in the direction of commanded thrust,
 * effective angle of attack drops and thrust reduces compared to static bollard pull.
 */
export function advanceRatioFactor(axialVelocity = 0, u = 0, kAdv = 0.30, vMax = 2.0) {
  if (Math.abs(u) < 1e-4) return 1.0;
  const alignedInflow = Math.sign(u) * axialVelocity;
  const factor = 1 - kAdv * (alignedInflow / vMax);
  return clamp(factor, 0.1, 1.15);
}

/**
 * Vertical ocean current shear profile:
 * Models current velocity reduction with depth due to seabed boundary layer friction.
 */
export function currentShear(depth, surfaceSpeed, seabedDepth = 18.0, bottomFactor = 0.5) {
  const d = clamp(depth, 0, seabedDepth);
  const shearFactor = 1.0 - (1.0 - bottomFactor) * (d / seabedDepth);
  return surfaceSpeed * Math.max(0, shearFactor);
}

export function thrusterWrench(commands, profile, nu_r = null) {
  const tau = [0, 0, 0, 0, 0, 0];
  const v_b = nu_r ? nu_r.slice(0, 3) : null;
  const omega = nu_r ? nu_r.slice(3, 6) : null;

  profile.thrusters.forEach((thruster, i) => {
    let ratio = effectiveThrustRatio(
      commands[i],
      thruster.reverseEfficiency ?? 0.75,
      thruster.deadband ?? 0.04
    );

    const rLever = scale(thruster.r, profile.leverScale);

    if (v_b && omega && ratio !== 0) {
      // Local flow velocity at thruster: v_local = v_b + omega x r
      const omegaCrossR = cross(omega, rLever);
      const vLocal = [v_b[0] + omegaCrossR[0], v_b[1] + omegaCrossR[1], v_b[2] + omegaCrossR[2]];
      const axialVelocity = vLocal[0] * thruster.f[0] + vLocal[1] * thruster.f[1] + vLocal[2] * thruster.f[2];
      const advFactor = advanceRatioFactor(
        axialVelocity,
        commands[i],
        thruster.kAdvance ?? 0.30,
        thruster.vMax ?? 2.0
      );
      ratio *= advFactor;
    }

    const force = scale(thruster.f, ratio * thruster.force);
    const moment = cross(rLever, force);
    for (let j = 0; j < 3; j++) {
      tau[j] += force[j];
      tau[j + 3] += moment[j];
    }
  });
  return tau;
}

function invert(matrix) {
  const size = matrix.length;
  const a = matrix.map((row, i) => [...row, ...Array.from({ length: size }, (_, j) => +(i === j))]);
  for (let col = 0; col < size; col++) {
    let pivot = col;
    for (let row = col + 1; row < size; row++) {
      if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) pivot = row;
    }
    [a[col], a[pivot]] = [a[pivot], a[col]];
    const divisor = a[col][col];
    for (let j = 0; j < size * 2; j++) a[col][j] /= divisor;
    for (let row = 0; row < size; row++) {
      if (row === col) continue;
      const factor = a[row][col];
      for (let j = 0; j < size * 2; j++) a[row][j] -= factor * a[col][j];
    }
  }
  return a.map(row => row.slice(size));
}

export function makeAllocator(profile) {
  // Columns are physical 6-DOF wrenches [force; moment]. Normalizing each row
  // by its available authority keeps force and moment units well conditioned.
  const columns = profile.thrusters.map(thruster => {
    const force = scale(thruster.f, thruster.force);
    return [...force, ...cross(scale(thruster.r, profile.leverScale), force)];
  });
  const authority = Array.from({ length: 6 }, (_, row) =>
    columns.reduce((sum, column) => sum + Math.abs(column[row]), 0)
  );
  const allocation = Array.from({ length: 6 }, (_, row) =>
    columns.map(column => authority[row] > 1e-8 ? column[row] / authority[row] : 0)
  );
  const gram = Array.from({ length: 6 }, (_, row) =>
    Array.from({ length: 6 }, (_, col) =>
      allocation[row].reduce((sum, value, i) => sum + value * allocation[col][i], 0) + (row === col ? .002 : 0)
    )
  );
  const gramInverse = invert(gram);
  return demand => {
    const dual = gramInverse.map(row => row.reduce((sum, x, i) => sum + x * demand[i], 0));
    const commands = columns.map((_, i) => allocation.reduce((sum, row, axis) => sum + row[i] * dual[axis], 0));
    const peak = Math.max(1, ...commands.map(Math.abs));
    return commands.map(value => clamp(value / peak, -1, 1));
  };
}

export class RovDynamics {
  constructor(profileKey = 'inspection') {
    this.state = new Array(13).fill(0);
    this.current = { speed: 0.18, direction: Math.PI * 0.68 };
    this.configure(profileKey);
    this.reset();
  }

  configure(profileKey, customProfile = null) {
    const profile = customProfile || ROV_PROFILES[profileKey];
    if (!profile) throw new Error(`Unknown ROV profile: ${profileKey}`);
    this.profileKey = profileKey;
    this.profile = profile;
    this.vehicle = buildVehicle(profile);
    this.allocate = makeAllocator(profile);
    this.thrusters = new Array(profile.thrusters.length).fill(0);
    this.thrusterTarget = new Array(profile.thrusters.length).fill(0);
    this.failureInjector = new FailureInjector(profile.thrusters.length);
    this.tether = new TetherModel();
    this.lumpedTether = new LumpedMassTether({ seabedDepth: 18.0 });
    this.useLumpedTether = false;
    this.contact = new ContactModel();
    this.sensors = new SensorSuite();
    this.navFilter = new NavigationFilter();
    this.tetherState = null;
    this.contactState = null;
    this.sensorReadings = null;
  }

  reset() {
    this.state.fill(0);
    this.state[8] = 2; // Down = 2m depth
    const q0 = euler2q(0, 0, 0.35); // Initial heading psi = 0.35 rad
    this.state[9] = q0[0];
    this.state[10] = q0[1];
    this.state[11] = q0[2];
    this.state[12] = q0[3];
    this.thrusters.fill(0);
    this.thrusterTarget.fill(0);
    this.time = 0;
    this.lumpedTether.reset([0, 0, 0.5], [this.state[6], this.state[7], this.state[8]]);
    this.navFilter.reset([this.state[6], this.state[7], this.state[8]], 0.35);
  }

  getEuler() {
    return q2euler(this.state.slice(9, 13));
  }

  setPilot({ surge, sway, heave, yaw, roll = 0, pitch = 0 }, assist, targets) {
    const nu = this.state.slice(0, 6);
    const [phi, theta, psi] = this.getEuler();
    const depth = this.state[8];
    let sx = surge, sy = sway, hz = heave, rz = yaw;
    let rr = roll, pp = pitch;
    if (assist) {
      sx = clamp(1.9 * (surge * 0.9 - nu[0]), -1, 1);
      sy = clamp(2.2 * (sway * 0.65 - nu[1]), -1, 1);
      hz = heave ? clamp(2.3 * (heave * 0.5 - nu[2]), -1, 1) : clamp(0.34 * (targets.depth - depth) - 1.1 * nu[2], -1, 1);
      rz = yaw ? clamp(2.0 * (yaw * 0.8 - nu[5]), -1, 1) : clamp(1.5 * wrapPi(targets.heading - psi) - 1.0 * nu[5], -1, 1);
      rr = clamp(-1.8 * phi - 0.8 * nu[3], -0.65, 0.65);
      pp = clamp(-1.8 * theta - 0.8 * nu[4], -0.65, 0.65);
    }
    this.thrusterTarget = this.allocate([sx, sy, hz, rr, pp, rz]);
  }

  step(dt) {
    const actuatorAlpha = 1 - Math.exp(-dt / this.profile.actuatorTau);
    this.thrusters = this.thrusters.map((v, i) => v + (this.thrusterTarget[i] - v) * actuatorAlpha);

    // Apply active fault injection to commanded outputs
    const effectiveThrusters = this.failureInjector.processCommands(this.thrusters);

    // Current shear: calculate local current speed at vehicle depth (state[8])
    const currentSpeed = currentShear(this.state[8], this.current.speed, 18.0);
    this.localCurrentSpeed = currentSpeed;
    const R = Rquat(this.state.slice(9, 13));
    const vc_n = [
      currentSpeed * Math.cos(this.current.direction),
      currentSpeed * Math.sin(this.current.direction),
      0,
    ];
    // Exact body current using quaternion rotation R^T * v_c^n
    const vc_b = [
      R[0][0] * vc_n[0] + R[1][0] * vc_n[1] + R[2][0] * vc_n[2],
      R[0][1] * vc_n[0] + R[1][1] * vc_n[1] + R[2][1] * vc_n[2],
      R[0][2] * vc_n[0] + R[1][2] * vc_n[1] + R[2][2] * vc_n[2],
    ];
    const nu_r = [
      this.state[0] - vc_b[0],
      this.state[1] - vc_b[1],
      this.state[2] - vc_b[2],
      this.state[3],
      this.state[4],
      this.state[5],
    ];

    const tauThruster = thrusterWrench(effectiveThrusters, this.profile, nu_r);

    // External interaction forces: umbilical tether and contact response
    const pos = this.state.slice(6, 9);
    const nu1 = this.state.slice(0, 3);
    let tetherData;
    if (this.useLumpedTether) {
      const v_ned = [
        R[0][0] * nu1[0] + R[0][1] * nu1[1] + R[0][2] * nu1[2],
        R[1][0] * nu1[0] + R[1][1] * nu1[1] + R[1][2] * nu1[2],
        R[2][0] * nu1[0] + R[2][1] * nu1[1] + R[2][2] * nu1[2],
      ];
      const lumpedRes = this.lumpedTether.step(dt, [0, 0, 0.5], pos, v_ned, R, currentSpeed, this.current.direction);
      const payout = Math.hypot(pos[0], pos[1], pos[2] - 0.5);
      tetherData = {
        payout,
        tension: lumpedRes.tensionMagnitude,
        totalForceBody: [lumpedRes.wrenchBody[0], lumpedRes.wrenchBody[1], lumpedRes.wrenchBody[2]],
        snag: lumpedRes.tensionMagnitude > 1200,
        advisory: lumpedRes.tensionMagnitude > 800,
        nodes: lumpedRes.nodes,
      };
    } else {
      tetherData = this.tether.compute(pos, nu1, R);
    }
    const contactData = this.contact.evaluate(pos, nu1, R);
    this.tetherState = tetherData;
    this.contactState = contactData;

    const tau = [
      tauThruster[0] + tetherData.totalForceBody[0] + contactData.contactForceBody[0],
      tauThruster[1] + tetherData.totalForceBody[1] + contactData.contactForceBody[1],
      tauThruster[2] + tetherData.totalForceBody[2] + contactData.contactForceBody[2],
      tauThruster[3],
      tauThruster[4],
      tauThruster[5],
    ];

    // Integrate the pure unconstrained MSS plant via MSS rk4 using depth-sheared current
    this.state = rk4(rovPlant, dt, this.state, tau, this.vehicle, currentSpeed, this.current.direction);

    // Normalize unit quaternion on S^3
    const qNorm = normalizeQuat(this.state.slice(9, 13));
    this.state[9] = qNorm[0];
    this.state[10] = qNorm[1];
    this.state[11] = qNorm[2];
    this.state[12] = qNorm[3];

    // Physical seabed boundary limits applied outside plant as environmental constraints
    this.state[8] = clamp(this.state[8], 0.15, 17.85); // Seabed is at 18m
    this.time += dt;

    // Sample synthetic instrument sensors
    this.sensorReadings = this.sensors.sample(this.state, null, R, dt);

    // Feed real-time observations into continuous-discrete Error-State EKF
    if (this.sensorReadings) {
      if (this.sensorReadings.imu) {
        this.navFilter.predict(dt, this.sensorReadings.imu.accel, this.sensorReadings.imu.rates);
      }
      if (this.sensorReadings.dvl && this.sensorReadings.dvl.bottomLock && this.sensorReadings.dvl.u !== null) {
        this.navFilter.updateDVL([this.sensorReadings.dvl.u, this.sensorReadings.dvl.v, this.sensorReadings.dvl.w]);
      }
      if (this.sensorReadings.depth !== null && this.sensorReadings.depth !== undefined) {
        this.navFilter.updateDepth(this.sensorReadings.depth);
      }
      if (this.sensorReadings.compass !== null && this.sensorReadings.compass !== undefined) {
        this.navFilter.updateCompass(this.sensorReadings.compass);
      }
    }
  }
}
