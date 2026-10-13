import { buildVehicle, rovPlant } from './mss/rovPlant.js';
import { rk4, ssa } from './mss/library.js';

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

export function thrusterWrench(commands, profile) {
  const tau = [0, 0, 0, 0, 0, 0];
  profile.thrusters.forEach((thruster, i) => {
    const ratio = effectiveThrustRatio(
      commands[i],
      thruster.reverseEfficiency ?? 0.75,
      thruster.deadband ?? 0.04
    );
    const force = scale(thruster.f, ratio * thruster.force);
    const moment = cross(scale(thruster.r, profile.leverScale), force);
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
    this.state = new Array(12).fill(0);
    this.current = { speed: 0.18, direction: Math.PI * 0.68 };
    this.configure(profileKey);
    this.reset();
  }

  configure(profileKey) {
    const profile = ROV_PROFILES[profileKey];
    if (!profile) throw new Error(`Unknown ROV profile: ${profileKey}`);
    this.profileKey = profileKey;
    this.profile = profile;
    this.vehicle = buildVehicle(profile);
    this.allocate = makeAllocator(profile);
    this.thrusters = new Array(profile.thrusters.length).fill(0);
    this.thrusterTarget = new Array(profile.thrusters.length).fill(0);
  }

  reset() {
    this.state.fill(0);
    this.state[8] = 2;       // Down = 2m depth
    this.state[11] = 0.35;   // Initial heading psi
    this.thrusters.fill(0);
    this.thrusterTarget.fill(0);
    this.time = 0;
  }

  setPilot({ surge, sway, heave, yaw, roll = 0, pitch = 0 }, assist, targets) {
    const nu = this.state.slice(0, 6), eta = this.state.slice(6);
    let sx = surge, sy = sway, hz = heave, rz = yaw;
    let rr = roll, pp = pitch;
    if (assist) {
      sx = clamp(1.9 * (surge * 0.9 - nu[0]), -1, 1);
      sy = clamp(2.2 * (sway * 0.65 - nu[1]), -1, 1);
      hz = heave ? clamp(2.3 * (heave * 0.5 - nu[2]), -1, 1) : clamp(0.34 * (targets.depth - eta[2]) - 1.1 * nu[2], -1, 1);
      rz = yaw ? clamp(2.0 * (yaw * 0.8 - nu[5]), -1, 1) : clamp(1.5 * wrapPi(targets.heading - eta[5]) - 1.0 * nu[5], -1, 1);
      rr = clamp(-1.8 * eta[3] - 0.8 * nu[3], -0.65, 0.65);
      pp = clamp(-1.8 * eta[4] - 0.8 * nu[4], -0.65, 0.65);
    }
    this.thrusterTarget = this.allocate([sx, sy, hz, rr, pp, rz]);
  }

  step(dt) {
    const actuatorAlpha = 1 - Math.exp(-dt / this.profile.actuatorTau);
    this.thrusters = this.thrusters.map((v, i) => v + (this.thrusterTarget[i] - v) * actuatorAlpha);
    const tau = thrusterWrench(this.thrusters, this.profile);

    // Integrate the pure unconstrained MSS plant via MSS rk4
    this.state = rk4(rovPlant, dt, this.state, tau, this.vehicle, this.current.speed, this.current.direction);

    // Physical seabed and boundary limits applied outside plant as environmental constraints
    this.state[8] = clamp(this.state[8], 0.15, 17.5); // Seabed is at 18m
    this.state[10] = clamp(this.state[10], -1.35, 1.35); // Pitch limits for Euler gimbal representation
    this.state[9] = wrapPi(this.state[9]);
    this.state[11] = wrapPi(this.state[11]);
    this.time += dt;
  }
}
