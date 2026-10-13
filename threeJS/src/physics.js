const G = 9.81;

export const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
export const wrapPi = angle => Math.atan2(Math.sin(angle), Math.cos(angle));

const add = (a, b) => a.map((v, i) => v + b[i]);
const scale = (a, s) => a.map(v => v * s);
const matVec = (m, v) => m.map(row => row.reduce((sum, x, i) => sum + x * v[i], 0));
const sub = (a, b) => a.map((v, i) => v - b[i]);
const skew = ([x, y, z]) => [[0, -z, y], [z, 0, -x], [-y, x, 0]];

function blockCoriolis(M, nu) {
  const p1 = [M[0] * nu[0], M[1] * nu[1], M[2] * nu[2]];
  const p2 = [M[3] * nu[3], M[4] * nu[4], M[5] * nu[5]];
  const s1 = skew(p1), s2 = skew(p2);
  return [
    [0, 0, 0, -s1[0][0], -s1[0][1], -s1[0][2]],
    [0, 0, 0, -s1[1][0], -s1[1][1], -s1[1][2]],
    [0, 0, 0, -s1[2][0], -s1[2][1], -s1[2][2]],
    [-s1[0][0], -s1[0][1], -s1[0][2], -s2[0][0], -s2[0][1], -s2[0][2]],
    [-s1[1][0], -s1[1][1], -s1[1][2], -s2[1][0], -s2[1][1], -s2[1][2]],
    [-s1[2][0], -s1[2][1], -s1[2][2], -s2[2][0], -s2[2][1], -s2[2][2]],
  ];
}

function rotation(phi, theta, psi) {
  const cp = Math.cos(phi), sp = Math.sin(phi);
  const ct = Math.cos(theta), st = Math.sin(theta);
  const cy = Math.cos(psi), sy = Math.sin(psi);
  return [
    [cy * ct, -sy * cp + cy * st * sp, sy * sp + cy * cp * st],
    [sy * ct, cy * cp + sp * st * sy, -cy * sp + st * sy * cp],
    [-st, ct * sp, ct * cp],
  ];
}

function eulerRates(phi, theta, [p, q, r]) {
  const cp = Math.cos(phi), sp = Math.sin(phi);
  const ct = Math.max(0.08, Math.cos(theta));
  const tt = Math.sin(theta) / ct;
  return [p + sp * tt * q + cp * tt * r, cp * q - sp * r, (sp * q + cp * r) / ct];
}

function rigidBodyCoriolis(mass, inertia, omega) {
  const ms = skew(scale(omega, mass));
  const is = skew(inertia.map((v, i) => v * omega[i]));
  return [
    [ms[0][0], ms[0][1], ms[0][2], 0, 0, 0],
    [ms[1][0], ms[1][1], ms[1][2], 0, 0, 0],
    [ms[2][0], ms[2][1], ms[2][2], 0, 0, 0],
    [0, 0, 0, -is[0][0], -is[0][1], -is[0][2]],
    [0, 0, 0, -is[1][0], -is[1][1], -is[1][2]],
    [0, 0, 0, -is[2][0], -is[2][1], -is[2][2]],
  ];
}

const C45 = Math.SQRT1_2;
const unit = (id, label, r, f, force) => ({ id, label, r, f, force });

export const ROV_PROFILES = {
  hobbyist: {
    name: 'Hobbyist', role: 'Recreation · 100 m', mass: 4.2,
    inertia: [.16, .24, .28], addedMass: [2.0, 3.1, 3.6, .04, .08, .09],
    linearDamping: [4.0, 6.2, 7.4, .45, .72, .78], quadraticDamping: [6, 9, 11, .32, .5, .55],
    leverScale: .58, cg: .018, cameraDistance: 4.2, actuatorTau: .10,
    thrusters: [
      unit('H1', 'Port drive', [0, -.43, 0], [1, 0, 0], 5),
      unit('H2', 'Starboard drive', [0, .43, 0], [1, 0, 0], 5),
      unit('V1', 'Center depth', [0, 0, -.05], [0, 0, 1], 6),
    ],
  },
  observation: {
    name: 'Observation', role: 'Visual survey · 300 m', mass: 22,
    inertia: [.72, 1.05, 1.2], addedMass: [8.5, 13.5, 16, .14, .42, .48],
    linearDamping: [11, 16, 20, 1.6, 2.5, 2.8], quadraticDamping: [18, 25, 30, 1.1, 1.8, 2],
    leverScale: .9, cg: .028, cameraDistance: 5.1, actuatorTau: .14,
    thrusters: [
      unit('H1', 'Port drive', [0, -.56, .16], [1, 0, 0], 18),
      unit('H2', 'Starboard drive', [0, .56, .16], [1, 0, 0], 18),
      unit('V1', 'Port lift', [.12, -.88, -.18], [0, 0, 1], 21),
      unit('V2', 'Starboard lift', [.12, .88, -.18], [0, 0, 1], 21),
    ],
  },
  inspection: {
    name: 'Inspection', role: 'Survey + intervention · 1,000 m', mass: 85,
    inertia: [8.5, 12.8, 14.2], addedMass: [36, 52, 61, 2.1, 5.8, 6.3],
    linearDamping: [38, 54, 68, 8, 13, 14], quadraticDamping: [65, 88, 106, 6, 10, 11],
    leverScale: 1.35, cg: .045, cameraDistance: 6.3, actuatorTau: .18,
    thrusters: [
      unit('H1', 'Front port vector', [.34, -.48, 0], [C45, C45, 0], 65),
      unit('H2', 'Front starboard vector', [.34, .48, 0], [C45, -C45, 0], 65),
      unit('H3', 'Aft port vector', [-.34, -.48, 0], [C45, -C45, 0], 65),
      unit('H4', 'Aft starboard vector', [-.34, .48, 0], [C45, C45, 0], 65),
      unit('V1', 'Front port lift', [.43, -.86, -.43], [0, 0, 1], 72),
      unit('V2', 'Front starboard lift', [.43, .86, -.43], [0, 0, 1], 72),
      unit('V3', 'Aft port lift', [-.43, -.86, -.43], [0, 0, 1], 72),
      unit('V4', 'Aft starboard lift', [-.43, .86, -.43], [0, 0, 1], 72),
    ],
  },
  'light-work': {
    name: 'Light work', role: 'Manipulation + tooling · 3,000 m', mass: 650,
    inertia: [145, 230, 260], addedMass: [285, 410, 475, 44, 116, 128],
    linearDamping: [250, 360, 430, 92, 146, 160], quadraticDamping: [390, 530, 650, 68, 112, 125],
    leverScale: 2.1, cg: .09, cameraDistance: 8.3, actuatorTau: .28,
    thrusters: [
      unit('H1', 'Front port vector', [.46, -.86, 0], [C45, C45, 0], 420),
      unit('H2', 'Front starboard vector', [.46, .86, 0], [C45, -C45, 0], 420),
      unit('H3', 'Aft port vector', [-.46, -.86, 0], [C45, -C45, 0], 420),
      unit('H4', 'Aft starboard vector', [-.46, .86, 0], [C45, C45, 0], 420),
      unit('V1', 'Port lift', [.08, -1.30, -.42], [0, 0, 1], 470),
      unit('V2', 'Starboard lift', [.08, 1.30, -.42], [0, 0, 1], 470),
    ],
  },
  'heavy-work': {
    name: 'Heavy work', role: 'Construction + recovery · 6,000 m', mass: 4200,
    inertia: [3100, 5450, 5900], addedMass: [1900, 2800, 3300, 860, 2100, 2300],
    linearDamping: [1500, 2300, 2800, 690, 1150, 1260], quadraticDamping: [2400, 3400, 4200, 520, 880, 980],
    leverScale: 3.4, cg: .18, cameraDistance: 13.2, actuatorTau: .42,
    thrusters: [
      unit('H1', 'Front port vector', [.58, -1.72, 0], [C45, C45, 0], 2100),
      unit('H2', 'Front starboard vector', [.58, 1.72, 0], [C45, -C45, 0], 2100),
      unit('H3', 'Aft port vector', [-.58, -1.72, 0], [C45, -C45, 0], 2100),
      unit('H4', 'Aft starboard vector', [-.58, 1.72, 0], [C45, C45, 0], 2100),
      unit('V1', 'Front port lift', [.72, -2.12, -.62], [0, 0, 1], 2350),
      unit('V2', 'Front starboard lift', [.72, 2.12, -.62], [0, 0, 1], 2350),
      unit('V3', 'Aft port lift', [-.72, -2.12, -.62], [0, 0, 1], 2350),
      unit('V4', 'Aft starboard lift', [-.72, 2.12, -.62], [0, 0, 1], 2350),
    ],
  },
};

function cross(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

function thrusterWrench(commands, profile) {
  const tau = [0, 0, 0, 0, 0, 0];
  profile.thrusters.forEach((thruster, i) => {
    const force = scale(thruster.f, commands[i] * thruster.force);
    const moment = cross(scale(thruster.r, profile.leverScale), force);
    for (let j = 0; j < 3; j++) { tau[j] += force[j]; tau[j + 3] += moment[j]; }
  });
  return tau;
}

function invert(matrix) {
  const size = matrix.length;
  const a = matrix.map((row, i) => [...row, ...Array.from({ length: size }, (_, j) => +(i === j))]);
  for (let col = 0; col < size; col++) {
    let pivot = col;
    for (let row = col + 1; row < size; row++) if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) pivot = row;
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

function makeAllocator(profile) {
  // Columns are physical 6-DOF wrenches [force; moment]. Normalizing each row
  // by its available authority keeps force and moment units well conditioned.
  const columns = profile.thrusters.map(thruster => {
    const force = scale(thruster.f, thruster.force);
    return [...force, ...cross(scale(thruster.r, profile.leverScale), force)];
  });
  const authority = Array.from({ length: 6 }, (_, row) => columns.reduce((sum, column) => sum + Math.abs(column[row]), 0));
  const allocation = Array.from({ length: 6 }, (_, row) => columns.map(column => authority[row] > 1e-8 ? column[row] / authority[row] : 0));
  const gram = Array.from({ length: 6 }, (_, row) => Array.from({ length: 6 }, (_, col) =>
    allocation[row].reduce((sum, value, i) => sum + value * allocation[col][i], 0) + (row === col ? .002 : 0)));
  const gramInverse = invert(gram);
  return demand => {
    const dual = matVec(gramInverse, demand);
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
    this.mass = profile.mass;
    this.inertia = [...profile.inertia];
    this.addedMass = [...profile.addedMass];
    this.totalMass = [this.mass + this.addedMass[0], this.mass + this.addedMass[1], this.mass + this.addedMass[2], ...this.inertia.map((v, i) => v + this.addedMass[i + 3])];
    this.linearDamping = [...profile.linearDamping];
    this.quadraticDamping = [...profile.quadraticDamping];
    this.weight = this.mass * G;
    this.buoyancy = this.weight * 1.006;
    this.rG = [0, 0, profile.cg];
    this.rB = [0, 0, 0];
    this.allocate = makeAllocator(profile);
    this.thrusters = new Array(profile.thrusters.length).fill(0);
    this.thrusterTarget = new Array(profile.thrusters.length).fill(0);
  }

  reset() {
    this.state.fill(0);
    this.state[8] = 2;
    this.state[11] = 0.35;
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

  derivative(state, commands) {
    const nu = state.slice(0, 6), eta = state.slice(6);
    const [phi, theta, psi] = eta.slice(3);
    const uc = this.current.speed * Math.cos(this.current.direction - psi);
    const vc = this.current.speed * Math.sin(this.current.direction - psi);
    const nuCurrent = [uc, vc, 0, 0, 0, 0];
    const nuR = sub(nu, nuCurrent);
    const currentDot = [nu[5] * vc, -nu[5] * uc, 0, 0, 0, 0];

    const crb = rigidBodyCoriolis(this.mass, this.inertia, nu.slice(3));
    const ca = blockCoriolis(this.addedMass, nuR);
    const coriolis = add(matVec(crb, nuR), matVec(ca, nuR));
    const damping = nuR.map((v, i) => this.linearDamping[i] * v + this.quadraticDamping[i] * Math.abs(v) * v);

    const R = rotation(phi, theta, psi);
    const dW = this.weight - this.buoyancy;
    const rgWrbB = sub(scale(this.rG, this.weight), scale(this.rB, this.buoyancy));
    const gvec = [
      -dW * R[2][0], -dW * R[2][1], -dW * R[2][2],
      -rgWrbB[1] * R[2][2] + rgWrbB[2] * R[2][1],
      -rgWrbB[2] * R[2][0] + rgWrbB[0] * R[2][2],
      -rgWrbB[0] * R[2][1] + rgWrbB[1] * R[2][0],
    ];
    const tau = thrusterWrench(commands, this.profile);
    const nuDot = tau.map((v, i) => currentDot[i] + (v - coriolis[i] - damping[i] - gvec[i]) / this.totalMass[i]);
    const posDot = matVec(R, nu.slice(0, 3));
    const angleDot = eulerRates(phi, theta, nu.slice(3));
    return [...nuDot, ...posDot, ...angleDot];
  }

  step(dt) {
    const actuatorAlpha = 1 - Math.exp(-dt / this.profile.actuatorTau);
    this.thrusters = this.thrusters.map((v, i) => v + (this.thrusterTarget[i] - v) * actuatorAlpha);
    const f = x => this.derivative(x, this.thrusters);
    const x = this.state;
    const k1 = f(x);
    const k2 = f(add(x, scale(k1, dt * 0.5)));
    const k3 = f(add(x, scale(k2, dt * 0.5)));
    const k4 = f(add(x, scale(k3, dt)));
    this.state = x.map((v, i) => v + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
    this.state[8] = clamp(this.state[8], 0.15, 34);
    this.state[10] = clamp(this.state[10], -1.35, 1.35);
    this.state[9] = wrapPi(this.state[9]);
    this.state[11] = wrapPi(this.state[11]);
    this.time += dt;
  }
}
