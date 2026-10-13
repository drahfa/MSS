// Subsea Underwater Vehicle-Manipulator System (UVMS) Dynamics Engine.
// Paper 4 & Milestone 2.2 — Robotic Intervention Mechanics.

/**
 * Multiplies two 4x4 homogeneous transformation matrices.
 */
function mat4Mul(A, B) {
  const out = Array.from({ length: 4 }, () => new Array(4).fill(0));
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      let sum = 0;
      for (let k = 0; k < 4; k++) sum += A[i][k] * B[k][j];
      out[i][j] = sum;
    }
  }
  return out;
}

/**
 * Standard Denavit-Hartenberg (D-H) homogeneous transformation matrix.
 * @param {number} a Link length
 * @param {number} alpha Link twist (rad)
 * @param {number} d Joint offset
 * @param {number} theta Joint angle (rad)
 */
function dhTransform(a, alpha, d, theta) {
  const ct = Math.cos(theta);
  const st = Math.sin(theta);
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);

  return [
    [ct, -st * ca,  st * sa, a * ct],
    [st,  ct * ca, -ct * sa, a * st],
    [0,   sa,       ca,      d],
    [0,   0,        0,       1],
  ];
}

/**
 * Cross product of two 3D vectors.
 */
function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

/**
 * Inverts a 6x6 matrix using Gauss-Jordan elimination.
 */
function invert6x6(A) {
  const n = 6;
  const M = A.map((row, i) => [...row, ...Array.from({ length: n }, (_, j) => (i === j ? 1 : 0))]);

  for (let i = 0; i < n; i++) {
    let pivot = i;
    for (let k = i + 1; k < n; k++) {
      if (Math.abs(M[k][i]) > Math.abs(M[pivot][i])) pivot = k;
    }
    if (Math.abs(M[pivot][i]) < 1e-12) return null; // Singular

    if (pivot !== i) {
      const temp = M[i];
      M[i] = M[pivot];
      M[pivot] = temp;
    }

    const div = M[i][i];
    for (let j = 0; j < 2 * n; j++) M[i][j] /= div;

    for (let r = 0; r < n; r++) {
      if (r !== i) {
        const factor = M[r][i];
        for (let c = 0; c < 2 * n; c++) {
          M[r][c] -= factor * M[i][c];
        }
      }
    }
  }

  return M.map(row => row.slice(n));
}

function invert3x3(A) {
  const a = A[0][0], b = A[0][1], c = A[0][2];
  const d = A[1][0], e = A[1][1], f = A[1][2];
  const g = A[2][0], h = A[2][1], i = A[2][2];

  const A_ = e * i - f * h;
  const B_ = -(d * i - f * g);
  const C_ = d * h - e * g;

  const det = a * A_ + b * B_ + c * C_;
  if (Math.abs(det) < 1e-12) return null;

  const invDet = 1 / det;
  return [
    [A_ * invDet, (c * h - b * i) * invDet, (b * f - c * e) * invDet],
    [B_ * invDet, (a * i - c * g) * invDet, (c * d - a * f) * invDet],
    [C_ * invDet, (b * g - a * h) * invDet, (a * e - b * d) * invDet],
  ];
}

export class UvmsManipulator {
  /**
   * Constructs an n-DOF articulated subsea manipulator.
   * Default configuration: 3-DOF articulated intervention arm.
   */
  constructor(config = {}) {
    this.mountOffset = config.mountOffset ?? [0.45, 0.0, 0.15]; // [x, y, z] mounted on ROV front chin
    this.waterDensity = config.waterDensity ?? 1025.0; // kg/m^3
    this.cdArm = config.cdArm ?? 1.1; // Cylinder cross-flow drag

    // Link definitions: [a, alpha, d, theta0, mass, length, radius]
    this.links = config.links ?? [
      { a: 0.0,   alpha: -Math.PI / 2, d: 0.10, min: -Math.PI / 2, max: Math.PI / 2, mass: 2.5, length: 0.15, radius: 0.04 }, // Joint 1: Base azimuth
      { a: 0.35,  alpha: 0.0,          d: 0.0,  min: -Math.PI / 3, max: Math.PI / 2, mass: 3.2, length: 0.35, radius: 0.035 }, // Joint 2: Shoulder pitch
      { a: 0.28,  alpha: 0.0,          d: 0.0,  min: -Math.PI / 2, max: Math.PI / 2, mass: 2.1, length: 0.28, radius: 0.03 }, // Joint 3: Elbow pitch
    ];

    this.dof = this.links.length;
    this.q = new Float64Array(this.dof); // Joint angles
    this.qDot = new Float64Array(this.dof); // Joint velocities
    this.qDDot = new Float64Array(this.dof); // Joint accelerations

    // Default neutral pose
    this.setJointAngles(config.initialQ ?? [0.0, 0.2, 0.4]);
  }

  setJointAngles(qArray) {
    for (let i = 0; i < this.dof; i++) {
      const qVal = qArray[i] ?? 0;
      this.q[i] = Math.min(this.links[i].max, Math.max(this.links[i].min, qVal));
    }
  }

  /**
   * Computes forward kinematics for all links up to the end-effector.
   * @returns {object} { frames, eePos, eeRot, jointPositions }
   */
  computeForwardKinematics() {
    let T = [
      [1, 0, 0, this.mountOffset[0]],
      [0, 1, 0, this.mountOffset[1]],
      [0, 0, 1, this.mountOffset[2]],
      [0, 0, 0, 1],
    ];

    const frames = [T];
    const jointPositions = [[T[0][3], T[1][3], T[2][3]]];

    for (let i = 0; i < this.dof; i++) {
      const link = this.links[i];
      const Ti = dhTransform(link.a, link.alpha, link.d, this.q[i]);
      T = mat4Mul(T, Ti);
      frames.push(T);
      jointPositions.push([T[0][3], T[1][3], T[2][3]]);
    }

    const eePos = [T[0][3], T[1][3], T[2][3]];
    const eeRot = [
      [T[0][0], T[0][1], T[0][2]],
      [T[1][0], T[1][1], T[1][2]],
      [T[2][0], T[2][1], T[2][2]],
    ];

    return { frames, eePos, eeRot, jointPositions };
  }

  /**
   * Computes the 6 x n geometric Jacobian J(q) in the vehicle body frame.
   */
  computeJacobian(frames, eePos) {
    const J = Array.from({ length: 6 }, () => new Float64Array(this.dof));

    for (let i = 0; i < this.dof; i++) {
      const Ti = frames[i];
      const zi = [Ti[0][2], Ti[1][2], Ti[2][2]]; // z-axis of frame i
      const pi = [Ti[0][3], Ti[1][3], Ti[2][3]]; // origin of frame i

      const r = [eePos[0] - pi[0], eePos[1] - pi[1], eePos[2] - pi[2]];
      const jv = cross(zi, r);

      // Linear velocity contribution
      J[0][i] = jv[0];
      J[1][i] = jv[1];
      J[2][i] = jv[2];

      // Angular velocity contribution
      J[3][i] = zi[0];
      J[4][i] = zi[1];
      J[5][i] = zi[2];
    }

    return J;
  }

  /**
   * Solves inverse kinematics for a desired end-effector target using Damped Least Squares (DLS).
   * @param {number[]} targetPos Desired [x, y, z] target in body frame
   * @param {number} maxIter Maximum solver iterations
   * @param {number} tol Target position tolerance in meters
   * @param {number} lambda Damping factor
   */
  solveIK(targetPos, maxIter = 50, tol = 1e-4, lambda = 0.02) {
    for (let iter = 0; iter < maxIter; iter++) {
      const fk = this.computeForwardKinematics();
      const err = [
        targetPos[0] - fk.eePos[0],
        targetPos[1] - fk.eePos[1],
        targetPos[2] - fk.eePos[2],
      ];

      const errPosMag = Math.hypot(err[0], err[1], err[2]);
      if (errPosMag < tol) return { converged: true, iterations: iter, error: errPosMag };

      const J = this.computeJacobian(fk.frames, fk.eePos);
      // Top 3 rows of J (linear velocity)
      const Jv = [J[0], J[1], J[2]];

      // Jv * Jv^T + lambda^2 * I (3x3)
      const JJt = Array.from({ length: 3 }, () => new Float64Array(3));
      for (let r = 0; r < 3; r++) {
        for (let c = 0; c < 3; c++) {
          let sum = 0;
          for (let k = 0; k < this.dof; k++) sum += Jv[r][k] * Jv[c][k];
          JJt[r][c] = sum + (r === c ? lambda * lambda : 0);
        }
      }

      const invJJt = invert3x3(JJt);
      if (!invJJt) break;

      // Delta q = Jv^T * (Jv * Jv^T + lambda^2 * I)^-1 * err
      for (let k = 0; k < this.dof; k++) {
        let deltaQk = 0;
        for (let r = 0; r < 3; r++) {
          let invErrR = 0;
          for (let c = 0; c < 3; c++) invErrR += invJJt[r][c] * err[c];
          deltaQk += Jv[r][k] * invErrR;
        }

        // Clamp joint step for numerical stability
        const step = Math.min(0.25, Math.max(-0.25, deltaQk));
        this.q[k] = Math.min(this.links[k].max, Math.max(this.links[k].min, this.q[k] + step));
      }
    }

    const finalFk = this.computeForwardKinematics();
    const finalErr = Math.hypot(targetPos[0] - finalFk.eePos[0], targetPos[1] - finalFk.eePos[1], targetPos[2] - finalFk.eePos[2]);
    return { converged: finalErr < tol, iterations: maxIter, error: finalErr };
  }

  /**
   * Computes the instantaneous reaction wrench tau_react = [Fx, Fy, Fz, Mx, My, Mz]^T
   * exerted on the ROV base hull by manipulator motion, gravity/buoyancy, and Morison link drag.
   * @param {number[]} qDot Joint velocities (rad/s)
   * @param {number[]} qDDot Joint accelerations (rad/s^2)
   * @param {number[][]} R SO(3) matrix from body to NED
   * @param {number[]} vRelBody Relative fluid velocity at vehicle [u, v, w]
   */
  computeReactionWrench(qDot, qDDot, R, vRelBody = [0, 0, 0]) {
    for (let i = 0; i < this.dof; i++) {
      this.qDot[i] = qDot[i] ?? 0;
      this.qDDot[i] = qDDot[i] ?? 0;
    }

    const fk = this.computeForwardKinematics();
    const gBody = [R[2][0] * 9.81, R[2][1] * 9.81, R[2][2] * 9.81];

    let totalFx = 0, totalFy = 0, totalFz = 0;
    let totalMx = 0, totalMy = 0, totalMz = 0;

    for (let i = 0; i < this.dof; i++) {
      const link = this.links[i];
      const pJoint = fk.jointPositions[i];
      const pNext = fk.jointPositions[i + 1];

      // Approximate link center of mass
      const com = [
        (pJoint[0] + pNext[0]) / 2,
        (pJoint[1] + pNext[1]) / 2,
        (pJoint[2] + pNext[2]) / 2,
      ];

      // Inertial and gravitational forces
      // Displaced volume assuming aluminum/steel density ratio (buoyancy ~ 0.4 of mass)
      const linkBuoyancy = (this.waterDensity / 2700.0) * link.mass * 9.81;
      const fGravBuoy = [
        0,
        0,
        (link.mass * 9.81 - linkBuoyancy) * (R[2][2] > 0 ? 1 : -1) * (gBody[2] / 9.81),
      ];

      // Dynamic acceleration reaction: F_acc = -m * a_com
      const radDist = Math.hypot(com[0] - this.mountOffset[0], com[1] - this.mountOffset[1], com[2] - this.mountOffset[2]);
      const tanAcc = radDist * this.qDDot[i];
      const fInertia = [
        -link.mass * tanAcc * 0.7,
        -link.mass * tanAcc * 0.3,
        -link.mass * tanAcc * 0.1,
      ];

      // Hydrodynamic Morison drag on cylindrical link
      // Linear speed of link COM due to joint rotation
      const linkSpeed = radDist * Math.abs(this.qDot[i]);
      const frontalArea = 2 * link.radius * link.length;
      const fDragMag = 0.5 * this.waterDensity * this.cdArm * frontalArea * linkSpeed * linkSpeed;
      const fDrag = [
        -fDragMag * Math.sign(this.qDot[i]) * 0.6,
        -fDragMag * Math.sign(this.qDot[i]) * 0.4,
        0,
      ];

      // Sum link force exerted on base
      const fLinkTotal = [
        fInertia[0] + fDrag[0],
        fInertia[1] + fDrag[1],
        fInertia[2] + fGravBuoy[2],
      ];

      totalFx += fLinkTotal[0];
      totalFy += fLinkTotal[1];
      totalFz += fLinkTotal[2];

      // Reaction moments at vehicle origin: M = r x F
      const mLink = cross(com, fLinkTotal);
      totalMx += mLink[0];
      totalMy += mLink[1];
      totalMz += mLink[2];
    }

    return [totalFx, totalFy, totalFz, totalMx, totalMy, totalMz];
  }
}
