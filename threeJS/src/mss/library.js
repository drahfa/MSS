// Line-by-line JavaScript ports of the MSS LIBRARY functions used by the ROV
// plant. Each function keeps the MSS name, argument order, and conventions so
// it can be compared directly against the MATLAB/Octave source. Parity is
// enforced by test/mss-library.test.js using fixtures produced by running the
// original .m files in GNU Octave (validation/mss/generate_fixtures.m).
//
// Matrices are arrays of row arrays; vectors are plain arrays.

// ---------------------------------------------------------------------------
// Matrix helpers (MATLAB operators)

export const zeros = (rows, cols) => Array.from({ length: rows }, () => new Array(cols).fill(0));
export const eye = n => Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => +(i === j)));
export const diag = v => v.map((x, i) => v.map((_, j) => (i === j ? x : 0)));
export const transpose = A => A[0].map((_, j) => A.map(row => row[j]));
export const matAdd = (A, B) => A.map((row, i) => row.map((x, j) => x + B[i][j]));
export const matSub = (A, B) => A.map((row, i) => row.map((x, j) => x - B[i][j]));
export const matScale = (A, s) => A.map(row => row.map(x => x * s));
export const vecAdd = (a, b) => a.map((x, i) => x + b[i]);
export const vecSub = (a, b) => a.map((x, i) => x - b[i]);
export const vecScale = (a, s) => a.map(x => x * s);

export function matMul(A, B) {
  const n = A.length, k = B.length, m = B[0].length;
  const C = zeros(n, m);
  for (let i = 0; i < n; i++) {
    for (let p = 0; p < k; p++) {
      const a = A[i][p];
      if (a === 0) continue;
      for (let j = 0; j < m; j++) C[i][j] += a * B[p][j];
    }
  }
  return C;
}

export function matVec(A, x) {
  return A.map(row => {
    let sum = 0;
    for (let j = 0; j < row.length; j++) sum += row[j] * x[j];
    return sum;
  });
}

/** Assemble a 2x2 block matrix [A B; C D]. */
export function blocks(A, B, C, D) {
  return [...A.map((row, i) => [...row, ...B[i]]), ...C.map((row, i) => [...row, ...D[i]])];
}

/** inv(A) by Gauss-Jordan elimination with partial pivoting. */
export function inv(A) {
  const n = A.length;
  const a = A.map((row, i) => [...row, ...Array.from({ length: n }, (_, j) => +(i === j))]);
  for (let col = 0; col < n; col++) {
    let pivot = col;
    for (let row = col + 1; row < n; row++) if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) pivot = row;
    if (Math.abs(a[pivot][col]) < 1e-300) throw new Error('Matrix is singular to working precision');
    [a[col], a[pivot]] = [a[pivot], a[col]];
    const divisor = a[col][col];
    for (let j = 0; j < 2 * n; j++) a[col][j] /= divisor;
    for (let row = 0; row < n; row++) {
      if (row === col) continue;
      const factor = a[row][col];
      if (factor === 0) continue;
      for (let j = 0; j < 2 * n; j++) a[row][j] -= factor * a[col][j];
    }
  }
  return a.map(row => row.slice(n));
}

/** x = A \ b by Gaussian elimination with partial pivoting (MATLAB mldivide for square A). */
export function mldivide(A, b) {
  const n = A.length;
  const a = A.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < n; col++) {
    let pivot = col;
    for (let row = col + 1; row < n; row++) if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) pivot = row;
    if (Math.abs(a[pivot][col]) < 1e-300) throw new Error('Matrix is singular to working precision');
    [a[col], a[pivot]] = [a[pivot], a[col]];
    for (let row = col + 1; row < n; row++) {
      const factor = a[row][col] / a[col][col];
      if (factor === 0) continue;
      for (let j = col; j <= n; j++) a[row][j] -= factor * a[col][j];
    }
  }
  const x = new Array(n).fill(0);
  for (let i = n - 1; i >= 0; i--) {
    let sum = a[i][n];
    for (let j = i + 1; j < n; j++) sum -= a[i][j] * x[j];
    x[i] = sum / a[i][i];
  }
  return x;
}

// ---------------------------------------------------------------------------
// LIBRARY/kinematics

/** Smtrx.m — skew-symmetric matrix S(a) such that S(a) * b = cross(a, b). */
export function Smtrx(a) {
  return [
    [0, -a[2], a[1]],
    [a[2], 0, -a[0]],
    [-a[1], a[0], 0],
  ];
}

/** Hmtrx.m — 6x6 system transformation matrix H(r) = [eye(3) S(r)'; zeros(3) eye(3)]. */
export function Hmtrx(r) {
  return blocks(eye(3), transpose(Smtrx(r)), zeros(3, 3), eye(3));
}

/** Rzyx.m — rotation matrix in SO(3) using the zyx convention. */
export function Rzyx(phi, theta, psi) {
  const cphi = Math.cos(phi), sphi = Math.sin(phi);
  const cth = Math.cos(theta), sth = Math.sin(theta);
  const cpsi = Math.cos(psi), spsi = Math.sin(psi);
  return [
    [cpsi * cth, -spsi * cphi + cpsi * sth * sphi, spsi * sphi + cpsi * cphi * sth],
    [spsi * cth, cpsi * cphi + sphi * sth * spsi, -cpsi * sphi + sth * spsi * cphi],
    [-sth, cth * sphi, cth * cphi],
  ];
}

/** Tzyx.m — Euler-angle attitude transformation matrix using the zyx convention. */
export function Tzyx(phi, theta) {
  const cphi = Math.cos(phi), sphi = Math.sin(phi);
  const cth = Math.cos(theta), sth = Math.sin(theta);
  if (cth === 0) throw new Error('Tzyx is singular for theta = +-90 degrees');
  return [
    [1, sphi * sth / cth, cphi * sth / cth],
    [0, cphi, -sphi],
    [0, sphi / cth, cphi / cth],
  ];
}

/** eulerang.m — returns { J, J1, J2 } with J = blkdiag(Rzyx, Tzyx). */
export function eulerang(phi, theta, psi) {
  const J1 = Rzyx(phi, theta, psi);
  const J2 = Tzyx(phi, theta);
  return { J: blocks(J1, zeros(3, 3), zeros(3, 3), J2), J1, J2 };
}

/** ssa.m — smallest signed angle, maps to [-pi, pi) using MATLAB mod semantics. */
export function ssa(angle) {
  const y = 2 * Math.PI;
  const a = angle + Math.PI;
  return a - Math.floor(a / y) * y - Math.PI;
}

/** euler2q.m — Euler angles [phi, theta, psi] to unit quaternion [eta, eps1, eps2, eps3]. */
export function euler2q(phi, theta, psi) {
  const c1 = Math.cos(phi * 0.5), s1 = Math.sin(phi * 0.5);
  const c2 = Math.cos(theta * 0.5), s2 = Math.sin(theta * 0.5);
  const c3 = Math.cos(psi * 0.5), s3 = Math.sin(psi * 0.5);
  return [
    c1 * c2 * c3 + s1 * s2 * s3,
    s1 * c2 * c3 - c1 * s2 * s3,
    c1 * s2 * c3 + s1 * c2 * s3,
    c1 * c2 * s3 - s1 * s2 * c3,
  ];
}

/** q2euler.m — unit quaternion [eta, eps1, eps2, eps3] to Euler angles [phi, theta, psi]. */
export function q2euler(q) {
  const [eta, e1, e2, e3] = q;
  const phi = Math.atan2(2 * (eta * e1 + e2 * e3), 1 - 2 * (e1 * e1 + e2 * e2));
  const sinTheta = Math.min(1, Math.max(-1, 2 * (eta * e2 - e3 * e1)));
  const theta = Math.asin(sinTheta);
  const psi = Math.atan2(2 * (eta * e3 + e1 * e2), 1 - 2 * (e2 * e2 + e3 * e3));
  return [phi, theta, psi];
}

/** Rotation matrix R in SO(3) directly from unit quaternion [eta, eps1, eps2, eps3]. */
export function Rquat(q) {
  const [eta, e1, e2, e3] = q;
  return [
    [1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - eta * e3), 2 * (e1 * e3 + eta * e2)],
    [2 * (e1 * e2 + eta * e3), 1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - eta * e1)],
    [2 * (e1 * e3 - eta * e2), 2 * (e2 * e3 + eta * e1), 1 - 2 * (e1 * e1 + e2 * e2)],
  ];
}

/** Quaternion kinematic rate qdot = 0.5 * E(q) * omega. */
export function qdot(q, omega) {
  const [eta, e1, e2, e3] = q;
  const [p, qm, r] = omega;
  return [
    -0.5 * (e1 * p + e2 * qm + e3 * r),
     0.5 * (eta * p - e3 * qm + e2 * r),
     0.5 * (e3 * p + eta * qm - e1 * r),
     0.5 * (-e2 * p + e1 * qm + eta * r),
  ];
}

/** Normalize quaternion to preserve unit length on S^3. */
export function normalizeQuat(q) {
  const norm = Math.hypot(q[0], q[1], q[2], q[3]);
  return norm > 1e-12 ? [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm] : [1, 0, 0, 0];
}

// ---------------------------------------------------------------------------
// LIBRARY/modeling

/**
 * rbody.m — rigid-body mass and Coriolis-centripetal matrices about the CO.
 * R44, R55, R66 are radii of gyration about the CG, nu2 = [p q r], and r_bG
 * is the CG position with respect to the CO.
 */
export function rbody(m, R44, R55, R66, nu2, r_bG) {
  const I3 = eye(3), O3 = zeros(3, 3);
  const I_G = matScale(diag([R44 ** 2, R55 ** 2, R66 ** 2]), m);
  const MRB_CG = blocks(matScale(I3, m), O3, O3, I_G);
  const CRB_CG = blocks(matScale(Smtrx(nu2), m), O3, O3, matScale(Smtrx(matVec(I_G, nu2)), -1));
  const H = Hmtrx(r_bG);
  const Ht = transpose(H);
  return {
    MRB: matMul(matMul(Ht, MRB_CG), H),
    CRB: matMul(matMul(Ht, CRB_CG), H),
  };
}

/** m2c.m — Coriolis-centripetal matrix C(nu) from a 6x6 system inertia matrix M. */
export function m2c(M, nu) {
  if (nu.length !== 6) throw new Error('m2c: only the 6-DOF form is ported');
  const Ms = matScale(matAdd(M, transpose(M)), 0.5);
  const M11 = Ms.slice(0, 3).map(r => r.slice(0, 3));
  const M12 = Ms.slice(0, 3).map(r => r.slice(3, 6));
  const M21 = transpose(M12);
  const M22 = Ms.slice(3, 6).map(r => r.slice(3, 6));
  const nu1 = nu.slice(0, 3), nu2 = nu.slice(3, 6);
  const nu1_dot = vecAdd(matVec(M11, nu1), matVec(M12, nu2));
  const nu2_dot = vecAdd(matVec(M21, nu1), matVec(M22, nu2));
  const S1 = matScale(Smtrx(nu1_dot), -1);
  const S2 = matScale(Smtrx(nu2_dot), -1);
  return blocks(zeros(3, 3), S1, S1, S2);
}

/** gRvect.m — restoring forces about the CO for a submerged body using R. */
export function gRvect(W, B, R, r_bG, r_bB) {
  const r1 = r_bG[0] * W - r_bB[0] * B;
  const r2 = r_bG[1] * W - r_bB[1] * B;
  const r3 = r_bG[2] * W - r_bB[2] * B;
  return [
    -(W - B) * R[2][0],
    -(W - B) * R[2][1],
    -(W - B) * R[2][2],
    -r2 * R[2][2] + r3 * R[2][1],
    -r3 * R[2][0] + r1 * R[2][2],
    -r1 * R[2][1] + r2 * R[2][0],
  ];
}

// ---------------------------------------------------------------------------
// LIBRARY/numericalMethods

/** rk4.m — x[k+1] for xdot = fn(x, ...args) with step h. */
export function rk4(fn, h, x, ...args) {
  const k1 = fn(x, ...args);
  const k2 = fn(vecAdd(x, vecScale(k1, 0.5 * h)), ...args);
  const k3 = fn(vecAdd(x, vecScale(k2, 0.5 * h)), ...args);
  const k4 = fn(vecAdd(x, vecScale(k3, h)), ...args);
  return x.map((v, i) => v + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
}

// ---------------------------------------------------------------------------
// LIBRARY/mssConstants.m and GNC/sat.m

export function mssConstants() {
  return { rho_water: 1025, rho_air: 1.225, g: 9.81 };
}

export function sat(x, x_max) {
  if (x_max <= 0) throw new Error('x_max must contain positive elements.');
  return Math.min(Math.max(x, -x_max), x_max);
}
