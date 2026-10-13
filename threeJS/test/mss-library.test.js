import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  Smtrx, Hmtrx, Rzyx, Tzyx, eulerang, ssa, rbody, m2c, gRvect, rk4, mldivide, inv, matMul, eye, diag,
  euler2q, q2euler, Rquat, qdot, normalizeQuat
} from '../src/mss/library.js';

function assertMatrixClose(actual, expected, eps = 1e-7) {
  assert.equal(actual.length, expected.length, 'Matrix row count mismatch');
  for (let i = 0; i < actual.length; i++) {
    assert.equal(actual[i].length, expected[i].length, `Matrix col count mismatch at row ${i}`);
    for (let j = 0; j < actual[i].length; j++) {
      const diff = Math.abs(actual[i][j] - expected[i][j]);
      assert.ok(diff < eps, `Matrix mismatch at (${i},${j}): actual ${actual[i][j]} vs expected ${expected[i][j]}`);
    }
  }
}

function assertVecClose(actual, expected, eps = 1e-7) {
  assert.equal(actual.length, expected.length, 'Vector length mismatch');
  for (let i = 0; i < actual.length; i++) {
    const diff = Math.abs(actual[i] - expected[i]);
    assert.ok(diff < eps, `Vector mismatch at [${i}]: actual ${actual[i]} vs expected ${expected[i]}`);
  }
}

test('Smtrx creates skew-symmetric cross-product matrix', () => {
  const v = [1, 2, 3];
  const S = Smtrx(v);
  // S(a) * b == a x b
  const b = [4, 5, 6];
  // 1*4+2*5+3*6... cross product: [2*6 - 3*5, 3*4 - 1*6, 1*5 - 2*4] = [-3, 6, -3]
  const Sb = [
    S[0][0]*b[0] + S[0][1]*b[1] + S[0][2]*b[2],
    S[1][0]*b[0] + S[1][1]*b[1] + S[1][2]*b[2],
    S[2][0]*b[0] + S[2][1]*b[1] + S[2][2]*b[2],
  ];
  assertVecClose(Sb, [-3, 6, -3]);
});

test('Rzyx is orthogonal and det(R) = 1', () => {
  const R = Rzyx(0.1, 0.2, 0.3);
  const Rt = R[0].map((_, j) => R.map(row => row[j]));
  const RRt = matMul(R, Rt);
  assertMatrixClose(RRt, eye(3));
});

test('eulerang returns matching J1=Rzyx and J2=Tzyx', () => {
  const { J, J1, J2 } = eulerang(0.1, 0.2, 0.3);
  assert.equal(J.length, 6);
  assertMatrixClose(J1, Rzyx(0.1, 0.2, 0.3));
  assertMatrixClose(J2, Tzyx(0.1, 0.2));
});

test('ssa maps angles to [-pi, pi)', () => {
  assert.ok(Math.abs(ssa(Math.PI * 3) - (-Math.PI)) < 1e-9 || Math.abs(ssa(Math.PI * 3) - Math.PI) < 1e-9);
  assert.ok(Math.abs(ssa(0.5) - 0.5) < 1e-9);
  assert.ok(Math.abs(ssa(-3.5) - (2 * Math.PI - 3.5)) < 1e-9);
});

test('mldivide solves A*x = b exactly', () => {
  const A = [
    [10, 2, 3],
    [1, 8, 2],
    [2, -1, 7],
  ];
  const b = [15, 11, 8];
  const x = mldivide(A, b);
  const Ax = [
    A[0][0]*x[0] + A[0][1]*x[1] + A[0][2]*x[2],
    A[1][0]*x[0] + A[1][1]*x[1] + A[1][2]*x[2],
    A[2][0]*x[0] + A[2][1]*x[1] + A[2][2]*x[2],
  ];
  assertVecClose(Ax, b);
});

test('rbody generates symmetric positive definite MRB', () => {
  const m = 85;
  const R44 = Math.sqrt(8.5 / m);
  const R55 = Math.sqrt(12.8 / m);
  const R66 = Math.sqrt(14.2 / m);
  const nu2 = [0.1, 0.05, -0.2];
  const r_bG = [0, 0, 0.045];
  const { MRB, CRB } = rbody(m, R44, R55, R66, nu2, r_bG);

  // Check MRB is symmetric
  for (let i = 0; i < 6; i++) {
    for (let j = 0; j < 6; j++) {
      assert.ok(Math.abs(MRB[i][j] - MRB[j][i]) < 1e-9);
    }
  }

  // Check CRB is skew-symmetric
  for (let i = 0; i < 6; i++) {
    for (let j = 0; j < 6; j++) {
      assert.ok(Math.abs(CRB[i][j] + CRB[j][i]) < 1e-9);
    }
  }
});

test('m2c produces skew-symmetric Coriolis matrix for symmetric M', () => {
  const M = [
    [100, 5, 0, 0, 0, 0],
    [5, 120, 0, 0, 0, 0],
    [0, 0, 150, 0, 0, 0],
    [0, 0, 0, 20, 0, 0],
    [0, 0, 0, 0, 30, 0],
    [0, 0, 0, 0, 0, 40],
  ];
  const nu = [1.5, -0.5, 0.2, 0.1, -0.1, 0.3];
  const C = m2c(M, nu);
  for (let i = 0; i < 6; i++) {
    for (let j = 0; j < 6; j++) {
      assert.ok(Math.abs(C[i][j] + C[j][i]) < 1e-9, `m2c not skew-symmetric at (${i},${j})`);
    }
  }
});

test('rk4 integrates harmonic oscillator x_ddot + x = 0 accurately', () => {
  // state: [pos, vel], derivative: [vel, -pos]
  const osc = ([x, v]) => [v, -x];
  let state = [1, 0];
  const dt = 0.01;
  const steps = 100; // t = 1.0s
  for (let i = 0; i < steps; i++) {
    state = rk4(osc, dt, state);
  }
  // exact at t=1: pos = cos(1) ≈ 0.5403023, vel = -sin(1) ≈ -0.84147098
  assert.ok(Math.abs(state[0] - Math.cos(1)) < 1e-5);
  assert.ok(Math.abs(state[1] - (-Math.sin(1))) < 1e-5);
});

test('euler2q and q2euler preserve angles across normal and steep pitch angles', () => {
  const angles = [
    [0, 0, 0],
    [0.2, 0.4, 0.8],
    [-0.35, 0.15, -1.2],
    [0.1, 1.55, 0.5], // Steep pitch θ ≈ 88.8°
    [-0.2, -1.56, 1.1], // Steep negative pitch θ ≈ -89.4°
  ];

  for (const [phi, theta, psi] of angles) {
    const q = euler2q(phi, theta, psi);
    // Unit length check
    const norm = Math.hypot(...q);
    assert.ok(Math.abs(norm - 1.0) < 1e-9, `Quaternion not unit norm: ${norm}`);

    const [rPhi, rTheta, rPsi] = q2euler(q);
    assert.ok(Math.abs(rPhi - phi) < 1e-6, `Roll mismatch: ${rPhi} vs ${phi}`);
    assert.ok(Math.abs(rTheta - theta) < 1e-6, `Pitch mismatch: ${rTheta} vs ${theta}`);
    assert.ok(Math.abs(rPsi - psi) < 1e-6, `Yaw mismatch: ${rPsi} vs ${psi}`);
  }
});

test('Rquat produces orthogonal SO(3) matrix matching Rzyx', () => {
  const phi = 0.25, theta = -0.42, psi = 1.15;
  const q = euler2q(phi, theta, psi);
  const Rq = Rquat(q);
  const Rz = Rzyx(phi, theta, psi);

  // 1. Rq equals Rz
  assertMatrixClose(Rq, Rz, 1e-9);

  // 2. Rq is orthogonal: Rq^T * Rq = I
  const RqT = [
    [Rq[0][0], Rq[1][0], Rq[2][0]],
    [Rq[0][1], Rq[1][1], Rq[2][1]],
    [Rq[0][2], Rq[1][2], Rq[2][2]],
  ];
  const prod = matMul(RqT, Rq);
  assertMatrixClose(prod, eye(3), 1e-9);
});

test('qdot and rk4 integrate pure 3D rotation without gimbal singularity', () => {
  // Angular rate omega = [0, 0.5, 0] (pure pitching through and past 90 degrees)
  const omega = [0, 0.5, 0];
  let q = euler2q(0, 0, 0); // start horizontal
  const dt = 0.01;
  const steps = 400; // t = 4s -> pitch reaches 2.0 rad (114.6°), well beyond 90° gimbal lock!

  for (let i = 0; i < steps; i++) {
    q = rk4((currQ) => qdot(currQ, omega), dt, q);
    q = normalizeQuat(q);
    assert.ok(!q.some(isNaN), 'Quaternion became NaN during pitch maneuver');
    const norm = Math.hypot(...q);
    assert.ok(Math.abs(norm - 1.0) < 1e-9, `Quaternion norm drifted: ${norm}`);
  }

  // Pitch angle after 4s at 0.5 rad/s should be exactly 2.0 rad (inverted beyond 90°)
  const R = Rquat(q);
  assert.ok(!R.some(row => row.some(isNaN)), 'Rotation matrix contains NaN');
});

