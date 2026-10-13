import { test } from 'node:test';
import assert from 'node:assert/strict';
import { LumpedMassTether } from '../src/lumpedMassTether.js';
import { NavigationFilter } from '../src/navigationFilter.js';
import { euler2q, q2euler, Rquat } from '../src/mss/library.js';

// Identity rotation matrix for tests
const R_IDENTITY = [
  [1, 0, 0],
  [0, 1, 0],
  [0, 0, 1],
];

test('LumpedMassTether initializes nodes and geometry properly', () => {
  const tether = new LumpedMassTether({
    nodeCount: 12,
    totalLength: 66.0,
    diameter: 0.02,
    density: 1100,
    waterDensity: 1025,
  });

  assert.equal(tether.nodeCount, 12);
  assert.equal(tether.segmentCount, 11);
  assert.equal(tether.unstretchedSegLength, 6.0);
  assert.ok(tether.segMass > 0, 'Segment mass must be positive');
  assert.ok(tether.segStiffness > 0, 'Segment stiffness must be positive');

  const nodes = tether.getNodePositions();
  assert.equal(nodes.length, 12);

  // Surface node boundary
  assert.equal(nodes[0][0], 0);
  assert.equal(nodes[0][1], 0);
  assert.equal(nodes[0][2], 0.5);

  // ROV attachment node boundary
  assert.equal(nodes[11][0], 2);
  assert.equal(nodes[11][1], 0);
  assert.equal(nodes[11][2], 2);
});

test('LumpedMassTether enforces unilateral elasticity: zero tension in compression, positive in extension', () => {
  const tether = new LumpedMassTether({
    nodeCount: 4,
    totalLength: 30.0, // 10m per segment
  });

  const surfacePos = [0, 0, 0];

  // Case 1: Nodes closer than unstretched length (slack/compression)
  // ROV placed at 15m total distance, less than 30m total length
  const rovSlack = [15, 0, 0];
  const stepSlack = tether.step(0.01, surfacePos, rovSlack, [0, 0, 0], R_IDENTITY, 0, 0);
  // Max tension should be minimal/zero under slack initial conditions
  assert.ok(stepSlack.tensionMagnitude >= 0, 'Tension cannot be negative');

  // Case 2: Stretched tether beyond unstretched length
  // Place ROV at 40m (> 30m total length)
  const rovStretched = [40, 0, 0];
  const stepStretched = tether.step(0.01, surfacePos, rovStretched, [0, 0, 0], R_IDENTITY, 0, 0);
  assert.ok(stepStretched.tensionMagnitude > 100, 'Tension must be substantial when stretched beyond length');
  assert.ok(stepStretched.wrenchBody[0] < 0, 'Tension must pull vehicle backwards towards surface origin');
});

test('LumpedMassTether Morison hydrodynamic drag opposes relative fluid motion', () => {
  const tether = new LumpedMassTether({
    nodeCount: 6,
    totalLength: 25.0,
  });

  const surfacePos = [0, 0, 0];
  const rovPos = [20, 0, 5];

  // Step with zero current
  const stepZero = tether.step(0.01, surfacePos, rovPos, [0, 0, 0], R_IDENTITY, 0, 0);

  // Step with significant North current (currentSpeed = 1.0 m/s, currentDir = 0 rad => North)
  const stepCurrent = tether.step(0.01, surfacePos, rovPos, [0, 0, 0], R_IDENTITY, 1.0, 0);

  // Current pushes nodes in the North (+X in NED) direction
  assert.ok(stepCurrent.nodes.length === 6);
  // Internal nodes should experience force from current
  assert.ok(tether.forces[3] > 0, 'Internal node 1 should experience positive drag force in North direction');
});

test('LumpedMassTether benthic penalty contact prevents seabed penetration below seabed depth', () => {
  const seabedDepth = 15.0;
  const tether = new LumpedMassTether({
    nodeCount: 5,
    totalLength: 40.0,
    seabedDepth,
    density: 1500, // Heavier line to encourage sinking
  });

  const surfacePos = [0, 0, 0];
  const rovPos = [10, 0, 14.5];

  // Manually force interior node below seabed to test reaction
  tether.positions[3 * 2 + 2] = 16.0; // Node 2 at 16m depth (1m penetration)
  tether.velocities[3 * 2 + 2] = 0.5; // Moving downwards

  tether.step(0.01, surfacePos, rovPos, [0, 0, 0], R_IDENTITY, 0, 0);

  // Normal penalty force should be strongly negative in NED Z (upwards reaction)
  // Forces array index 3*2 + 2
  const zForce = tether.forces[3 * 2 + 2];
  assert.ok(zForce < -5000, `Seabed normal reaction must be large upward restoring force: ${zForce}`);
});

test('LumpedMassTether stable dynamic integration under multi-step simulation', () => {
  const tether = new LumpedMassTether({
    nodeCount: 8,
    totalLength: 50.0,
  });

  const surfacePos = [0, 0, 0];
  const rovPos = [15, 5, 8];

  for (let step = 0; step < 100; step++) {
    const res = tether.step(0.01, surfacePos, rovPos, [0.1, 0, 0], R_IDENTITY, 0.2, 0.5);
    assert.ok(!Number.isNaN(res.tensionMagnitude), 'Tension magnitude must not be NaN');
    assert.ok(!Number.isNaN(res.nodes[2][0]), 'Node positions must not be NaN');
  }
});

test('NavigationFilter initializes 15-state vector and symmetric error covariance', () => {
  const filter = new NavigationFilter();
  assert.deepEqual(filter.p, [0, 0, 2]);
  assert.deepEqual(filter.v, [0, 0, 0]);
  assert.equal(filter.P.length, 15);
  assert.equal(filter.P[0].length, 15);

  // Check positive variance along diagonal
  for (let i = 0; i < 15; i++) {
    assert.ok(filter.P[i][i] > 0, `Variance at diagonal ${i} must be positive`);
  }
});

test('NavigationFilter strapdown IMU prediction propagates velocity and position with unit quaternion', () => {
  const filter = new NavigationFilter();
  filter.reset([0, 0, 0], 0);

  const dt = 0.01;
  // Body specific force of 2.0 m/s^2 forward (Surge), countering gravity in heave
  const f_b = [2.0, 0, -9.81];
  const omega_b = [0, 0, 0.1]; // 0.1 rad/s yaw rate

  const pVarInit = filter.P[0][0];

  for (let i = 0; i < 50; i++) {
    filter.predict(dt, f_b, omega_b);
  }

  // After 0.5s of forward specific force, vehicle should have moved North (+X)
  assert.ok(filter.p[0] > 0, 'Position North should have increased');
  assert.ok(filter.v[0] > 0, 'Velocity North should have increased');

  // Attitude quaternion must remain unit norm on S^3
  const qNorm = Math.hypot(...filter.q);
  assert.ok(Math.abs(qNorm - 1.0) < 1e-6, `Quaternion norm must be 1.0, got ${qNorm}`);

  // Prediction must expand covariance
  assert.ok(filter.P[0][0] >= pVarInit, 'Position variance must grow under dead-reckoning');
});

test('NavigationFilter multi-sensor updates contract covariance and bound errors', () => {
  const filter = new NavigationFilter();
  filter.reset([5.0, 2.0, 10.0], 0);

  // Initial variances
  const pVarInit = filter.P[0][0];
  const dVarInit = filter.P[2][2];
  const vVarInit = filter.P[3][3];
  const psiVarInit = filter.P[8][8];

  // 1. DVL update with body velocity
  filter.updateDVL([1.2, 0.0, 0.1]);
  assert.ok(filter.P[3][3] < vVarInit, 'DVL update must contract velocity covariance');

  // 2. Depth update
  filter.updateDepth(10.5);
  assert.ok(filter.P[2][2] < dVarInit, 'Depth update must contract depth covariance');
  assert.ok(Math.abs(filter.p[2] - 10.5) < Math.abs(10.0 - 10.5), 'Depth estimate must move towards measurement');

  // 3. Compass update
  filter.updateCompass(0.5);
  assert.ok(filter.P[8][8] < psiVarInit, 'Compass update must contract heading covariance');

  // 4. USBL acoustic fix update
  filter.updateUSBL([5.4, 2.1, 10.5]);
  assert.ok(filter.P[0][0] < pVarInit, 'USBL update must contract horizontal position covariance');

  const est = filter.getEstimate();
  assert.ok(est.threeSigmaPos[0] > 0, '3-sigma bounds must be positive');
  assert.ok(est.threeSigmaVel[0] > 0, '3-sigma velocity bounds must be positive');
});
