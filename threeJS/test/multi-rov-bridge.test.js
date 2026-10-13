import { test } from 'node:test';
import assert from 'node:assert/strict';
import { MultiRovTetherBridge, segmentSegmentDistance3D } from '../src/multiRovTetherBridge.js';
import { euler2q } from '../src/mss/library.js';

test('segmentSegmentDistance3D computes exact 3D distances between segments', () => {
  // Parallel segments separated along Y by 4.0 meters
  const P1 = [0, 0, 0], P2 = [10, 0, 0];
  const Q1 = [0, 4, 0], Q2 = [10, 4, 0];
  const distParallel = segmentSegmentDistance3D(P1, P2, Q1, Q2);
  assert.equal(Math.round(distParallel), 4);

  // Orthogonal skew segments: P along X at z=0, Q along Y at z=3
  const P3 = [-5, 0, 0], P4 = [5, 0, 0];
  const Q3 = [0, -5, 3], Q4 = [0, 5, 3];
  const distOrthogonal = segmentSegmentDistance3D(P3, P4, Q3, Q4);
  assert.equal(Math.round(distOrthogonal), 3);
});

test('MultiRovTetherBridge detects dual-tether clearance and triggers advisory and critical alarms', () => {
  const bridge = new MultiRovTetherBridge({ warnDistance: 3.0, critDistance: 1.0 });

  // Tether A: straight down along Z at (0, 0)
  const tetherA = [
    [0, 0, 0],
    [0, 0, 5],
    [0, 0, 10],
    [0, 0, 15],
  ];

  // Tether B: well separated at East = 8.0 m
  const tetherBSafe = [
    [0, 8, 0],
    [0, 8, 5],
    [0, 8, 10],
    [0, 8, 15],
  ];
  const resSafe = bridge.computeTetherClearance(tetherA, tetherBSafe);
  assert.equal(resSafe.minDistance, 8.0);
  assert.equal(resSafe.advisory, false);
  assert.equal(resSafe.criticalAlert, false);

  // Tether B approaching in warning envelope at East = 2.2 m
  const tetherBWarn = [
    [0, 2.2, 0],
    [0, 2.2, 5],
    [0, 2.2, 10],
    [0, 2.2, 15],
  ];
  const resWarn = bridge.computeTetherClearance(tetherA, tetherBWarn);
  assert.equal(resWarn.minDistance, 2.2);
  assert.equal(resWarn.advisory, true);
  assert.equal(resWarn.criticalAlert, false);

  // Tether B critical crossover at East = 0.6 m
  const tetherBCrit = [
    [0, 0.6, 0],
    [0, 0.6, 5],
    [0, 0.6, 10],
    [0, 0.6, 15],
  ];
  const resCrit = bridge.computeTetherClearance(tetherA, tetherBCrit);
  assert.equal(resCrit.minDistance, 0.6);
  assert.equal(resCrit.advisory, true);
  assert.equal(resCrit.criticalAlert, true);
});

test('MultiRovTetherBridge evaluates cooperative payload distance and restoring tension', () => {
  const bridge = new MultiRovTetherBridge();

  const posA = [0, 0, 5];
  const posB_exact = [6.0, 0, 5]; // Exactly 6.0 m length

  const resExact = bridge.evaluateCooperativePayload(posA, posB_exact, 6.0, 100.0, 10000);
  assert.equal(resExact.distance, 6.0);
  assert.equal(resExact.error, 0);
  assert.equal(resExact.tension, 0);
  assert.ok(resExact.forceOnA[2] > 0, 'Must support half the submerged gravity load');
  assert.equal(resExact.forceOnA[2], resExact.forceOnB[2]);

  // Vehicle B drifts forward to 6.1 meters (0.1m stretch)
  const posB_stretch = [6.1, 0, 5];
  const resStretch = bridge.evaluateCooperativePayload(posA, posB_stretch, 6.0, 100.0, 10000);
  assert.equal(resStretch.distance, 6.1);
  assert.equal(resStretch.error, 0.1);
  assert.equal(resStretch.tension, 1000.0); // 10000 N/m * 0.1 m = 1000 N
  // Force on A pulls forward (+X), force on B pulls backward (-X)
  assert.ok(resStretch.forceOnA[0] > 990, 'Force on A must pull forward along +X');
  assert.ok(resStretch.forceOnB[0] < -990, 'Force on B must pull backward along -X');
});

test('MultiRovTetherBridge serializes standard ROS 2 Odometry, Imu, and Twist messages', () => {
  const bridge = new MultiRovTetherBridge();
  const q = euler2q(0.1, 0.2, 0.3);
  const state = [1.2, 0.3, -0.4, 0.05, -0.02, 0.1, 10.0, 5.0, 8.0, q[0], q[1], q[2], q[3]];

  // 1. Odometry
  const odom = bridge.serializeRos2Message('Odometry', state, null, 12.34);
  assert.equal(odom.header.frame_id, 'world_ned');
  assert.equal(odom.child_frame_id, 'base_link');
  assert.equal(odom.pose.pose.position.x, 10.0);
  assert.equal(odom.pose.pose.position.z, 8.0);
  assert.equal(odom.twist.twist.linear.x, 1.2);

  // 2. Imu
  const imu = bridge.serializeRos2Message('Imu', state, null, 12.34);
  assert.equal(imu.header.frame_id, 'base_link');
  assert.equal(imu.angular_velocity.x, 0.05);

  // 3. Twist
  const twist = bridge.serializeRos2Message('Twist', state);
  assert.equal(twist.linear.x, 1.2);
  assert.equal(twist.linear.y, 0.3);
});

test('MultiRovTetherBridge serializes MAVLink telemetry and executes lossless command round-trip', () => {
  const bridge = new MultiRovTetherBridge();
  const q = euler2q(0, 0, 0.5);
  const state = [1.0, 0, 0, 0, 0, 0, 5.0, 2.0, 4.0, q[0], q[1], q[2], q[3]];
  const thrusters = [0.5, -0.5, 0.25, -0.25];

  // 1. Heartbeat
  const hb = bridge.serializeMavlinkMessage('HEARTBEAT', state);
  assert.equal(hb.type, 12); // MAV_TYPE_SUBMARINE

  // 2. Attitude
  const att = bridge.serializeMavlinkMessage('ATTITUDE', state, thrusters, 5000);
  assert.equal(att.time_boot_ms, 5000);
  assert.ok(Math.abs(att.yaw - 0.5) < 0.001);

  // 3. Servo Output Raw
  const servos = bridge.serializeMavlinkMessage('SERVO_OUTPUT_RAW', state, thrusters, 5000);
  assert.equal(servos.servo1_raw, 1700); // 1500 + 0.5 * 400
  assert.equal(servos.servo2_raw, 1300); // 1500 - 0.5 * 400

  // 4. Command Override Round-Trip
  const demandsIn = { surge: 0.75, sway: -0.5, heave: 0.25, yaw: -0.8, pitch: 0.1, roll: -0.1 };
  const encodedPwm = bridge.encodeMavlinkOverride(demandsIn);
  assert.equal(encodedPwm.length, 8);

  const decodedDemands = bridge.parseMavlinkOverride(encodedPwm);
  assert.ok(Math.abs(decodedDemands.surge - demandsIn.surge) < 0.005);
  assert.ok(Math.abs(decodedDemands.sway - demandsIn.sway) < 0.005);
  assert.ok(Math.abs(decodedDemands.heave - demandsIn.heave) < 0.005);
  assert.ok(Math.abs(decodedDemands.yaw - demandsIn.yaw) < 0.005);
});
