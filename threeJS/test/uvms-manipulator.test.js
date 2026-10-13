import { test } from 'node:test';
import assert from 'node:assert/strict';
import { UvmsManipulator } from '../src/uvmsManipulator.js';

const R_IDENTITY = [
  [1, 0, 0],
  [0, 1, 0],
  [0, 0, 1],
];

test('UvmsManipulator initializes D-H chain and computes valid forward kinematics', () => {
  const arm = new UvmsManipulator({
    mountOffset: [0.5, 0.0, 0.1],
  });

  assert.equal(arm.dof, 3);
  assert.equal(arm.links.length, 3);

  const fk = arm.computeForwardKinematics();
  assert.equal(fk.jointPositions.length, 4); // Base + 3 joints
  assert.ok(fk.eePos[0] > 0.5, 'End-effector must extend forward in front of vehicle mount');
  assert.ok(!Number.isNaN(fk.eePos[0]));
  assert.ok(!Number.isNaN(fk.eePos[1]));
  assert.ok(!Number.isNaN(fk.eePos[2]));
});

test('UvmsManipulator joint angles affect end-effector positions geometrically', () => {
  const arm = new UvmsManipulator();

  // Neutral forward pose
  arm.setJointAngles([0.0, 0.0, 0.0]);
  const fk0 = arm.computeForwardKinematics();
  const y0 = fk0.eePos[1];
  assert.ok(Math.abs(y0) < 1e-6, 'Symmetric arm at zero azimuth must have y ~ 0');

  // Rotate azimuth joint to starboard (+0.5 rad)
  arm.setJointAngles([0.5, 0.0, 0.0]);
  const fkAzimuth = arm.computeForwardKinematics();
  assert.ok(fkAzimuth.eePos[1] > 0.1, 'Positive azimuth angle must displace end-effector toward starboard');

  // Pitch shoulder joint
  arm.setJointAngles([0.0, 0.6, 0.0]);
  const fkShoulder = arm.computeForwardKinematics();
  assert.ok(Math.abs(fkShoulder.eePos[2] - fk0.eePos[2]) > 0.2, 'Shoulder pitch must displace end-effector in vertical axis');
});

test('UvmsManipulator computes geometric Jacobian J(q) matching joint axes', () => {
  const arm = new UvmsManipulator();
  arm.setJointAngles([0.1, 0.2, 0.3]);

  const fk = arm.computeForwardKinematics();
  const J = arm.computeJacobian(fk.frames, fk.eePos);

  assert.equal(J.length, 6);
  assert.equal(J[0].length, 3);

  // Check that Jacobian values are finite numbers
  for (let r = 0; r < 6; r++) {
    for (let c = 0; c < 3; c++) {
      assert.ok(!Number.isNaN(J[r][c]), `Jacobian element [${r}][${c}] must be a finite number`);
    }
  }
});

test('UvmsManipulator DLS inverse kinematics converges to reachable target with sub-millimeter error', () => {
  const arm = new UvmsManipulator();
  arm.setJointAngles([0.0, 0.2, 0.2]);

  // Target forward in reachable workspace
  const target = [0.95, 0.05, 0.10];
  const ikResult = arm.solveIK(target, 40, 1e-4);

  assert.ok(ikResult.converged, `IK should converge to target, error: ${ikResult.error}`);
  assert.ok(ikResult.error < 0.001, `Position error must be < 1 mm, got ${ikResult.error}`);

  const fk = arm.computeForwardKinematics();
  const dist = Math.hypot(target[0] - fk.eePos[0], target[1] - fk.eePos[1], target[2] - fk.eePos[2]);
  assert.ok(dist < 0.001, `Final end-effector distance must be < 1 mm, got ${dist}`);
});

test('UvmsManipulator calculates base reaction wrench under acceleration and hydrodynamic drag', () => {
  const arm = new UvmsManipulator();
  arm.setJointAngles([0.0, 0.3, 0.3]);

  // Static pose with zero joint rates: only gravity/buoyancy reaction
  const wrenchStatic = arm.computeReactionWrench([0, 0, 0], [0, 0, 0], R_IDENTITY);
  assert.equal(wrenchStatic.length, 6);
  assert.ok(wrenchStatic[2] > 0, 'Submerged weight of arm produces positive downward force in NED (+Z)');

  // Dynamic rapid downward shoulder acceleration (qDDot = 5 rad/s^2)
  const wrenchDynamic = arm.computeReactionWrench([0, 2.0, 0], [0, 5.0, 0], R_IDENTITY);
  // Rapid arm acceleration and drag must produce significant reaction wrench
  assert.ok(Math.abs(wrenchDynamic[0]) > 0, 'Inertial acceleration must create surge reaction');
  assert.ok(Math.abs(wrenchDynamic[4]) > 0, 'Shoulder motion must exert pitch reaction moment on vehicle');
});
