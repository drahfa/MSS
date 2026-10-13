import { test } from 'node:test';
import assert from 'node:assert/strict';
import { validateProfile, formatProfileJson } from '../src/mss/profileValidator.js';
import { ROV_PROFILES, RovDynamics } from '../src/physics.js';
import { buildVehicle } from '../src/mss/rovPlant.js';

test('Validator accepts all 5 built-in ROV profiles and produces valid diagnostics', () => {
  for (const [key, profile] of Object.entries(ROV_PROFILES)) {
    const res = validateProfile(profile);
    assert.ok(res.valid, `${key}: profile failed schema validation: ${res.errors.join('; ')}`);
    assert.equal(res.errors.length, 0);
    assert.ok(res.summary.mass > 0);
    assert.ok(res.summary.thrusterCount >= 3);
  }
});

test('Validator catches invalid profile structures and corrupted parameters', () => {
  // Test missing required field
  const missingMass = { name: 'Faulty', role: 'Test' };
  assert.equal(validateProfile(missingMass).valid, false);

  // Test negative mass
  const negativeMass = { ...ROV_PROFILES.inspection, mass: -10 };
  assert.equal(validateProfile(negativeMass).valid, false);

  // Test corrupted inertia length
  const badInertia = { ...ROV_PROFILES.inspection, inertia: [1, 2] };
  assert.equal(validateProfile(badInertia).valid, false);

  // Test invalid thruster force
  const badThruster = {
    ...ROV_PROFILES.inspection,
    thrusters: [{ id: 'T1', label: 'Bad', r: [0, 0, 0], f: [1, 0, 0], force: -50 }],
  };
  assert.equal(validateProfile(badThruster).valid, false);

  // Test zero direction vector
  const zeroVector = {
    ...ROV_PROFILES.inspection,
    thrusters: [{ id: 'T1', label: 'Zero', r: [0, 0, 0], f: [0, 0, 0], force: 50 }],
  };
  assert.equal(validateProfile(zeroVector).valid, false);
});

test('formatProfileJson serializes cleanly and parses back to identical profile structure', () => {
  const profile = ROV_PROFILES.inspection;
  const jsonStr = formatProfileJson(profile);
  const parsed = JSON.parse(jsonStr);

  const res = validateProfile(parsed);
  assert.ok(res.valid, `Parsed JSON failed validation: ${res.errors.join(', ')}`);
  assert.equal(parsed.name, profile.name);
  assert.equal(parsed.mass, profile.mass);
  assert.equal(parsed.thrusters.length, profile.thrusters.length);
});

test('RovDynamics dynamically accepts an imported custom profile without source edits', () => {
  const customProfile = {
    name: 'Experimental Deep Sea',
    role: 'Ultra-deep science · 4,000 m',
    mass: 120.0,
    inertia: [12.0, 18.0, 20.0],
    addedMass: [50.0, 75.0, 90.0, 3.0, 7.5, 8.0],
    linearDamping: [50.0, 70.0, 85.0, 10.0, 16.0, 18.0],
    quadraticDamping: [80.0, 110.0, 130.0, 8.0, 12.0, 14.0],
    leverScale: 1.5,
    cg: 0.06,
    cameraDistance: 7.0,
    actuatorTau: 0.20,
    buoyancyRatio: 1.008,
    thrusters: [
      { id: 'H1', label: 'Port', r: [0.4, -0.5, 0], f: [1, 0, 0], force: 100, reverseEfficiency: 0.8, deadband: 0.03 },
      { id: 'H2', label: 'Starboard', r: [0.4, 0.5, 0], f: [1, 0, 0], force: 100, reverseEfficiency: 0.8, deadband: 0.03 },
      { id: 'V1', label: 'Vertical', r: [0, 0, -0.2], f: [0, 0, 1], force: 120, reverseEfficiency: 0.75, deadband: 0.04 },
    ]
  };

  const validation = validateProfile(customProfile);
  assert.ok(validation.valid);

  const sim = new RovDynamics('inspection');
  assert.equal(sim.profile.mass, 85.0);

  // Dynamically configure simulator with custom profile
  sim.configure('custom', customProfile);
  assert.equal(sim.profile.mass, 120.0);
  assert.equal(sim.profile.name, 'Experimental Deep Sea');
  assert.equal(sim.vehicle.m, 120.0);

  // Verify step runs stably with the newly imported vehicle
  sim.step(0.01);
  assert.ok(!Number.isNaN(sim.state[0]));
});
