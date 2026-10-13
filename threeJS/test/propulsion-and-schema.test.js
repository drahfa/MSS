import { test } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { effectiveThrustRatio, thrusterWrench, ROV_PROFILES } from '../src/physics.js';
import { buildVehicle, rovPlant } from '../src/mss/rovPlant.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const vehiclesDir = path.resolve(__dirname, '../public/vehicles');

test('effectiveThrustRatio models deadband and reverse asymmetry accurately', () => {
  const deadband = 0.05;
  const reverseEfficiency = 0.70;

  // 1. Inside deadband -> exactly zero thrust
  assert.equal(effectiveThrustRatio(0, reverseEfficiency, deadband), 0);
  assert.equal(effectiveThrustRatio(0.02, reverseEfficiency, deadband), 0);
  assert.equal(effectiveThrustRatio(-0.049, reverseEfficiency, deadband), 0);

  // 2. Full forward command -> 1.0
  assert.ok(Math.abs(effectiveThrustRatio(1.0, reverseEfficiency, deadband) - 1.0) < 1e-9);

  // 3. Full reverse command -> -reverseEfficiency (-0.70)
  assert.ok(Math.abs(effectiveThrustRatio(-1.0, reverseEfficiency, deadband) - (-0.70)) < 1e-9);

  // 4. Mid-range linearity above deadband
  // u = 0.525 -> active = (0.525 - 0.05) / 0.95 = 0.50
  assert.ok(Math.abs(effectiveThrustRatio(0.525, reverseEfficiency, deadband) - 0.50) < 1e-9);
  assert.ok(Math.abs(effectiveThrustRatio(-0.525, reverseEfficiency, deadband) - (-0.35)) < 1e-9);
});

test('Vehicle JSON profiles load and match runtime vehicle structures', () => {
  for (const key of Object.keys(ROV_PROFILES)) {
    const jsonPath = path.join(vehiclesDir, `${key}.json`);
    assert.ok(fs.existsSync(jsonPath), `Missing JSON profile: ${jsonPath}`);

    const raw = fs.readFileSync(jsonPath, 'utf8');
    const profile = JSON.parse(raw);

    assert.equal(profile.name, ROV_PROFILES[key].name);
    assert.equal(profile.mass, ROV_PROFILES[key].mass);
    assert.equal(profile.thrusters.length, ROV_PROFILES[key].thrusters.length);

    // Build vehicle from external JSON profile
    const vehicle = buildVehicle(profile);
    assert.equal(vehicle.M.length, 6);
    assert.equal(vehicle.DL.length, 6);
    assert.ok(vehicle.W > 0);
  }
});
