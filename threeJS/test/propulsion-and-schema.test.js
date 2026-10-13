import { test } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { effectiveThrustRatio, advanceRatioFactor, currentShear, thrusterWrench, ROV_PROFILES } from '../src/physics.js';
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

test('advanceRatioFactor models through-water inflow degradation accurately', () => {
  const kAdv = 0.30;
  const vMax = 2.0;

  // 1. Zero inflow or zero command -> 1.0 (bollard pull)
  assert.equal(advanceRatioFactor(0, 1.0, kAdv, vMax), 1.0);
  assert.equal(advanceRatioFactor(1.0, 0, kAdv, vMax), 1.0);

  // 2. Forward thrust with forward inflow (Va = 1.0 m/s):
  // factor = 1 - 0.30 * (1.0 / 2.0) = 0.85 (15% drop)
  const factorHalf = advanceRatioFactor(1.0, 1.0, kAdv, vMax);
  assert.ok(Math.abs(factorHalf - 0.85) < 1e-9);

  // 3. Full advance velocity (Va = 2.0 m/s):
  // factor = 1 - 0.30 * (2.0 / 2.0) = 0.70 (30% drop)
  const factorFull = advanceRatioFactor(2.0, 1.0, kAdv, vMax);
  assert.ok(Math.abs(factorFull - 0.70) < 1e-9);

  // 4. Reverse thrust with reverse inflow (-1.0 m/s with u = -1):
  // alignedInflow = (-1) * (-1.0) = 1.0 -> same 15% drop
  const factorRev = advanceRatioFactor(-1.0, -1.0, kAdv, vMax);
  assert.ok(Math.abs(factorRev - 0.85) < 1e-9);

  // 5. Opposing flow (pushing forward while being towed backwards Va = -2.0 m/s):
  // alignedInflow = 1.0 * (-2.0) = -2.0 -> factor = 1 - 0.30 * (-1.0) = 1.30 -> clamped to 1.15
  const factorOppose = advanceRatioFactor(-2.0, 1.0, kAdv, vMax);
  assert.equal(factorOppose, 1.15);
});

test('currentShear calculates depth-dependent velocity gradient', () => {
  const surfaceSpeed = 0.50;
  const seabedDepth = 20.0;
  const bottomFactor = 0.40;

  // At surface (depth = 0) -> 100% surface speed
  assert.equal(currentShear(0, surfaceSpeed, seabedDepth, bottomFactor), 0.50);

  // At seabed (depth = 20) -> bottomFactor * surfaceSpeed = 0.40 * 0.50 = 0.20
  assert.equal(currentShear(20, surfaceSpeed, seabedDepth, bottomFactor), 0.20);

  // At midwater (depth = 10) -> (1.0 - 0.6 * 0.5) * 0.50 = 0.70 * 0.50 = 0.35
  assert.ok(Math.abs(currentShear(10, surfaceSpeed, seabedDepth, bottomFactor) - 0.35) < 1e-9);

  // Clamped beyond seabed
  assert.equal(currentShear(25, surfaceSpeed, seabedDepth, bottomFactor), 0.20);
});

test('thrusterWrench computes static bollard pull without relative velocity and degrades with inflow', () => {
  const profile = ROV_PROFILES.inspection;
  const commands = [1, 1, 1, 1, 0, 0, 0, 0]; // 4 horizontal thrusters full forward

  // Static bollard pull (no nu_r)
  const tauStatic = thrusterWrench(commands, profile);

  // High forward relative surge velocity (u_rel = 1.5 m/s)
  const nu_r_moving = [1.5, 0, 0, 0, 0, 0];
  const tauMoving = thrusterWrench(commands, profile, nu_r_moving);

  // Surge force should be positive in both cases, but static > moving
  assert.ok(tauStatic[0] > 0);
  assert.ok(tauMoving[0] > 0);
  assert.ok(tauStatic[0] > tauMoving[0], `Static surge force (${tauStatic[0]}) should exceed forward moving surge force (${tauMoving[0]})`);
});

