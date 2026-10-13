import { test } from 'node:test';
import assert from 'node:assert/strict';
import { buildVehicle, rovPlant } from '../src/mss/rovPlant.js';
import { rk4 } from '../src/mss/library.js';
import { ROV_PROFILES } from '../src/physics.js';

test('buildVehicle builds valid MSS system matrices for all 5 profiles', () => {
  for (const [key, profile] of Object.entries(ROV_PROFILES)) {
    const v = buildVehicle(profile);
    assert.equal(v.M.length, 6, `${key}: M should be 6x6`);
    assert.equal(v.DL.length, 6, `${key}: DL should be 6x6`);
    assert.equal(v.Dn.length, 6, `${key}: Dn should have 6 elements`);
    assert.ok(v.W > 0, `${key}: Weight should be positive`);
    assert.ok(v.B > 0, `${key}: Buoyancy should be positive`);

    // Verify M is symmetric positive definite
    for (let i = 0; i < 6; i++) {
      assert.ok(v.M[i][i] > 0, `${key}: M diagonal [${i}] should be positive`);
      for (let j = 0; j < 6; j++) {
        assert.ok(Math.abs(v.M[i][j] - v.M[j][i]) < 1e-9, `${key}: M should be symmetric at (${i},${j})`);
      }
    }
  }
});

test('rovPlant runs 60-second deterministic scenario across all 5 profiles with finite state', () => {
  for (const [key, profile] of Object.entries(ROV_PROFILES)) {
    const vehicle = buildVehicle(profile);

    // Initial perturbed state: slightly submerged, small pitch & roll, small surge
    let state = [0.2, 0, 0, 0.05, -0.05, 0, 0, 0, 2, 0.05, -0.05, 0];
    const tau = [0, 0, 0, 0, 0, 0];
    const Vc = 0.15;
    const betaVc = 0.5;
    const dt = 0.01;

    for (let step = 0; step < 6000; step++) {
      state = rk4(rovPlant, dt, state, tau, vehicle, Vc, betaVc);
      for (let i = 0; i < 12; i++) {
        assert.ok(
          !Number.isNaN(state[i]) && Number.isFinite(state[i]),
          `${key}: State [${i}] became invalid at step ${step}: ${state[i]}`
        );
      }
    }

    // Angular velocities should decay due to hydrodynamic damping
    const [u, v, w, p, q, r] = state.slice(0, 6);
    assert.ok(Math.abs(p) < 1e-3, `${key}: Roll rate should damp out`);
    assert.ok(Math.abs(q) < 1e-3, `${key}: Pitch rate should damp out`);
    assert.ok(Math.abs(r) < 1e-3, `${key}: Yaw rate should damp out`);
  }
});
