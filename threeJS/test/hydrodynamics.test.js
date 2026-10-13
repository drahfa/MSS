import { test } from 'node:test';
import assert from 'node:assert/strict';
import { buildVehicle, rovPlant } from '../src/mss/rovPlant.js';
import { rk4, euler2q, q2euler, matVec, mldivide, normalizeQuat } from '../src/mss/library.js';
import { ROV_PROFILES } from '../src/physics.js';

test('Pitch free-decay oscillation matches analytical damped frequency and decays monotonically', () => {
  for (const [key, profile] of Object.entries(ROV_PROFILES)) {
    const v = buildVehicle(profile);
    const M55 = v.M[4][4];
    const restoringStiffness = v.r_bG[2] * v.W;
    const wn = Math.sqrt(restoringStiffness / M55);
    const zeta = v.DL[4][4] / (2 * Math.sqrt(M55 * restoringStiffness));
    const wd = wn * Math.sqrt(Math.max(0, 1 - zeta * zeta));
    const theoreticalTd = (2 * Math.PI) / wd;

    // Perturb pitch by 0.10 rad (5.73 deg)
    const q0 = euler2q(0, 0.10, 0);
    let x = [0, 0, 0, 0, 0, 0, 0, 0, 5, ...q0];
    const tau = [0, 0, 0, 0, 0, 0];
    const dt = 0.01;
    const peaks = [];
    let prevPitchRate = 0;

    // Simulate for 4 cycles or 25 seconds
    const simTime = Math.min(25.0, theoreticalTd * 4.5);
    for (let t = 0; t <= simTime; t += dt) {
      x = rk4(rovPlant, dt, x, tau, v, 0, 0);
      const qNorm = normalizeQuat(x.slice(9, 13));
      x[9] = qNorm[0]; x[10] = qNorm[1]; x[11] = qNorm[2]; x[12] = qNorm[3];
      const [, theta] = q2euler(qNorm);
      const pitchRate = x[4];

      if (t > 0.1 && prevPitchRate > 0 && pitchRate <= 0 && theta > 0) {
        peaks.push({ t, theta });
      }
      prevPitchRate = pitchRate;
    }

    // Must record at least 2 oscillation peaks
    assert.ok(peaks.length >= 2, `${key}: Expected at least 2 positive pitch peaks, got ${peaks.length}`);

    // Verify peak period matches theoretical damped period within 3% tolerance
    const measuredPeriod = peaks[1].t - peaks[0].t;
    const periodError = Math.abs(measuredPeriod - theoreticalTd) / theoreticalTd;
    assert.ok(
      periodError < 0.03,
      `${key}: Measured period (${measuredPeriod.toFixed(3)}s) diverges from theoretical (${theoreticalTd.toFixed(3)}s) by ${(periodError * 100).toFixed(2)}%`
    );

    // Verify amplitude decays monotonically across successive peaks
    for (let i = 0; i < peaks.length - 1; i++) {
      assert.ok(
        peaks[i + 1].theta < peaks[i].theta,
        `${key}: Peak ${i + 1} (${peaks[i + 1].theta.toFixed(4)}) should be smaller than peak ${i} (${peaks[i].theta.toFixed(4)})`
      );
    }
  }
});

test('Surge step-response acceleration and terminal velocity converge to analytical solutions', () => {
  for (const [key, profile] of Object.entries(ROV_PROFILES)) {
    const v = buildVehicle(profile);
    const DL1 = v.DL[0][0];
    const Dn1 = v.Dn[0];

    // Commanded constant surge force (15% of vehicle weight in Newtons)
    const tauSurge = v.W * 0.15;
    const tau = [tauSurge, 0, 0, 0, 0, 0];

    // Analytical initial acceleration considering coupled 6x6 mass solve M \ tau
    const coupledAcc0 = mldivide(v.M, tau)[0];
    const theoreticalUInf = (-DL1 + Math.sqrt(DL1 * DL1 + 4 * Dn1 * tauSurge)) / (2 * Dn1);

    let x = [0, 0, 0, 0, 0, 0, 0, 0, 5, 1, 0, 0, 0];
    const dt = 0.01;

    // Check initial acceleration at t = 0 matches coupled solve to machine precision
    const xdot0 = rovPlant(x, tau, v, 0, 0);
    const acc0Error = Math.abs(xdot0[0] - coupledAcc0) / coupledAcc0;
    assert.ok(
      acc0Error < 1e-6,
      `${key}: Initial acceleration (${xdot0[0].toFixed(6)}) diverges from coupled theoretical (${coupledAcc0.toFixed(6)}) by ${(acc0Error * 100).toFixed(4)}%`
    );

    // Integrate until steady state (35 seconds)
    for (let t = 0; t <= 35.0; t += dt) {
      x = rk4(rovPlant, dt, x, tau, v, 0, 0);
    }

    const uFinal = x[0];
    const uInfError = Math.abs(uFinal - theoreticalUInf) / theoreticalUInf;
    assert.ok(
      uInfError < 0.02,
      `${key}: Final surge velocity (${uFinal.toFixed(4)} m/s) diverges from theoretical (${theoreticalUInf.toFixed(4)} m/s) by ${(uInfError * 100).toFixed(2)}%`
    );
  }
});

test('Unforced hydrodynamic system strictly dissipates mechanical energy (Lyapunov passivity)', () => {
  for (const [key, profile] of Object.entries(ROV_PROFILES)) {
    // Test with neutrally buoyant profile to isolate restoring potential and kinetic dissipation
    const v = buildVehicle({ ...profile, buoyancyRatio: 1.0 });

    // Initial multi-axis disturbance in velocity and attitude
    const q0 = euler2q(0.08, -0.06, 0.04);
    let x = [0.25, -0.15, 0.10, 0.05, -0.04, 0.03, 0, 0, 5, ...q0];
    const tau = [0, 0, 0, 0, 0, 0];
    const dt = 0.01;

    // Compute initial total mechanical energy
    const nu0 = x.slice(0, 6);
    const Mnu0 = matVec(v.M, nu0);
    const E_kin0 = 0.5 * nu0.reduce((sum, val, i) => sum + val * Mnu0[i], 0);
    const [phi0, theta0] = q2euler(q0);
    const E_pot0 = (v.r_bG[2] * v.W) * (1 - Math.cos(theta0) * Math.cos(phi0));
    const initialEnergy = E_kin0 + E_pot0;

    let prevEnergy = initialEnergy;
    let nonDecreasingSteps = 0;

    for (let t = 0; t <= 12.0; t += dt) {
      const nu = x.slice(0, 6);
      const q = normalizeQuat(x.slice(9, 13));
      const [phi, theta] = q2euler(q);

      const Mnu = matVec(v.M, nu);
      const E_kin = 0.5 * nu.reduce((sum, val, i) => sum + val * Mnu[i], 0);
      const E_pot = (v.r_bG[2] * v.W) * (1 - Math.cos(theta) * Math.cos(phi));
      const E_total = E_kin + E_pot;

      // Numerical tolerance (1e-6 J) for RK4 discretization
      if (E_total > prevEnergy + 1e-6) {
        nonDecreasingSteps++;
      }
      prevEnergy = E_total;

      x = rk4(rovPlant, dt, x, tau, v, 0, 0);
      const qStep = normalizeQuat(x.slice(9, 13));
      x[9] = qStep[0]; x[10] = qStep[1]; x[11] = qStep[2]; x[12] = qStep[3];
    }

    assert.equal(
      nonDecreasingSteps,
      0,
      `${key}: Energy increased at ${nonDecreasingSteps} time steps during unforced motion`
    );
    // At least 98% of total mechanical energy must be dissipated over 12 seconds
    const dissipatedRatio = (initialEnergy - prevEnergy) / initialEnergy;
    assert.ok(
      dissipatedRatio > 0.98,
      `${key}: Energy dissipation insufficient: ${(dissipatedRatio * 100).toFixed(1)}% dissipated (remaining ${prevEnergy.toFixed(4)} J of ${initialEnergy.toFixed(4)} J)`
    );
  }
});
