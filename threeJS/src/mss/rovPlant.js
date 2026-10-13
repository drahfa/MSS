// MSS-composed 6-DOF ROV plant.
//
// This module is the JavaScript twin of validation/mss/rovPlant.m. The .m
// reference is written exclusively with MSS LIBRARY calls and follows the
// structure of CRAFT/AUV/models/remus100.m and CRAFT/hydroVessel.m:
//
//   eta_dot = J(eta) * nu
//   nu_dot  = nu_c_dot + M \ (tau - C(nu_r) * nu_r - D(nu_r) * nu_r - g(eta))
//
//   nu_r      = nu - nu_c                          relative velocity
//   nu_c_dot  = [-Smtrx(nu(4:6)) * v_c; 0; 0; 0]   irrotational current (hydroVessel.m)
//   M         = MRB + MA                           MRB from rbody.m about the CO
//   C         = CRB + CA                           rbody.m, m2c.m
//   D(nu_r)   = D_L + diag(D_n .* abs(nu_r))       vehicle-defined damping data
//   g(eta)    = gRvect(W, B, R, r_bG, r_bB)        gRvect.m
//   J, R      = eulerang(phi, theta, psi)          eulerang.m
//
// The plant contains no browser clamps, no actuator model, and no control
// logic. Thruster allocation, actuator lag, assist controllers, angle
// wrapping, and scene envelope constraints live outside it in physics.js and
// reach the plant only through the generalized force tau.

import { diag, eulerang, gRvect, m2c, matAdd, matVec, mldivide, mssConstants, rbody, Smtrx, vecSub } from './library.js';

/**
 * Convert a simulator profile into an MSS-style vehicle structure. All
 * quantities are SI and expressed about the coordinate origin (CO).
 */
export function buildVehicle(profile) {
  const { g } = mssConstants();
  const m = profile.mass;
  const [Ix, Iy, Iz] = profile.inertia;
  const R44 = Math.sqrt(Ix / m), R55 = Math.sqrt(Iy / m), R66 = Math.sqrt(Iz / m);
  const r_bG = [0, 0, profile.cg];
  const r_bB = [0, 0, 0];
  const MA = profile.addedMassMatrix ?? diag(profile.addedMass);
  const { MRB } = rbody(m, R44, R55, R66, [0, 0, 0], r_bG);
  const W = m * g;
  return {
    name: profile.name,
    m, R44, R55, R66, r_bG, r_bB,
    MA,
    MRB,
    M: matAdd(MRB, MA),
    DL: profile.linearDampingMatrix ?? diag(profile.linearDamping),
    Dn: [...profile.quadraticDamping],
    W,
    B: W * (profile.buoyancyRatio ?? 1.006),
  };
}

/**
 * xdot = rovPlant(x, tau, vehicle, Vc, betaVc)
 *
 * @param {number[]} x       12-state [u v w p q r north east down phi theta psi]
 * @param {number[]} tau     6x1 generalized force [X Y Z K M N] about the CO
 * @param {object}   vehicle Structure from buildVehicle()
 * @param {number}   Vc      Horizontal current speed (m/s)
 * @param {number}   betaVc  Current direction in the North-East frame (rad)
 */
export function rovPlant(x, tau, vehicle, Vc = 0, betaVc = 0) {
  const nu = x.slice(0, 6);
  const eta = x.slice(6, 12);
  const omega = nu.slice(3, 6);

  // Irrotational North-East current expressed in BODY (hydroVessel.m)
  const v_c = [Vc * Math.cos(betaVc - eta[5]), Vc * Math.sin(betaVc - eta[5]), 0];
  const nu_c = [...v_c, 0, 0, 0];
  const Sv = matVec(Smtrx(omega), v_c);
  const Dnu_c = [-Sv[0], -Sv[1], -Sv[2], 0, 0, 0];
  const nu_r = vecSub(nu, nu_c);

  // Rigid-body and hydrodynamic Coriolis-centripetal matrices
  const { CRB } = rbody(vehicle.m, vehicle.R44, vehicle.R55, vehicle.R66, omega, vehicle.r_bG);
  const CA = m2c(vehicle.MA, nu_r);
  const C = matAdd(CRB, CA);

  // Linear plus quadratic damping evaluated at the relative velocity
  const D = vehicle.DL.map((row, i) => row.map((d, j) => (i === j ? d + vehicle.Dn[i] * Math.abs(nu_r[i]) : d)));

  // Kinematics and restoring forces
  const { J, J1: R } = eulerang(eta[3], eta[4], eta[5]);
  const g = gRvect(vehicle.W, vehicle.B, R, vehicle.r_bG, vehicle.r_bB);

  const Cnu = matVec(C, nu_r);
  const Dnu = matVec(D, nu_r);
  const rhs = tau.map((t, i) => t - Cnu[i] - Dnu[i] - g[i]);
  const nuDot = mldivide(vehicle.M, rhs).map((v, i) => Dnu_c[i] + v);
  const etaDot = matVec(J, nu);
  return [...nuDot, ...etaDot];
}
