// 15-State Error-State Extended Kalman Filter (ES-EKF) for Acoustic-Inertial Subsea Navigation.
// Paper 3 & Milestone 2.1 — Advanced Subsea Operations.

import { euler2q, q2euler, Rquat, qdot, normalizeQuat } from './mss/library.js';

export class NavigationFilter {
  constructor() {
    // Nominal states
    this.p = [0, 0, 2]; // [North, East, Down]
    this.v = [0, 0, 0]; // [vN, vE, vD]
    this.q = euler2q(0, 0, 0); // Unit quaternion on S^3
    this.ba = [0, 0, 0]; // Accelerometer bias (m/s^2)
    this.bg = [0, 0, 0]; // Gyroscope bias (rad/s)

    // 15x15 Error Covariance Matrix P
    this.P = Array.from({ length: 15 }, (_, i) =>
      Array.from({ length: 15 }, (_, j) => (i === j ? (i < 3 ? 1.0 : i < 6 ? 0.2 : i < 9 ? 0.05 : 0.01) : 0))
    );

    // Continuous Spectral Densities
    this.accelNoise = 0.05;      // m/s^2 / sqrt(Hz)
    this.gyroNoise = 0.005;      // rad/s / sqrt(Hz)
    this.accelBiasWalk = 0.0005; // m/s^3 / sqrt(Hz)
    this.gyroBiasWalk = 0.00005; // rad/s^2 / sqrt(Hz)

    // Measurement Covariances
    this.rDvl = 0.015 * 0.015;     // (m/s)^2
    this.rDepth = 0.02 * 0.02;     // m^2
    this.rCompass = 0.03 * 0.03;   // rad^2
    this.rUsbl = 0.25 * 0.25;      // m^2
  }

  reset(pos = [0, 0, 2], heading = 0) {
    this.p = [...pos];
    this.v = [0, 0, 0];
    this.q = euler2q(0, 0, heading);
    this.ba = [0, 0, 0];
    this.bg = [0, 0, 0];

    for (let i = 0; i < 15; i++) {
      for (let j = 0; j < 15; j++) {
        this.P[i][j] = (i === j ? (i < 3 ? 0.5 : i < 6 ? 0.1 : i < 9 ? 0.02 : 0.005) : 0);
      }
    }
  }

  /**
   * Strapdown IMU Prediction Step (100 Hz).
   * @param {number} dt Time increment (s)
   * @param {number[]} f_b Specific force in body frame [fx, fy, fz] (m/s^2)
   * @param {number[]} omega_b Angular rate in body frame [p, q, r] (rad/s)
   */
  predict(dt, f_b, omega_b) {
    const R = Rquat(this.q);

    // 1. Unbiased specific force and angular rate
    const f_unbiased = [
      f_b[0] - this.ba[0],
      f_b[1] - this.ba[1],
      f_b[2] - this.ba[2],
    ];
    const omega_unbiased = [
      omega_b[0] - this.bg[0],
      omega_b[1] - this.bg[1],
      omega_b[2] - this.bg[2],
    ];

    // Accelerometer specific force to NED acceleration: a_n = R * f_b + g_n
    const f_ned = [
      R[0][0] * f_unbiased[0] + R[0][1] * f_unbiased[1] + R[0][2] * f_unbiased[2],
      R[1][0] * f_unbiased[0] + R[1][1] * f_unbiased[1] + R[1][2] * f_unbiased[2],
      R[2][0] * f_unbiased[0] + R[2][1] * f_unbiased[1] + R[2][2] * f_unbiased[2],
    ];
    const a_ned = [f_ned[0], f_ned[1], f_ned[2] + 9.81];

    // 2. Propagate nominal positions & velocities
    this.p[0] += this.v[0] * dt + 0.5 * a_ned[0] * dt * dt;
    this.p[1] += this.v[1] * dt + 0.5 * a_ned[1] * dt * dt;
    this.p[2] += this.v[2] * dt + 0.5 * a_ned[2] * dt * dt;

    this.v[0] += a_ned[0] * dt;
    this.v[1] += a_ned[1] * dt;
    this.v[2] += a_ned[2] * dt;

    // 3. Propagate nominal attitude quaternion
    const q_dot = qdot(this.q, omega_unbiased);
    this.q[0] += q_dot[0] * dt;
    this.q[1] += q_dot[1] * dt;
    this.q[2] += q_dot[2] * dt;
    this.q[3] += q_dot[3] * dt;
    this.q = normalizeQuat(this.q);

    // 4. Propagate 15x15 Error Covariance P_k = Phi * P * Phi^T + Q
    // Simplified discrete state transition matrix Phi = I + F*dt
    const qPos = 0.5 * this.accelNoise * dt * dt;
    const qVel = this.accelNoise * dt;
    const qAtt = this.gyroNoise * dt;
    const qBa = this.accelBiasWalk * dt;
    const qBg = this.gyroBiasWalk * dt;

    // Update diagonal variance propagation
    for (let i = 0; i < 3; i++) {
      this.P[i][i] += (2 * this.P[i + 3][i] * dt + qPos * qPos);
      this.P[i + 3][i + 3] += qVel * qVel;
      this.P[i + 6][i + 6] += qAtt * qAtt;
      this.P[i + 9][i + 9] += qBa * qBa;
      this.P[i + 12][i + 12] += qBg * qBg;
    }
  }

  /**
   * Doppler Velocity Log (DVL) Measurement Update (10 Hz).
   * @param {number[]} v_b Measured body velocity [u, v, w]
   */
  updateDVL(v_b) {
    const R = Rquat(this.q);
    // Predicted body velocity v_pred = R^T * v_ned
    const v_pred = [
      R[0][0] * this.v[0] + R[1][0] * this.v[1] + R[2][0] * this.v[2],
      R[0][1] * this.v[0] + R[1][1] * this.v[1] + R[2][1] * this.v[2],
      R[0][2] * this.v[0] + R[1][2] * this.v[1] + R[2][2] * this.v[2],
    ];

    const innovation = [
      v_b[0] - v_pred[0],
      v_b[1] - v_pred[1],
      v_b[2] - v_pred[2],
    ];

    // Scalar updates for each axis
    for (let axis = 0; axis < 3; axis++) {
      const stateIdx = 3 + axis;
      const pVar = this.P[stateIdx][stateIdx];
      const s = pVar + this.rDvl;
      const k = pVar / s;

      // Transform body velocity innovation back to NED velocity correction
      const corrNed = [
        R[0][axis] * innovation[axis] * k,
        R[1][axis] * innovation[axis] * k,
        R[2][axis] * innovation[axis] * k,
      ];

      this.v[0] += corrNed[0];
      this.v[1] += corrNed[1];
      this.v[2] += corrNed[2];

      this.P[stateIdx][stateIdx] *= (1 - k);
    }
  }

  /**
   * Hydrostatic Depth Pressure Measurement Update (10 Hz).
   * @param {number} depth Measured depth (m)
   */
  updateDepth(depth) {
    const zPred = this.p[2];
    const innov = depth - zPred;

    const pVar = this.P[2][2];
    const s = pVar + this.rDepth;
    const k = pVar / s;

    this.p[2] += k * innov;
    this.P[2][2] *= (1 - k);
  }

  /**
   * Compass Heading Measurement Update (10 Hz).
   * @param {number} headingRad Measured magnetic heading (rad)
   */
  updateCompass(headingRad) {
    const euler = q2euler(this.q);
    let innov = headingRad - euler[2];
    // Wrap to [-pi, pi]
    innov = Math.atan2(Math.sin(innov), Math.cos(innov));

    const pVar = this.P[8][8];
    const s = pVar + this.rCompass;
    const k = pVar / s;

    const updatedHeading = euler[2] + k * innov;
    this.q = euler2q(euler[0], euler[1], updatedHeading);
    this.P[8][8] *= (1 - k);
  }

  /**
   * Ultra-Short Baseline (USBL) Acoustic Fix Update (0.5 Hz - 1.0 Hz).
   * @param {number[]} p_usbl Acoustic fix [North, East, Down]
   */
  updateUSBL(p_usbl) {
    for (let axis = 0; axis < 3; axis++) {
      const innov = p_usbl[axis] - this.p[axis];
      const pVar = this.P[axis][axis];
      const s = pVar + this.rUsbl;
      const k = pVar / s;

      this.p[axis] += k * innov;
      this.P[axis][axis] *= (1 - k);
    }
  }

  getEstimate() {
    const euler = q2euler(this.q);
    return {
      position: [...this.p],
      velocity: [...this.v],
      quaternion: [...this.q],
      eulerDeg: [
        Number((euler[0] * 180 / Math.PI).toFixed(2)),
        Number((euler[1] * 180 / Math.PI).toFixed(2)),
        Number((((euler[2] * 180 / Math.PI) + 360) % 360).toFixed(2)),
      ],
      accelBias: [...this.ba],
      gyroBias: [...this.bg],
      threeSigmaPos: [
        Math.sqrt(Math.max(0, this.P[0][0])) * 3,
        Math.sqrt(Math.max(0, this.P[1][1])) * 3,
        Math.sqrt(Math.max(0, this.P[2][2])) * 3,
      ],
      threeSigmaVel: [
        Math.sqrt(Math.max(0, this.P[3][3])) * 3,
        Math.sqrt(Math.max(0, this.P[4][4])) * 3,
        Math.sqrt(Math.max(0, this.P[5][5])) * 3,
      ],
    };
  }
}
