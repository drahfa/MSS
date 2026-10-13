// Multi-ROV Cooperative Operations & Networked Digital Twin Telemetry Bridge.
// Paper 5 & Milestone 2.3 — Advanced Cooperative Intervention.

import { q2euler } from './mss/library.js';

/**
 * Computes the minimum Euclidean distance between two 3D line segments P1-P2 and Q1-Q2.
 * Based on exact geometric projection with boundary clamping to [0, 1].
 */
export function segmentSegmentDistance3D(P1, P2, Q1, Q2) {
  const u = [P2[0] - P1[0], P2[1] - P1[1], P2[2] - P1[2]];
  const v = [Q2[0] - Q1[0], Q2[1] - Q1[1], Q2[2] - Q1[2]];
  const w = [P1[0] - Q1[0], P1[1] - Q1[1], P1[2] - Q1[2]];

  const a = u[0] * u[0] + u[1] * u[1] + u[2] * u[2]; // |u|^2
  const b = u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; // u . v
  const c = v[0] * v[0] + v[1] * v[1] + v[2] * v[2]; // |v|^2
  const d = u[0] * w[0] + u[1] * w[1] + u[2] * w[2]; // u . w
  const e = v[0] * w[0] + v[1] * w[1] + v[2] * w[2]; // v . w

  const D = a * c - b * b; // Cross product magnitude squared
  let sN, sD = D; // Default s = sN / sD
  let tN, tD = D; // Default t = tN / tD

  // Check if segments are almost parallel
  if (D < 1e-8) {
    sN = 0.0;
    sD = 1.0;
    tN = e;
    tD = c;
  } else {
    sN = (b * e - c * d);
    tN = (a * e - b * d);

    if (sN < 0.0) {
      sN = 0.0;
      tN = e;
      tD = c;
    } else if (sN > sD) {
      sN = sD;
      tN = e + b;
      tD = c;
    }
  }

  if (tN < 0.0) {
    tN = 0.0;
    if (-d < 0.0) sN = 0.0;
    else if (-d > a) sN = sD;
    else {
      sN = -d;
      sD = a;
    }
  } else if (tN > tD) {
    tN = tD;
    if ((-d + b) < 0.0) sN = 0.0;
    else if ((-d + b) > a) sN = sD;
    else {
      sN = (-d + b);
      sD = a;
    }
  }

  const sc = Math.abs(sN) < 1e-8 ? 0.0 : sN / sD;
  const tc = Math.abs(tN) < 1e-8 ? 0.0 : tN / tD;

  // Closest points: P1 + sc * u and Q1 + tc * v
  const dPx = w[0] + (sc * u[0]) - (tc * v[0]);
  const dPy = w[1] + (sc * u[1]) - (tc * v[1]);
  const dPz = w[2] + (sc * u[2]) - (tc * v[2]);

  return Math.hypot(dPx, dPy, dPz);
}

export class MultiRovTetherBridge {
  constructor(config = {}) {
    this.warnDistance = config.warnDistance ?? 3.0; // Warning threshold in meters
    this.critDistance = config.critDistance ?? 1.0; // Critical entanglement threshold in meters
  }

  /**
   * Computes spatial clearance and minimum distance between two multi-node umbilical cables.
   * @param {number[][]} nodesA Array of [x, y, z] points for cable A
   * @param {number[][]} nodesB Array of [x, y, z] points for cable B
   * @returns {object} Clearance report
   */
  computeTetherClearance(nodesA, nodesB) {
    if (!nodesA || !nodesB || nodesA.length < 2 || nodesB.length < 2) {
      return { minDistance: Infinity, advisory: false, criticalAlert: false };
    }

    let minDistance = Infinity;
    let closestSegA = 0;
    let closestSegB = 0;

    for (let i = 0; i < nodesA.length - 1; i++) {
      for (let j = 0; j < nodesB.length - 1; j++) {
        const dist = segmentSegmentDistance3D(nodesA[i], nodesA[i + 1], nodesB[j], nodesB[j + 1]);
        if (dist < minDistance) {
          minDistance = dist;
          closestSegA = i;
          closestSegB = j;
        }
      }
    }

    return {
      minDistance: Number(minDistance.toFixed(3)),
      closestSegA,
      closestSegB,
      advisory: minDistance <= this.warnDistance,
      criticalAlert: minDistance <= this.critDistance,
    };
  }

  /**
   * Evaluates closed-chain kinematics and restoring forces for a shared rigid subsea payload.
   * @param {number[]} posA Position of Lead ROV [x, y, z] in NED
   * @param {number[]} posB Position of Follower ROV [x, y, z] in NED
   * @param {number} payloadLength Nominal rigid payload length (m)
   * @param {number} payloadMass Mass of payload in air (kg)
   * @param {number} stiffness Structural coupling stiffness (N/m)
   */
  evaluateCooperativePayload(posA, posB, payloadLength = 6.0, payloadMass = 120.0, stiffness = 15000) {
    const rx = posB[0] - posA[0];
    const ry = posB[1] - posA[1];
    const rz = posB[2] - posA[2];
    const actualDist = Math.hypot(rx, ry, rz);

    const error = actualDist - payloadLength;
    const tensionMag = stiffness * error; // Positive = tension, negative = compression

    const nx = rx / (actualDist || 1);
    const ny = ry / (actualDist || 1);
    const nz = rz / (actualDist || 1);

    // 50% submerged weight share (assuming steel density 7850 kg/m^3)
    const submergedWeight = payloadMass * 9.81 * (1 - 1025.0 / 7850.0);
    const loadShareZ = submergedWeight * 0.5;

    return {
      distance: Number(actualDist.toFixed(3)),
      error: Number(error.toFixed(4)),
      tension: Number(tensionMag.toFixed(1)),
      forceOnA: [tensionMag * nx, tensionMag * ny, tensionMag * nz + loadShareZ],
      forceOnB: [-tensionMag * nx, -tensionMag * ny, -tensionMag * nz + loadShareZ],
      submergedWeight: Number(submergedWeight.toFixed(1)),
    };
  }

  /**
   * Serializes vehicle physical state and sensors into standard ROS 2 JSON payloads.
   * @param {string} msgType 'Odometry' | 'Imu' | 'Twist'
   * @param {number[]} state 13-state vector
   * @param {object} sensors Sensor readings
   * @param {number} timestampSec Time in seconds
   */
  serializeRos2Message(msgType, state, sensors = null, timestampSec = 0) {
    const sec = Math.floor(timestampSec);
    const nanosec = Math.floor((timestampSec - sec) * 1e9);

    if (msgType === 'Odometry') {
      return {
        header: { stamp: { sec, nanosec }, frame_id: 'world_ned' },
        child_frame_id: 'base_link',
        pose: {
          pose: {
            position: { x: state[6], y: state[7], z: state[8] },
            orientation: { w: state[9], x: state[10], y: state[11], z: state[12] },
          },
        },
        twist: {
          twist: {
            linear: { x: state[0], y: state[1], z: state[2] },
            angular: { x: state[3], y: state[4], z: state[5] },
          },
        },
      };
    }

    if (msgType === 'Imu') {
      const imuData = sensors?.imu ?? { rates: [state[3], state[4], state[5]], accel: [0, 0, 9.81] };
      return {
        header: { stamp: { sec, nanosec }, frame_id: 'base_link' },
        orientation: { w: state[9], x: state[10], y: state[11], z: state[12] },
        angular_velocity: { x: imuData.rates[0], y: imuData.rates[1], z: imuData.rates[2] },
        linear_acceleration: { x: imuData.accel[0], y: imuData.accel[1], z: imuData.accel[2] },
      };
    }

    if (msgType === 'Twist') {
      return {
        linear: { x: state[0], y: state[1], z: state[2] },
        angular: { x: state[3], y: state[4], z: state[5] },
      };
    }

    throw new Error(`Unknown ROS 2 message type: ${msgType}`);
  }

  /**
   * Serializes vehicle physical state and telemetry into standard MAVLink 2.0 JSON payloads.
   * @param {string} msgType 'HEARTBEAT' | 'ATTITUDE' | 'GLOBAL_POSITION_INT' | 'SERVO_OUTPUT_RAW'
   * @param {number[]} state 13-state vector
   * @param {number[]} thrusters Thruster allocation outputs [-1, 1]
   * @param {number} timeBootMs Time since boot in milliseconds
   */
  serializeMavlinkMessage(msgType, state, thrusters = [], timeBootMs = 0) {
    if (msgType === 'HEARTBEAT') {
      return {
        message: 'HEARTBEAT',
        type: 12, // MAV_TYPE_SUBMARINE
        autopilot: 3, // MAV_AUTOPILOT_ARDUPILOTMEGA
        base_mode: 128 | 64, // ARMED | MANUAL_INPUT_ENABLED
        custom_mode: 19, // ALT_HOLD / MANUAL mode
        system_status: 4, // MAV_STATE_ACTIVE
      };
    }

    if (msgType === 'ATTITUDE') {
      const [roll, pitch, yaw] = q2euler(state.slice(9, 13));
      return {
        message: 'ATTITUDE',
        time_boot_ms: Math.floor(timeBootMs),
        roll: Number(roll.toFixed(4)),
        pitch: Number(pitch.toFixed(4)),
        yaw: Number(yaw.toFixed(4)),
        rollspeed: Number(state[3].toFixed(4)),
        pitchspeed: Number(state[4].toFixed(4)),
        yawspeed: Number(state[5].toFixed(4)),
      };
    }

    if (msgType === 'GLOBAL_POSITION_INT') {
      // 10^7 scale for lat/lon, mm scale for depth
      const latOrigin = 60.3913 * 1e7;
      const lonOrigin = 5.3221 * 1e7;
      return {
        message: 'GLOBAL_POSITION_INT',
        time_boot_ms: Math.floor(timeBootMs),
        lat: Math.round(latOrigin + (state[6] / 111111) * 1e7),
        lon: Math.round(lonOrigin + (state[7] / (111111 * Math.cos(60.3913 * Math.PI / 180))) * 1e7),
        alt: Math.round(-state[8] * 1000), // mm altitude above surface (negative depth)
        relative_alt: Math.round(-state[8] * 1000),
        vx: Math.round(state[0] * 100), // cm/s
        vy: Math.round(state[1] * 100),
        vz: Math.round(state[2] * 100),
        hdg: Math.round((((q2euler(state.slice(9, 13))[2] * 180 / Math.PI) + 360) % 360) * 100), // cdeg
      };
    }

    if (msgType === 'SERVO_OUTPUT_RAW') {
      const pwm = thrusters.map(v => Math.round(1500 + v * 400));
      return {
        message: 'SERVO_OUTPUT_RAW',
        time_usec: Math.floor(timeBootMs * 1000),
        port: 0,
        servo1_raw: pwm[0] ?? 1500,
        servo2_raw: pwm[1] ?? 1500,
        servo3_raw: pwm[2] ?? 1500,
        servo4_raw: pwm[3] ?? 1500,
        servo5_raw: pwm[4] ?? 1500,
        servo6_raw: pwm[5] ?? 1500,
        servo7_raw: pwm[6] ?? 1500,
        servo8_raw: pwm[7] ?? 1500,
      };
    }

    throw new Error(`Unknown MAVLink message type: ${msgType}`);
  }

  /**
   * Decodes incoming MAVLink RC_CHANNELS_OVERRIDE (1100 - 1900 us) to normalized pilot commands.
   * @param {number[]} pwmChannels Array of 8 PWM integer values
   */
  parseMavlinkOverride(pwmChannels) {
    const normalize = pwm => {
      if (!pwm || pwm === 0 || pwm === 65535) return 0.0;
      return Math.min(1.0, Math.max(-1.0, (pwm - 1500) / 400));
    };

    return {
      pitch: normalize(pwmChannels[0]),
      roll: normalize(pwmChannels[1]),
      heave: normalize(pwmChannels[2]),
      yaw: normalize(pwmChannels[3]),
      surge: normalize(pwmChannels[4]),
      sway: normalize(pwmChannels[5]),
    };
  }

  /**
   * Encodes normalized pilot demands into 8-channel PWM integers (1100 - 1900 us).
   */
  encodeMavlinkOverride({ pitch = 0, roll = 0, heave = 0, yaw = 0, surge = 0, sway = 0 }) {
    const toPwm = val => Math.round(1500 + Math.min(1.0, Math.max(-1.0, val)) * 400);
    return [
      toPwm(pitch),
      toPwm(roll),
      toPwm(heave),
      toPwm(yaw),
      toPwm(surge),
      toPwm(sway),
      1500,
      1500,
    ];
  }
}
