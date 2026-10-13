// Subsea Sensor Simulation Models: DVL, Depth Transducer, Magnetometer Compass, IMU, and Altimeter.
// Includes configurable Gaussian noise, bias drift, and physical dropout envelopes.

/**
 * Box-Muller transform for generating standard normal random numbers N(0, 1).
 */
export function gaussianNoise(mean = 0, stdDev = 1) {
  let u1 = 0, u2 = 0;
  while (u1 === 0) u1 = Math.random();
  while (u2 === 0) u2 = Math.random();
  const z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
  return mean + z0 * stdDev;
}

export class SensorSuite {
  constructor(config = {}) {
    this.config = {
      // DVL (Doppler Velocity Log)
      dvlStdDev: config.dvlStdDev ?? 0.005, // 5 mm/s velocity noise
      dvlMaxAltitude: config.dvlMaxAltitude ?? 25.0, // DVL bottom-lock lost if altitude > 25m
      // Depth pressure sensor
      depthStdDev: config.depthStdDev ?? 0.015, // 15 mm depth resolution
      // Compass / Heading
      compassStdDev: config.compassStdDev ?? (0.4 * Math.PI / 180), // 0.4 deg compass noise
      magneticDeclination: config.magneticDeclination ?? (1.2 * Math.PI / 180), // 1.2 deg declination
      // IMU (Gyros & Accelerometers)
      gyroStdDev: config.gyroStdDev ?? 0.002, // 2 mrad/s angular rate noise
      accelStdDev: config.accelStdDev ?? 0.04, // 0.04 m/s^2 specific force noise
      // Altimeter
      altimeterStdDev: config.altimeterStdDev ?? 0.02, // 20 mm echo sounder noise
      seabedDepth: config.seabedDepth ?? 18.0,
      enabled: config.enabled ?? true,
      ...config,
    };

    // Gyro random-walk bias drift
    this.gyroBias = [0.001, -0.0015, 0.0008];
  }

  /**
   * Sample all subsea instruments given true physical vehicle state and kinematics.
   * @param {number[]} state 13-state vector [u v w p q r N E D eta eps1 eps2 eps3]
   * @param {number[]} nuDot Generalized accelerations from plant
   * @param {number[][]} R SO(3) rotation matrix from body to NED
   * @param {number} dt Timestep in seconds
   * @returns {object} Sensor readings object
   */
  sample(state, nuDot, R, dt = 0.01) {
    const nu = state.slice(0, 6);
    const pos = state.slice(6, 9);
    const depth = pos[2];
    const altitude = Math.max(0, this.config.seabedDepth - depth);

    // Slowly drift gyro bias
    for (let i = 0; i < 3; i++) {
      this.gyroBias[i] += gaussianNoise(0, 0.00005 * Math.sqrt(dt));
    }

    if (!this.config.enabled) {
      // Return ideal noise-free readings
      return {
        dvl: { u: nu[0], v: nu[1], w: nu[2], bottomLock: altitude <= this.config.dvlMaxAltitude },
        depth: depth,
        altimeter: altitude,
        compass: Math.atan2(R[1][0], R[0][0]),
        imu: {
          rates: [nu[3], nu[4], nu[5]],
          accel: nuDot ? [nuDot[0], nuDot[1], nuDot[2] + 9.81 * R[2][2]] : [0, 0, 9.81],
        },
      };
    }

    // 1. DVL bottom-track velocity
    const bottomLock = altitude <= this.config.dvlMaxAltitude && altitude > 0.1;
    const dvl = bottomLock
      ? {
          u: nu[0] + gaussianNoise(0, this.config.dvlStdDev),
          v: nu[1] + gaussianNoise(0, this.config.dvlStdDev),
          w: nu[2] + gaussianNoise(0, this.config.dvlStdDev),
          bottomLock: true,
        }
      : { u: null, v: null, w: null, bottomLock: false };

    // 2. Hydrostatic Depth Transducer
    const depthMeasured = Math.max(0, depth + gaussianNoise(0, this.config.depthStdDev));

    // 3. Acoustic Altimeter (Echo Sounder)
    const altimeterMeasured = bottomLock
      ? Math.max(0, altitude + gaussianNoise(0, this.config.altimeterStdDev))
      : null;

    // 4. Fluxgate Compass
    const trueHeading = Math.atan2(R[1][0], R[0][0]);
    const compassMeasured = trueHeading + this.config.magneticDeclination + gaussianNoise(0, this.config.compassStdDev);

    // 5. 6-DOF IMU
    const rateGyros = [
      nu[3] + this.gyroBias[0] + gaussianNoise(0, this.config.gyroStdDev),
      nu[4] + this.gyroBias[1] + gaussianNoise(0, this.config.gyroStdDev),
      nu[5] + this.gyroBias[2] + gaussianNoise(0, this.config.gyroStdDev),
    ];

    // Accelerometer measures specific force in body frame: f_b = a_b - R^T * g_n
    const g_body = [R[2][0] * 9.81, R[2][1] * 9.81, R[2][2] * 9.81];
    const a_body = nuDot ? nuDot.slice(0, 3) : [0, 0, 0];
    const specificForce = [
      a_body[0] - g_body[0] + gaussianNoise(0, this.config.accelStdDev),
      a_body[1] - g_body[1] + gaussianNoise(0, this.config.accelStdDev),
      a_body[2] - g_body[2] + gaussianNoise(0, this.config.accelStdDev),
    ];

    return {
      dvl,
      depth: depthMeasured,
      altimeter: altimeterMeasured,
      compass: compassMeasured,
      imu: {
        rates: rateGyros,
        accel: specificForce,
      },
    };
  }
}
