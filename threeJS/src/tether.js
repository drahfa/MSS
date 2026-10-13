// Umbilical Tether Simulation Model: Catenary payout, hydrodynamic line drag, tension, and snag alerts.

export class TetherModel {
  constructor(config = {}) {
    this.config = {
      surfaceAnchor: config.surfaceAnchor ?? [0, 0, 0], // Surface ship / buoy position in NED
      maxLength: config.maxLength ?? 120.0, // Max spooled tether length in meters
      diameter: config.diameter ?? 0.018, // 18 mm umbilical cable
      dragCoeff: config.dragCoeff ?? 1.2, // Cross-flow cylinder drag coefficient
      densityWater: config.densityWater ?? 1025.0, // Seawater density kg/m^3
      catenarySlackRatio: config.catenarySlackRatio ?? 0.08, // 8% catenary sag
      elasticStiffness: config.elasticStiffness ?? 850.0, // N/m cable extension stiffness past max payout
      warningThresholdRatio: config.warningThresholdRatio ?? 0.88, // Trigger warning at 88% payout
      ...config,
    };
  }

  /**
   * Computes tether state, drag force in body frame, tension, and snag warnings.
   * @param {number[]} pos Vehicle position in NED [N, E, D]
   * @param {number[]} nu1 Body linear velocity [u, v, w]
   * @param {number[][]} R SO(3) body-to-NED rotation matrix
   * @returns {object} { payoutLength, tension, dragForceBody, isNearLimit, isSnagged, warning }
   */
  compute(pos, nu1, R) {
    const anchor = this.config.surfaceAnchor;
    const delta = [pos[0] - anchor[0], pos[1] - anchor[1], pos[2] - anchor[2]];
    const directDistance = Math.hypot(delta[0], delta[1], delta[2]);

    // Catenary payout with sag
    const payoutLength = directDistance * (1 + this.config.catenarySlackRatio);
    const maxLength = this.config.maxLength;

    // Check snag and payout thresholds
    const isNearLimit = payoutLength >= maxLength * this.config.warningThresholdRatio;
    const isSnagged = payoutLength >= maxLength;

    let warning = null;
    if (isSnagged) {
      warning = 'TETHER SNAGGED: Maximum cable spool limit reached';
    } else if (isNearLimit) {
      warning = 'TETHER ADVISORY: Approaching maximum cable payout';
    }

    // Hydrodynamic line drag opposing vehicle linear velocity
    // F_drag = -0.5 * rho * Cd * (diameter * length) * |nu| * nu
    const speed = Math.hypot(nu1[0], nu1[1], nu1[2]);
    const frontalArea = this.config.diameter * payoutLength;
    const dragMag = 0.5 * this.config.densityWater * this.config.dragCoeff * frontalArea * speed;
    const dragForceBody = [
      -dragMag * nu1[0],
      -dragMag * nu1[1],
      -dragMag * nu1[2],
    ];

    // Restraining elastic tension if cable pulled taut past max spool length
    let tension = 0;
    const tensionForceBody = [0, 0, 0];
    if (isSnagged && directDistance > 1e-3) {
      const extension = payoutLength - maxLength;
      tension = this.config.elasticStiffness * extension;
      // Pull direction towards surface anchor in NED
      const unitDirNed = [-delta[0] / directDistance, -delta[1] / directDistance, -delta[2] / directDistance];
      // Map tension from NED to body: F_b = R^T * F_n
      const tensionNed = [unitDirNed[0] * tension, unitDirNed[1] * tension, unitDirNed[2] * tension];
      tensionForceBody[0] = R[0][0] * tensionNed[0] + R[1][0] * tensionNed[1] + R[2][0] * tensionNed[2];
      tensionForceBody[1] = R[0][1] * tensionNed[0] + R[1][1] * tensionNed[1] + R[2][1] * tensionNed[2];
      tensionForceBody[2] = R[0][2] * tensionNed[0] + R[1][2] * tensionNed[1] + R[2][2] * tensionNed[2];
    }

    return {
      payoutLength,
      directDistance,
      tension,
      dragForceBody,
      tensionForceBody,
      totalForceBody: [
        dragForceBody[0] + tensionForceBody[0],
        dragForceBody[1] + tensionForceBody[1],
        dragForceBody[2] + tensionForceBody[2],
      ],
      isNearLimit,
      isSnagged,
      warning,
    };
  }
}
