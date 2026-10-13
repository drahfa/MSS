// Contact Mechanics Model: Seabed and Subsea Structure Collision Response.
// Uses an elastic-viscous penalty contact model with tangential Coulomb friction.

export class ContactModel {
  constructor(config = {}) {
    this.config = {
      seabedDepth: config.seabedDepth ?? 18.0, // NED Down coordinate (meters)
      hullRadius: config.hullRadius ?? 0.45, // Effective bounding sphere radius (meters)
      springConstant: config.springConstant ?? 18000.0, // N/m contact stiffness
      dampingConstant: config.dampingConstant ?? 1500.0, // N*s/m contact damping
      frictionCoeff: config.frictionCoeff ?? 0.35, // Seabed mud/sand Coulomb friction coefficient
      // Subsea habitat obstacle (North, East, Down radius)
      habitatCenterNed: config.habitatCenterNed ?? [15.0, -7.0, 16.0],
      habitatRadius: config.habitatRadius ?? 3.5,
      ...config,
    };
  }

  /**
   * Evaluates contact forces acting on the vehicle body from seabed and structures.
   * @param {number[]} pos Position in NED [N, E, D]
   * @param {number[]} nu1 Body linear velocity [u, v, w]
   * @param {number[][]} R SO(3) body-to-NED rotation matrix
   * @returns {object} { contactForceBody, isSeabedContact, isStructureContact, penetrationDepth }
   */
  evaluate(pos, nu1, R) {
    const contactForceNed = [0, 0, 0];
    let isSeabedContact = false;
    let isStructureContact = false;
    let maxPenetration = 0;

    // Convert body velocity to NED velocity: v_ned = R * nu1
    const v_ned = [
      R[0][0] * nu1[0] + R[0][1] * nu1[1] + R[0][2] * nu1[2],
      R[1][0] * nu1[0] + R[1][1] * nu1[1] + R[1][2] * nu1[2],
      R[2][0] * nu1[0] + R[2][1] * nu1[1] + R[2][2] * nu1[2],
    ];

    // 1. Seabed plane contact (z = seabedDepth)
    const lowestPoint = pos[2] + this.config.hullRadius;
    const seabedPenetration = lowestPoint - this.config.seabedDepth;

    if (seabedPenetration > 0) {
      isSeabedContact = true;
      maxPenetration = Math.max(maxPenetration, seabedPenetration);

      // Normal restoring force pushes UP (negative Down direction)
      const normalSpring = this.config.springConstant * seabedPenetration;
      const normalDamping = this.config.dampingConstant * Math.max(0, v_ned[2]);
      const normalForce = normalSpring + normalDamping;

      contactForceNed[2] -= normalForce;

      // Tangential Coulomb friction opposing horizontal slip
      const slipSpeed = Math.hypot(v_ned[0], v_ned[1]);
      if (slipSpeed > 1e-4) {
        const frictionMag = Math.min(normalForce * this.config.frictionCoeff, 2500.0);
        contactForceNed[0] -= frictionMag * (v_ned[0] / slipSpeed);
        contactForceNed[1] -= frictionMag * (v_ned[1] / slipSpeed);
      }
    }

    // 2. Subsea habitat obstacle sphere contact
    const habCenter = this.config.habitatCenterNed;
    const deltaHab = [pos[0] - habCenter[0], pos[1] - habCenter[1], pos[2] - habCenter[2]];
    const distHab = Math.hypot(deltaHab[0], deltaHab[1], deltaHab[2]);
    const minClearance = this.config.habitatRadius + this.config.hullRadius;

    if (distHab < minClearance && distHab > 1e-3) {
      isStructureContact = true;
      const structPenetration = minClearance - distHab;
      maxPenetration = Math.max(maxPenetration, structPenetration);

      const normDir = [deltaHab[0] / distHab, deltaHab[1] / distHab, deltaHab[2] / distHab];
      // Velocity along penetration vector
      const vNormal = v_ned[0] * normDir[0] + v_ned[1] * normDir[1] + v_ned[2] * normDir[2];

      const fStruct = this.config.springConstant * structPenetration - this.config.dampingConstant * Math.min(0, vNormal);
      contactForceNed[0] += fStruct * normDir[0];
      contactForceNed[1] += fStruct * normDir[1];
      contactForceNed[2] += fStruct * normDir[2];
    }

    // Map contact force from NED to BODY frame: F_b = R^T * F_ned
    const contactForceBody = [
      R[0][0] * contactForceNed[0] + R[1][0] * contactForceNed[1] + R[2][0] * contactForceNed[2],
      R[0][1] * contactForceNed[0] + R[1][1] * contactForceNed[1] + R[2][1] * contactForceNed[2],
      R[0][2] * contactForceNed[0] + R[1][2] * contactForceNed[1] + R[2][2] * contactForceNed[2],
    ];

    return {
      contactForceBody,
      contactForceNed,
      isSeabedContact,
      isStructureContact,
      penetrationDepth: maxPenetration,
    };
  }
}
