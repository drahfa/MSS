// N-Node Viscoelastic Lumped-Mass Umbilical Cable Model.
// Paper 3 & Milestone 2.1 — Advanced Subsea Operations.

export class LumpedMassTether {
  constructor(config = {}) {
    this.nodeCount = config.nodeCount ?? 12; // Default 12 nodes (11 segments)
    this.totalLength = config.totalLength ?? 70.0; // 70 meters total line
    this.diameter = config.diameter ?? 0.022; // 22 mm umbilical diameter
    this.density = config.density ?? 1150; // kg/m^3 (slightly negative buoyancy)
    this.waterDensity = config.waterDensity ?? 1025; // Seawater density
    this.elasticModulus = config.elasticModulus ?? 8e7; // Pa (effective axial modulus)
    this.dampingRatio = config.dampingRatio ?? 0.15;
    this.cdNormal = config.cdNormal ?? 1.2;
    this.cdTangent = config.cdTangent ?? 0.03;
    this.seabedDepth = config.seabedDepth ?? 18.0;

    this.segmentCount = this.nodeCount - 1;
    this.unstretchedSegLength = this.totalLength / this.segmentCount;

    const crossSectionArea = (Math.PI * this.diameter * this.diameter) / 4;
    this.segMass = this.density * crossSectionArea * this.unstretchedSegLength;
    this.segSubmergedWeight = (this.density - this.waterDensity) * crossSectionArea * this.unstretchedSegLength * 9.81;
    this.segStiffness = (this.elasticModulus * crossSectionArea) / this.unstretchedSegLength;
    this.segDamping = 2 * this.dampingRatio * Math.sqrt(this.segStiffness * this.segMass);

    // Flat pre-allocated typed arrays for N nodes: [x, y, z] in NED
    this.positions = new Float64Array(this.nodeCount * 3);
    this.velocities = new Float64Array(this.nodeCount * 3);
    this.forces = new Float64Array(this.nodeCount * 3);

    // Initial catenary staging: linear interpolation between surface clump and initial ROV position
    this.reset([0, 0, 0.5], [2, 0, 2]);
  }

  reset(surfacePos = [0, 0, 0.5], rovPos = [2, 0, 2]) {
    this.velocities.fill(0);
    this.forces.fill(0);

    for (let i = 0; i < this.nodeCount; i++) {
      const fraction = i / this.segmentCount;
      const idx = i * 3;
      // Staged catenary profile
      this.positions[idx] = surfacePos[0] + fraction * (rovPos[0] - surfacePos[0]);
      this.positions[idx + 1] = surfacePos[1] + fraction * (rovPos[1] - surfacePos[1]);
      this.positions[idx + 2] = surfacePos[2] + fraction * (rovPos[2] - surfacePos[2]) + Math.sin(fraction * Math.PI) * 1.5;
    }
  }

  step(dt, surfacePos, rovPos, rovVel, R_body_to_ned, currentSpeed = 0.18, currentDir = 2.14) {
    // 1. Enforce Dirichlet boundary condition at surface TMS (Node 0)
    this.positions[0] = surfacePos[0];
    this.positions[1] = surfacePos[1];
    this.positions[2] = surfacePos[2];
    this.velocities[0] = 0;
    this.velocities[1] = 0;
    this.velocities[2] = 0;

    // 2. Enforce boundary condition at ROV attachment (Node N-1)
    const lastIdx = (this.nodeCount - 1) * 3;
    this.positions[lastIdx] = rovPos[0];
    this.positions[lastIdx + 1] = rovPos[1];
    this.positions[lastIdx + 2] = rovPos[2];
    this.velocities[lastIdx] = rovVel[0];
    this.velocities[lastIdx + 1] = rovVel[1];
    this.velocities[lastIdx + 2] = rovVel[2];

    // Reset forces on all internal nodes
    this.forces.fill(0);

    const l0 = this.unstretchedSegLength;
    const vcNorth = currentSpeed * Math.cos(currentDir);
    const vcEast = currentSpeed * Math.sin(currentDir);

    let maxTension = 0;
    let terminalTensionVector = [0, 0, 0];

    // 3. Segment Viscoelastic Tension & Hydrodynamic Drag
    for (let k = 0; k < this.segmentCount; k++) {
      const iA = k * 3;
      const iB = (k + 1) * 3;

      const rx = this.positions[iB] - this.positions[iA];
      const ry = this.positions[iB + 1] - this.positions[iA + 1];
      const rz = this.positions[iB + 2] - this.positions[iA + 2];
      const len = Math.hypot(rx, ry, rz);

      if (len < 1e-6) continue;

      const tx = rx / len;
      const ty = ry / len;
      const tz = rz / len;

      const strain = (len - l0) / l0;
      const vRelX = this.velocities[iB] - this.velocities[iA];
      const vRelY = this.velocities[iB + 1] - this.velocities[iA + 1];
      const vRelZ = this.velocities[iB + 2] - this.velocities[iA + 2];
      const strainRate = (tx * vRelX + ty * vRelY + tz * vRelZ) / l0;

      // Unilateral elastic tension: zero under compression
      let tensionMag = 0;
      if (strain > 0) {
        tensionMag = this.segStiffness * strain + this.segDamping * strainRate;
        if (tensionMag < 0) tensionMag = 0;
      }

      if (tensionMag > maxTension) maxTension = tensionMag;

      const TenX = tensionMag * tx;
      const TenY = tensionMag * ty;
      const TenZ = tensionMag * tz;

      // Segment k pulls Node A in +t direction and Node B in -t direction
      this.forces[iA] += TenX;
      this.forces[iA + 1] += TenY;
      this.forces[iA + 2] += TenZ;

      this.forces[iB] -= TenX;
      this.forces[iB + 1] -= TenY;
      this.forces[iB + 2] -= TenZ;

      // Record terminal segment tension applied to ROV
      if (k === this.segmentCount - 1) {
        terminalTensionVector = [-TenX, -TenY, -TenZ];
      }

      // Morison Hydrodynamic Drag
      const midDepth = (this.positions[iA + 2] + this.positions[iB + 2]) / 2;
      const shearFactor = Math.max(0.2, 1.0 - 0.5 * (midDepth / this.seabedDepth));
      const segVcNorth = vcNorth * shearFactor;
      const segVcEast = vcEast * shearFactor;

      const segVx = (this.velocities[iA] + this.velocities[iB]) / 2 - segVcNorth;
      const segVy = (this.velocities[iA + 1] + this.velocities[iB + 1]) / 2 - segVcEast;
      const segVz = (this.velocities[iA + 2] + this.velocities[iB + 2]) / 2;

      const vTangDot = segVx * tx + segVy * ty + segVz * tz;
      const vTangX = vTangDot * tx;
      const vTangY = vTangDot * ty;
      const vTangZ = vTangDot * tz;

      const vNormX = segVx - vTangX;
      const vNormY = segVy - vTangY;
      const vNormZ = segVz - vTangZ;
      const vNormMag = Math.hypot(vNormX, vNormY, vNormZ);

      const dragNorm = 0.5 * this.waterDensity * this.cdNormal * this.diameter * len * vNormMag;
      const fDragX = -dragNorm * vNormX;
      const fDragY = -dragNorm * vNormY;
      const fDragZ = -dragNorm * vNormZ;

      // Distribute drag equally to adjacent nodes
      this.forces[iA] += fDragX * 0.5;
      this.forces[iA + 1] += fDragY * 0.5;
      this.forces[iA + 2] += fDragZ * 0.5;
      this.forces[iB] += fDragX * 0.5;
      this.forces[iB + 1] += fDragY * 0.5;
      this.forces[iB + 2] += fDragZ * 0.5;
    }

    // 4. Submerged Weight & Seabed Penalty Reactions for Internal Nodes
    const kBed = 22000; // N/m
    const cBed = 1200;  // N*s/m
    const muBed = 0.45;

    for (let i = 1; i < this.nodeCount - 1; i++) {
      const idx = i * 3;
      // Submerged gravity (acting down in NED frame: +Z)
      this.forces[idx + 2] += this.segSubmergedWeight;

      // Benthic contact
      const depth = this.positions[idx + 2];
      if (depth >= this.seabedDepth) {
        const penetration = depth - this.seabedDepth;
        const vZ = this.velocities[idx + 2];
        const normalReaction = Math.max(0, kBed * penetration + cBed * vZ);
        this.forces[idx + 2] -= normalReaction;

        // Friction opposing horizontal velocity
        const vH = Math.hypot(this.velocities[idx], this.velocities[idx + 1]);
        if (vH > 1e-4) {
          const fFriction = muBed * normalReaction;
          this.forces[idx] -= fFriction * (this.velocities[idx] / vH);
          this.forces[idx + 1] -= fFriction * (this.velocities[idx + 1] / vH);
        }
      }
    }

    // 5. Numerical Integration (Semi-Implicit Euler) for Interior Nodes
    const mNode = this.segMass;
    for (let i = 1; i < this.nodeCount - 1; i++) {
      const idx = i * 3;
      const ax = this.forces[idx] / mNode;
      const ay = this.forces[idx + 1] / mNode;
      const az = this.forces[idx + 2] / mNode;

      this.velocities[idx] += ax * dt;
      this.velocities[idx + 1] += ay * dt;
      this.velocities[idx + 2] += az * dt;

      // Damping velocity bleed for numerical passivity
      this.velocities[idx] *= 0.999;
      this.velocities[idx + 1] *= 0.999;
      this.velocities[idx + 2] *= 0.999;

      this.positions[idx] += this.velocities[idx] * dt;
      this.positions[idx + 1] += this.velocities[idx + 1] * dt;
      this.positions[idx + 2] += this.velocities[idx + 2] * dt;
    }

    // 6. Transform terminal tension to body frame wrench tau_tether^b
    // terminalTensionVector is in NED frame acting on ROV
    const F_body = [
      R_body_to_ned[0][0] * terminalTensionVector[0] + R_body_to_ned[1][0] * terminalTensionVector[1] + R_body_to_ned[2][0] * terminalTensionVector[2],
      R_body_to_ned[0][1] * terminalTensionVector[0] + R_body_to_ned[1][1] * terminalTensionVector[1] + R_body_to_ned[2][1] * terminalTensionVector[2],
      R_body_to_ned[0][2] * terminalTensionVector[0] + R_body_to_ned[1][2] * terminalTensionVector[1] + R_body_to_ned[2][2] * terminalTensionVector[2],
    ];

    return {
      wrenchBody: [F_body[0], F_body[1], F_body[2], 0, 0, 0],
      tensionMagnitude: Math.hypot(...terminalTensionVector),
      maxTension,
      nodes: this.getNodePositions(),
    };
  }

  getNodePositions() {
    const list = [];
    for (let i = 0; i < this.nodeCount; i++) {
      list.push([
        this.positions[i * 3],
        this.positions[i * 3 + 1],
        this.positions[i * 3 + 2],
      ]);
    }
    return list;
  }
}
