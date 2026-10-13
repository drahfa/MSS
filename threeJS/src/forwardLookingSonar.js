// 2D Multibeam Forward-Looking Sonar (FLS) Simulation Engine.
// Paper 6 & Milestone 3.0 — Synthetic Acoustic Perception.

import { Rzyx } from './mss/library.js';

export class ForwardLookingSonar {
  constructor(config = {}) {
    this.beamCount = config.beamCount ?? 96; // 96 acoustic beams
    this.binCount = config.binCount ?? 128; // 128 range bins
    this.horizontalFovRad = config.horizontalFovRad ?? (120 * Math.PI / 180); // 120 deg azimuth
    this.verticalFovRad = config.verticalFovRad ?? (20 * Math.PI / 180); // 20 deg elevation fan beam
    this.tiltAngleRad = config.tiltAngleRad ?? 0.0; // 0 deg level default (fan covers [-10°, +10°])

    this.rMin = config.rMin ?? 0.5;
    this.rMax = config.rMax ?? 40.0; // 40 m in navigation mode
    this.frequencyKhz = config.frequencyKhz ?? 450; // 450 kHz default

    // Chemical absorption coefficient alpha (dB/m)
    this.alphaDb = this.frequencyKhz === 900 ? 0.285 : 0.088;

    // Beam-range intensity matrix [beams x bins]
    this.intensities = new Float32Array(this.beamCount * this.binCount);

    // Precompute beam azimuth angles
    this.beamAngles = new Float64Array(this.beamCount);
    const halfFov = this.horizontalFovRad / 2;
    for (let m = 0; m < this.beamCount; m++) {
      this.beamAngles[m] = -halfFov + (m + 0.5) * (this.horizontalFovRad / this.beamCount);
    }
  }

  setMode(mode) {
    if (mode === 'inspection') {
      this.rMax = 20.0;
      this.frequencyKhz = 900;
      this.alphaDb = 0.285;
    } else {
      this.rMax = 40.0;
      this.frequencyKhz = 450;
      this.alphaDb = 0.088;
    }
  }

  /**
   * Scans acoustic environment and populates the intensity matrix.
   * @param {number[]} rovPosNed Vehicle position [North, East, Down]
   * @param {number[]} rovEuler Vehicle attitude [roll, pitch, yaw]
   * @param {Array} obstacles List of scene obstacle objects { pos: [N, E, D], radius, strength }
   * @param {number} seabedDepth Depth of seafloor in meters (default 18.0)
   */
  scan(rovPosNed, rovEuler, obstacles = [], seabedDepth = 18.0) {
    const R = Rzyx(...rovEuler);
    const deltaR = (this.rMax - this.rMin) / this.binCount;

    // Reset intensities
    this.intensities.fill(0);

    // Default reference targets if none passed: Benthic habitat structure, seabed, and boulders
    const defaultObstacles = [
      { pos: [18.0, -10.0, 14.0], radius: 4.5, strength: 0.95, label: 'Underwater Research Habitat' },
      { pos: [12.0, -4.0, 17.0], radius: 1.8, strength: 0.85, label: 'Subsea Wellhead' },
      { pos: [8.0, 6.0, 16.5], radius: 2.2, strength: 0.75, label: 'Benthic Boulder' },
    ];
    const activeObstacles = obstacles.length > 0 ? obstacles : defaultObstacles;

    const vMin = this.tiltAngleRad - this.verticalFovRad / 2;
    const vMax = this.tiltAngleRad + this.verticalFovRad / 2;
    const beamWidth = this.horizontalFovRad / this.beamCount;

    for (let m = 0; m < this.beamCount; m++) {
      const az = this.beamAngles[m];

      // 1. Ray-obstacle intersection taking into account vertical fan beam
      let closestHitDist = Infinity;
      let hitStrength = 0;

      for (let o = 0; o < activeObstacles.length; o++) {
        const obs = activeObstacles[o];
        const dxN = obs.pos[0] - rovPosNed[0];
        const dyN = obs.pos[1] - rovPosNed[1];
        const dzN = obs.pos[2] - rovPosNed[2];

        // Transform relative target position into ROV body frame: p_b = R^T * p_n
        const dxB = R[0][0] * dxN + R[1][0] * dyN + R[2][0] * dzN;
        const dyB = R[0][1] * dxN + R[1][1] * dyN + R[2][1] * dzN;
        const dzB = R[0][2] * dxN + R[1][2] * dyN + R[2][2] * dzN;

        const dist = Math.sqrt(dxB * dxB + dyB * dyB + dzB * dzB);
        if (dist <= 0.001 || dxB <= 0.0) continue; // Behind ROV

        const targetAz = Math.atan2(dyB, dxB);
        const horizDist = Math.sqrt(dxB * dxB + dyB * dyB);
        const targetElev = Math.atan2(dzB, horizDist);
        const angularRad = Math.asin(Math.min(1.0, obs.radius / dist));

        // Check if obstacle is inside or overlaps vertical fan beam
        const inVerticalFan = (targetElev + angularRad >= vMin) && (targetElev - angularRad <= vMax);

        if (inVerticalFan) {
          const dAz = Math.abs(az - targetAz);
          if (dAz <= angularRad + beamWidth * 0.5) {
            // Distance to front surface along line of sight
            const surfaceDist = Math.max(this.rMin, dist - obs.radius);
            if (surfaceDist > this.rMin && surfaceDist < this.rMax && surfaceDist < closestHitDist) {
              closestHitDist = surfaceDist;
              hitStrength = obs.strength ?? 0.85;
            }
          }
        }
      }

      // 2. Seabed grazing backscatter intersection
      let seabedHitDist = Infinity;
      const dSeabed = seabedDepth - rovPosNed[2];
      if (dSeabed > 0) {
        // Find bottom ray of vertical elevation aperture
        const bottomElev = Math.max(0.02, vMax);
        const rayB = [Math.cos(bottomElev) * Math.cos(az), Math.cos(bottomElev) * Math.sin(az), Math.sin(bottomElev)];
        const rayN_z = R[2][0] * rayB[0] + R[2][1] * rayB[1] + R[2][2] * rayB[2];
        if (rayN_z > 0.05) {
          const rBed = dSeabed / rayN_z;
          if (rBed >= this.rMin && rBed <= this.rMax) {
            seabedHitDist = rBed;
          }
        }
      }

      // 3. Populate range bin intensities with TVG (Time Varied Gain) compensation
      for (let k = 0; k < this.binCount; k++) {
        const r = this.rMin + k * deltaR;
        const cellIdx = m * this.binCount + k;

        // Acoustic Shadow Region strictly behind solid obstacle
        if (r > closestHitDist + deltaR * 1.5) {
          this.intensities[cellIdx] = 0.0;
          continue;
        }

        // Structural Obstacle Echo Return
        if (Math.abs(r - closestHitDist) <= deltaR * 1.25) {
          const twoWayLoss = 40 * Math.log10(Math.max(1.0, r)) + 2 * this.alphaDb * r;
          const tvgGain = 32 * Math.log10(Math.max(1.0, r)) + 1.6 * this.alphaDb * r;
          const netAtten = Math.pow(10, -Math.max(0, twoWayLoss - tvgGain) / 40);
          this.intensities[cellIdx] = Math.min(1.0, Math.max(0.45, hitStrength * netAtten));
          continue;
        }

        // Seabed Grazing Backscatter
        if (Math.abs(r - seabedHitDist) <= deltaR * 1.5) {
          const twoWayLoss = 40 * Math.log10(Math.max(1.0, r)) + 2 * this.alphaDb * r;
          const tvgGain = 30 * Math.log10(Math.max(1.0, r)) + 1.5 * this.alphaDb * r;
          const netAtten = Math.pow(10, -Math.max(0, twoWayLoss - tvgGain) / 40);
          this.intensities[cellIdx] = Math.min(0.85, 0.5 * netAtten);
          continue;
        }

        // Ambient water-column volume reverberation / speckle noise
        const speckle = (Math.sin(m * 17.3 + k * 31.7) * 0.5 + 0.5) * 0.035;
        this.intensities[cellIdx] = Math.max(0, speckle);
      }
    }
  }

  /**
   * Renders polar sector sonar display onto an HTML5 Canvas.
   * @param {HTMLCanvasElement} canvas Target canvas element
   * @param {string} colormap 'amber' (default) or 'cyan'
   */
  renderDisplay(canvas, colormap = 'amber') {
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;

    // Dark sonar monitor background
    ctx.fillStyle = '#0a1014';
    ctx.fillRect(0, 0, w, h);

    const originX = w / 2;
    const originY = h - 14;
    const maxRadius = Math.min(w * 0.46, h - 28);

    const deltaR = (this.rMax - this.rMin) / this.binCount;
    const deltaAz = this.horizontalFovRad / this.beamCount;

    // Draw Sonar Phosphor Sector
    for (let m = 0; m < this.beamCount; m++) {
      const az = this.beamAngles[m];
      const sinA = Math.sin(az);
      const cosA = Math.cos(az);

      for (let k = 0; k < this.binCount; k += 2) { // Step by 2 for high rendering performance
        const intensity = this.intensities[m * this.binCount + k];
        if (intensity <= 0.015) continue;

        const rNorm = (this.rMin + k * deltaR) / this.rMax;
        const rad = rNorm * maxRadius;

        const px = originX + rad * sinA;
        const py = originY - rad * cosA;

        if (colormap === 'cyan') {
          const b = Math.min(255, Math.floor(intensity * 255));
          const g = Math.min(255, Math.floor(intensity * 220));
          ctx.fillStyle = `rgba(0, ${g}, ${b}, ${Math.min(1.0, intensity * 1.5)})`;
        } else {
          // Amber phosphor colormap (Kongsberg / Oculus style)
          const rCol = Math.min(255, Math.floor(intensity * 255));
          const gCol = Math.min(230, Math.floor(intensity * 180));
          ctx.fillStyle = `rgba(${rCol}, ${gCol}, 20, ${Math.min(1.0, intensity * 1.5)})`;
        }

        ctx.fillRect(px - 1.5, py - 1.5, 3, 3);
      }
    }

    // Overlay Graticule: Range Rings
    ctx.strokeStyle = 'rgba(70, 130, 140, 0.35)';
    ctx.lineWidth = 1;

    [0.25, 0.5, 0.75, 1.0].forEach(ratio => {
      const r = maxRadius * ratio;
      ctx.beginPath();
      const startAngle = -Math.PI / 2 - (this.horizontalFovRad / 2);
      const endAngle = -Math.PI / 2 + (this.horizontalFovRad / 2);
      ctx.arc(originX, originY, r, startAngle, endAngle);
      ctx.stroke();

      // Range text labels
      const distM = (this.rMax * ratio).toFixed(0);
      ctx.fillStyle = 'rgba(100, 180, 190, 0.75)';
      ctx.font = '10px monospace';
      ctx.fillText(`${distM}m`, originX + 4, originY - r + 3);
    });

    // Azimuth Guides (-60, -30, 0, +30, +60 deg)
    [-Math.PI / 3, -Math.PI / 6, 0, Math.PI / 6, Math.PI / 3].forEach(angle => {
      ctx.beginPath();
      ctx.moveTo(originX, originY);
      ctx.lineTo(originX + maxRadius * Math.sin(angle), originY - maxRadius * Math.cos(angle));
      ctx.stroke();
    });

    // Top Header Readout
    ctx.fillStyle = colormap === 'cyan' ? '#00e5ff' : '#f39c12';
    ctx.font = 'bold 11px monospace';
    ctx.fillText(`FLS ${this.frequencyKhz}kHz · R:${this.rMax}m · FOV:120°`, 8, 16);
  }
}
