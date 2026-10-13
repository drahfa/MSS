// 3D Multi-ROV Fleet, Dual Lumped-Mass Umbilical Splines, and Anti-Entanglement Clearance Visualizer.
// Paper 5 & Milestone 2.3 — Multi-Agent Subsea Digital Twin.

import * as THREE from 'three';
import { LumpedMassTether } from './lumpedMassTether.js';
import { MultiRovTetherBridge } from './multiRovTetherBridge.js';

export class MultiRovView {
  constructor(scene) {
    this.scene = scene;
    this.bridge = new MultiRovTetherBridge({ warnDistance: 3.0, critDistance: 1.0 });

    // Secondary ROV Follower Group
    this.followerGroup = new THREE.Group();
    this.followerGroup.name = 'follower_rov_group';
    this.followerGroup.visible = false;
    this.scene.add(this.followerGroup);

    // Procedural Observation Eyeball ROV Hull for Follower
    this.buildFollowerMesh();

    // 12-node lumped mass tether simulation for Follower
    this.followerTetherSim = new LumpedMassTether({
      nodeCount: 12,
      totalLength: 70.0,
      seabedDepth: 18.0,
    });
    this.followerPosNed = [4.0, 5.0, 3.5]; // Default station-keeping offset [N, E, D]

    // Line Materials
    this.matPrimaryTether = new THREE.LineBasicMaterial({
      color: 0x00e5ff, // Bright Cyan Umbilical
      linewidth: 3,
    });
    this.matFollowerTether = new THREE.LineBasicMaterial({
      color: 0xffaa00, // Amber Umbilical
      linewidth: 3,
    });
    this.matClearanceVector = new THREE.LineBasicMaterial({
      color: 0x2ecc71, // Green (Safe)
      linewidth: 2,
      transparent: true,
      opacity: 0.85,
    });

    // 1. Primary ROV Dynamic Umbilical Line
    const primGeo = new THREE.BufferGeometry();
    const primPositions = new Float32Array(12 * 3);
    primGeo.setAttribute('position', new THREE.BufferAttribute(primPositions, 3));
    this.primaryLine = new THREE.Line(primGeo, this.matPrimaryTether);
    this.scene.add(this.primaryLine);

    // 2. Follower ROV Dynamic Umbilical Line
    const folGeo = new THREE.BufferGeometry();
    const folPositions = new Float32Array(12 * 3);
    folGeo.setAttribute('position', new THREE.BufferAttribute(folPositions, 3));
    this.followerLine = new THREE.Line(folGeo, this.matFollowerTether);
    this.followerLine.visible = false;
    this.scene.add(this.followerLine);

    // 3. Dynamic Clearance Vector Line (Connecting closest cable points)
    const clrGeo = new THREE.BufferGeometry();
    const clrPositions = new Float32Array(2 * 3);
    clrGeo.setAttribute('position', new THREE.BufferAttribute(clrPositions, 3));
    this.clearanceLine = new THREE.Line(clrGeo, this.matClearanceVector);
    this.clearanceLine.visible = false;
    this.scene.add(this.clearanceLine);

    // Surface TMS Anchor Marker
    const tmsGeo = new THREE.CylinderGeometry(0.4, 0.4, 0.3, 16);
    const tmsMat = new THREE.MeshStandardMaterial({ color: 0xe74c3c, metalness: 0.8, roughness: 0.3 });
    this.tmsMarker = new THREE.Mesh(tmsGeo, tmsMat);
    this.tmsMarker.position.set(0, -0.5, 0); // At surface [E=0, Y=-0.5, N=0]
    this.scene.add(this.tmsMarker);
  }

  buildFollowerMesh() {
    const hullMat = new THREE.MeshStandardMaterial({
      color: 0xffbb00, // High-visibility bright yellow observation hull
      metalness: 0.4,
      roughness: 0.5,
    });
    const frameMat = new THREE.MeshStandardMaterial({
      color: 0x1f2937,
      metalness: 0.8,
      roughness: 0.3,
    });

    // Spherical/cylindrical observation ROV
    const hullGeo = new THREE.CylinderGeometry(0.28, 0.28, 0.65, 20);
    const hull = new THREE.Mesh(hullGeo, hullMat);
    hull.rotation.z = Math.PI / 2;
    hull.castShadow = true;
    this.followerGroup.add(hull);

    // Crash frame
    const frameGeo = new THREE.BoxGeometry(0.85, 0.45, 0.65);
    const frame = new THREE.Mesh(frameGeo, frameMat);
    frame.castShadow = true;
    this.followerGroup.add(frame);

    // Eyeball camera dome
    const domeGeo = new THREE.SphereGeometry(0.14, 16, 16);
    const domeMat = new THREE.MeshStandardMaterial({ color: 0x111111, roughness: 0.1 });
    const dome = new THREE.Mesh(domeGeo, domeMat);
    dome.position.set(0, 0, -0.35);
    this.followerGroup.add(dome);

    // LED floodlights
    [-0.25, 0.25].forEach(x => {
      const ledGeo = new THREE.CylinderGeometry(0.04, 0.04, 0.08, 12);
      const ledMat = new THREE.MeshBasicMaterial({ color: 0x00ffff });
      const led = new THREE.Mesh(ledGeo, ledMat);
      led.rotation.x = Math.PI / 2;
      led.position.set(x, 0.1, -0.34);
      this.followerGroup.add(led);
    });
  }

  setFollowerEnabled(enabled) {
    this.followerGroup.visible = enabled;
    this.followerLine.visible = enabled;
    this.clearanceLine.visible = enabled;
  }

  /**
   * Updates all dynamic umbilical cables and multi-tether anti-entanglement spatial clearance.
   * @param {number} dt Frame timestep
   * @param {number[]} primaryPosNed [N, E, D] of Lead ROV
   * @param {LumpedMassTether} primaryTetherSim Primary vehicle lumped mass tether instance
   */
  update(dt, primaryPosNed, primaryTetherSim) {
    // 1. Update Primary Umbilical Cable Nodes
    const primNodes = primaryTetherSim.getNodePositions();
    const primPosAttr = this.primaryLine.geometry.attributes.position;
    for (let i = 0; i < primNodes.length; i++) {
      const [n, e, d] = primNodes[i];
      // Convert NED -> Three.js (X=E, Y=-D, Z=-N)
      primPosAttr.setXYZ(i, e, -d, -n);
    }
    primPosAttr.needsUpdate = true;

    if (!this.followerGroup.visible) {
      return { active: false, clearance: null };
    }

    // 2. Animate Follower ROV Station-Keeping around primary vehicle
    // Station keeps at offset [N + 4, E + 5, D + 1.5]
    const targetFolN = primaryPosNed[0] + 3.5;
    const targetFolE = primaryPosNed[1] + 4.5;
    const targetFolD = Math.max(1.5, primaryPosNed[2] - 0.5);

    this.followerPosNed[0] += (targetFolN - this.followerPosNed[0]) * 0.05;
    this.followerPosNed[1] += (targetFolE - this.followerPosNed[1]) * 0.05;
    this.followerPosNed[2] += (targetFolD - this.followerPosNed[2]) * 0.05;

    // Position Follower mesh in Three.js coordinates
    this.followerGroup.position.set(
      this.followerPosNed[1],
      -this.followerPosNed[2],
      -this.followerPosNed[0],
    );

    // Step Follower Tether Sim from second surface TMS at [0, 4, 0.5]
    const folSurfaceTms = [0, 3.5, 0.5];
    const folR = [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ];
    this.followerTetherSim.step(dt, folSurfaceTms, this.followerPosNed, [0, 0, 0], folR);

    // Update Follower Cable Geometry
    const folNodes = this.followerTetherSim.getNodePositions();
    const folPosAttr = this.followerLine.geometry.attributes.position;
    for (let i = 0; i < folNodes.length; i++) {
      const [n, e, d] = folNodes[i];
      folPosAttr.setXYZ(i, e, -d, -n);
    }
    folPosAttr.needsUpdate = true;

    // 3. Compute Real-Time 3D Spatial Clearance between both 12-node cables
    const clearance = this.bridge.computeTetherClearance(primNodes, folNodes);

    // Update Dynamic Clearance Line between closest segments
    const segA = Math.min(clearance.closestSegA, primNodes.length - 1);
    const segB = Math.min(clearance.closestSegB, folNodes.length - 1);
    const [nA, eA, dA] = primNodes[segA];
    const [nB, eB, dB] = folNodes[segB];

    const clrAttr = this.clearanceLine.geometry.attributes.position;
    clrAttr.setXYZ(0, eA, -dA, -nA);
    clrAttr.setXYZ(1, eB, -dB, -nB);
    clrAttr.needsUpdate = true;

    // Update color based on entanglement safety envelope
    if (clearance.criticalAlert) {
      this.matClearanceVector.color.setHex(0xe74c3c); // RED (CRITICAL ALERT)
    } else if (clearance.advisory) {
      this.matClearanceVector.color.setHex(0xf39c12); // AMBER (PROXIMITY WARNING)
    } else {
      this.matClearanceVector.color.setHex(0x2ecc71); // GREEN (SAFE)
    }

    return {
      active: true,
      clearance,
      followerPos: [...this.followerPosNed],
    };
  }
}
