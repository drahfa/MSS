// Procedural 3D Subsea Manipulator Arm View for Three.js.
// Paper 4 & Milestone 2.2 — Subsea UVMS Robotic Intervention.

import * as THREE from 'three';

export class ManipulatorView {
  constructor(parentRovGroup) {
    this.root = new THREE.Group();
    this.root.name = 'uvms_manipulator_root';

    // Materials
    this.matMetal = new THREE.MeshStandardMaterial({
      color: 0x242e38,
      metalness: 0.85,
      roughness: 0.35,
    });
    this.matArm = new THREE.MeshStandardMaterial({
      color: 0xe67e22, // Safety subsea industrial orange
      metalness: 0.3,
      roughness: 0.4,
    });
    this.matPiston = new THREE.MeshStandardMaterial({
      color: 0xbdc3c7,
      metalness: 0.95,
      roughness: 0.15,
    });
    this.matJaw = new THREE.MeshStandardMaterial({
      color: 0x1a252f,
      metalness: 0.9,
      roughness: 0.25,
    });

    this.buildArmHierarchy();
    parentRovGroup.add(this.root);
  }

  buildArmHierarchy() {
    // 1. Mount bracket at ROV lower nose/chin
    const mountGeo = new THREE.CylinderGeometry(0.06, 0.07, 0.08, 16);
    this.mountMesh = new THREE.Mesh(mountGeo, this.matMetal);
    this.mountMesh.castShadow = true;
    this.root.add(this.mountMesh);

    // 2. Base Azimuth Pivot (Joint 1: yaw about Y)
    this.joint1 = new THREE.Group();
    this.joint1.position.set(0, 0.04, 0);
    this.root.add(this.joint1);

    const shoulderTurretGeo = new THREE.CylinderGeometry(0.05, 0.05, 0.07, 16);
    const shoulderTurret = new THREE.Mesh(shoulderTurretGeo, this.matMetal);
    shoulderTurret.castShadow = true;
    this.joint1.add(shoulderTurret);

    // 3. Shoulder Pitch Pivot (Joint 2: pitch about X)
    this.joint2 = new THREE.Group();
    this.joint2.position.set(0, 0.04, 0);
    this.joint1.add(this.joint2);

    // Upper arm link (Length 0.35m)
    const upperArmLength = 0.35;
    const upperArmGeo = new THREE.CylinderGeometry(0.035, 0.035, upperArmLength, 16);
    const upperArmMesh = new THREE.Mesh(upperArmGeo, this.matArm);
    upperArmMesh.position.set(0, 0, -upperArmLength / 2);
    upperArmMesh.rotation.x = Math.PI / 2;
    upperArmMesh.castShadow = true;
    this.joint2.add(upperArmMesh);

    // Hydraulic piston visual accent
    const pistonGeo = new THREE.CylinderGeometry(0.015, 0.015, upperArmLength * 0.75, 12);
    const pistonMesh = new THREE.Mesh(pistonGeo, this.matPiston);
    pistonMesh.position.set(0.04, 0.02, -upperArmLength / 2);
    pistonMesh.rotation.x = Math.PI / 2;
    this.joint2.add(pistonMesh);

    // 4. Elbow Pitch Pivot (Joint 3: pitch about X)
    this.joint3 = new THREE.Group();
    this.joint3.position.set(0, 0, -upperArmLength);
    this.joint2.add(this.joint3);

    const elbowHubGeo = new THREE.SphereGeometry(0.045, 16, 16);
    const elbowHub = new THREE.Mesh(elbowHubGeo, this.matMetal);
    elbowHub.castShadow = true;
    this.joint3.add(elbowHub);

    // Forearm link (Length 0.28m)
    const forearmLength = 0.28;
    const forearmGeo = new THREE.CylinderGeometry(0.028, 0.028, forearmLength, 16);
    const forearmMesh = new THREE.Mesh(forearmGeo, this.matArm);
    forearmMesh.position.set(0, 0, -forearmLength / 2);
    forearmMesh.rotation.x = Math.PI / 2;
    forearmMesh.castShadow = true;
    this.joint3.add(forearmMesh);

    // 5. End-Effector / Tool Wrist & Gripper
    this.wrist = new THREE.Group();
    this.wrist.position.set(0, 0, -forearmLength);
    this.joint3.add(this.wrist);

    const wristGeo = new THREE.CylinderGeometry(0.032, 0.032, 0.05, 16);
    const wristMesh = new THREE.Mesh(wristGeo, this.matMetal);
    wristMesh.rotation.x = Math.PI / 2;
    this.wrist.add(wristMesh);

    // Gripper Jaws
    const jawGeo = new THREE.BoxGeometry(0.015, 0.04, 0.08);
    this.jawLeft = new THREE.Mesh(jawGeo, this.matJaw);
    this.jawLeft.position.set(-0.025, 0, -0.05);
    this.jawRight = new THREE.Mesh(jawGeo, this.matJaw);
    this.jawRight.position.set(0.025, 0, -0.05);

    this.wrist.add(this.jawLeft, this.jawRight);
  }

  /**
   * Updates mount position and scales based on ROV class profile.
   */
  updateProfile(profile) {
    const scale = profile.leverScale ?? 1.0;
    // Position on chin in front (-Z) and below center (-Y)
    this.root.position.set(0, -0.18 * scale, -0.65 * scale);
  }

  /**
   * Updates joint mesh rotations from UVMS manipulator joint angles q.
   * @param {number[]|Float64Array} q Joint angles [azimuth, shoulder, elbow]
   * @param {number} jawGrip Normalized jaw opening [0=closed, 1=open]
   */
  updateJoints(q, jawGrip = 0.5) {
    if (!q) return;

    // Joint 1: Azimuth (about local Y)
    this.joint1.rotation.y = -(q[0] ?? 0);

    // Joint 2: Shoulder Pitch (about local X)
    this.joint2.rotation.x = (q[1] ?? 0);

    // Joint 3: Elbow Pitch (about local X)
    this.joint3.rotation.x = (q[2] ?? 0);

    // Jaw gripper opening
    const spread = 0.015 + (jawGrip ?? 0.5) * 0.025;
    this.jawLeft.position.x = -spread;
    this.jawRight.position.x = spread;
  }

  setVisible(visible) {
    this.root.visible = visible;
  }
}
