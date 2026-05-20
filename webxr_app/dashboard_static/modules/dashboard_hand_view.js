// Loads the same RightHand.fbx the VR app uses, drives the wrist pose
// from the WebXR right-wrist sample, and bends the finger bones from the
// 5-vector of normalized curls the server now publishes alongside the
// wrist. Per-joint positions aren't in the snapshot, so the curl-driven
// bend (same shape as HandView.driveCurls) is what gives us moving
// fingers without paying the bandwidth of streaming 25 joints.

import * as THREE from 'three';
import { FBXLoader } from 'three/addons/loaders/FBXLoader.js';

const DEFAULT_URL = './assets/RightHand.fbx';
const FALLBACK_COLOR = 0xeecbbf;

const JOINT_TO_BONE_SUFFIX = {
  2: 'thumb1',  3: 'thumb2',  4: 'thumb3',
  6: 'index1',  7: 'index2',  8: 'index3',
  11: 'middle1', 12: 'middle2', 13: 'middle3',
  16: 'ring1', 17: 'ring2', 18: 'ring3',
  20: 'pinky0', 21: 'pinky1', 22: 'pinky2', 23: 'pinky3',
};
const JOINT_NEXT = {
  2: 3,  3: 4,
  6: 7,  7: 8,  8: 9,
  11: 12, 12: 13, 13: 14,
  16: 17, 17: 18, 18: 19,
  20: 21, 21: 22, 22: 23, 23: 24,
};
const JOINT_TO_FINGER_IDX = {
  2: 0, 3: 0, 4: 0,
  6: 1, 7: 1, 8: 1,
  11: 2, 12: 2, 13: 2,
  16: 3, 17: 3, 18: 3,
  20: 4, 21: 4, 22: 4, 23: 4,
};
const SEGMENT_CURL_MAX_RAD = (Math.PI / 180) * 70;

export class DashboardHandView {
  constructor(parent, {
    url = DEFAULT_URL,
    scale = 0.01,
    offsetPosition = [-0.03, -0.05, -0.1],
    offsetQuaternion = [0, 1, 0, 0],   // 180 deg around Y, matches VR HandView
  } = {}) {
    this._group = new THREE.Group();
    this._group.visible = false;
    parent.add(this._group);

    this._inner = new THREE.Group();
    this._inner.position.set(...offsetPosition);
    this._inner.quaternion.set(...offsetQuaternion);
    this._inner.scale.setScalar(scale);
    this._group.add(this._inner);

    // jointIdx -> { bone, restLocalQ, restParentDir }
    this._bones = {};
    this._pendingCurls = null;

    new FBXLoader().load(url, (root) => {
      root.traverse((c) => {
        if (!c.isMesh) return;
        const m = Array.isArray(c.material) ? c.material[0] : c.material;
        if (!m || m.map == null) {
          c.material = new THREE.MeshStandardMaterial({
            color: FALLBACK_COLOR, roughness: 0.6, metalness: 0.0,
          });
        }
      });
      this._inner.add(root);
      this._inner.updateMatrixWorld(true);

      const suffixToJoint = {};
      for (const [j, s] of Object.entries(JOINT_TO_BONE_SUFFIX)) {
        suffixToJoint[s] = parseInt(j);
      }

      let wristBone = null;
      root.traverse((c) => {
        if (!c.isBone) return;
        if (!wristBone && /wrist/i.test(c.name)) wristBone = c;
        const m = c.name.match(/b_[lr]_([a-z]+\d)/);
        if (!m) return;
        const j = suffixToJoint[m[1]];
        if (j === undefined) return;
        this._bones[j] = { bone: c, restLocalQ: c.quaternion.clone(), restParentDir: null };
      });

      // Anchor FBX wrist bone to this._group's origin so tracked wrist
      // pose drives the visible wrist, not the FBX root.
      if (wristBone) {
        wristBone.updateMatrixWorld(true);
        const wristWorld = new THREE.Vector3().setFromMatrixPosition(wristBone.matrixWorld);
        const wristInGroup = this._group.worldToLocal(wristWorld.clone());
        this._inner.position.sub(wristInGroup);
        this._inner.updateMatrixWorld(true);
      }

      // restParentDir: unit vector from this bone to the next bone in
      // its chain, expressed in the parent's frame. Same construction as
      // HandView so driveCurls below matches the VR app's behaviour.
      for (const j in this._bones) {
        const { bone, restLocalQ } = this._bones[j];
        const nextJ = JOINT_NEXT[j];
        let nextBone = null;
        if (nextJ != null && this._bones[nextJ]) nextBone = this._bones[nextJ].bone;
        if (!nextBone) {
          nextBone = bone.children.find(ch => ch.isBone) || null;
        }
        if (!nextBone) continue;
        const dir = nextBone.position.clone().applyQuaternion(restLocalQ);
        if (dir.lengthSq() < 1e-12) continue;
        dir.normalize();
        this._bones[j].restParentDir = dir;
      }

      // Apply any curls that arrived before the FBX finished loading.
      if (this._pendingCurls) {
        this.driveCurls(this._pendingCurls);
        this._pendingCurls = null;
      }
    }, undefined, (err) => console.warn('[dashboard_hand_view] load failed', err));
  }

  setPose(position, orientation) {
    this._group.position.set(position.x, position.y, position.z);
    if (orientation && orientation.length === 4) {
      this._group.quaternion.set(
        orientation[0], orientation[1], orientation[2], orientation[3],
      );
    }
  }

  // Bend finger bones from a 5-vector of normalized curls (thumb..pinky,
  // 0..1). Mirror of HandView.driveCurls so the dashboard hand follows
  // the same shape as the ghost/live hand in VR.
  driveCurls(curls) {
    if (!curls) return;
    if (Object.keys(this._bones).length === 0) {
      this._pendingCurls = curls;
      return;
    }
    const indices = Object.keys(this._bones).map(Number).sort((a, b) => a - b);
    const _palmGuess = new THREE.Vector3(0, 1, 0);
    const _bendAxis = new THREE.Vector3();
    const _desired = new THREE.Vector3();
    const _qBend = new THREE.Quaternion();
    const _qAlign = new THREE.Quaternion();
    for (const j of indices) {
      const info = this._bones[j];
      if (!info.restParentDir) continue;
      const fingerIdx = JOINT_TO_FINGER_IDX[j];
      if (fingerIdx == null) continue;
      const c = Math.max(0, Math.min(1, curls[fingerIdx] ?? 0));
      const angle = c * SEGMENT_CURL_MAX_RAD;
      _bendAxis.crossVectors(info.restParentDir, _palmGuess);
      if (_bendAxis.lengthSq() < 1e-8) _bendAxis.set(0, 0, 1);
      _bendAxis.normalize();
      _qBend.setFromAxisAngle(_bendAxis, angle);
      _desired.copy(info.restParentDir).applyQuaternion(_qBend);
      _qAlign.setFromUnitVectors(info.restParentDir, _desired);
      info.bone.quaternion.copy(_qAlign).multiply(info.restLocalQ);
      info.bone.updateMatrix();
      info.bone.updateMatrixWorld(true);
    }
  }

  setVisible(v) { this._group.visible = !!v; }
}
