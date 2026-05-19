// Renders a Quest 2 Touch controller mesh at one WebXR controller's grip
// pose. The shipped FBX (assets/quest2/quest2_controllers_div0.fbx) bundles
// BOTH controllers as siblings at symmetric world-X; we drop the meshes on
// the wrong side at load time so each ControllerView only carries one
// controller.
//
// update(controllerSample) accepts InputReader's per-controller sample:
//   { position: [x,y,z], orientation: [x,y,z,w] }
// The FBX local axes / origin almost certainly don't match the WebXR grip
// frame; tune via setOffset(position, quaternion) once you see it in VR.

import * as THREE from 'three';
import { FBXLoader } from 'three/addons/loaders/FBXLoader.js';

const DEFAULT_URL = './assets/quest2/quest2_controllers_div0.fbx';

export class ControllerView {
  constructor(scene, {
    url = DEFAULT_URL,
    handed = 'left',
    scale = 0.01,
    offsetPosition = [0, 0, 0],
    // Composed below from yawDeg (face away) + tiltDeg (lean ring forward).
    offsetQuaternion = null,
    yawDeg = 180,
    tiltDeg = -20,
    bodyColor = 0xc8c8c8,
    bodyRoughness = 0.45,
    bodyMetalness = 0.05,
  } = {}) {
    this._handed = handed;
    this._group = new THREE.Group();
    this._group.visible = false;
    scene.add(this._group);

    this._inner = new THREE.Group();
    this._inner.position.set(...offsetPosition);
    if (offsetQuaternion) {
      this._inner.quaternion.set(...offsetQuaternion);
    } else {
      // Yaw first (around world Y), then tilt around world X. The tilt is
      // negative so the controller's top (ring) leans forward (-Z) instead
      // of toward the user.
      const qYaw = new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(0, 1, 0), yawDeg * Math.PI / 180);
      const qTilt = new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(1, 0, 0), tiltDeg * Math.PI / 180);
      this._inner.quaternion.multiplyQuaternions(qTilt, qYaw);
    }
    this._inner.scale.setScalar(scale);
    this._group.add(this._inner);

    new FBXLoader().load(url, (root) => {
      root.updateMatrixWorld(true);
      const wantSign = handed === 'left' ? -1 : 1;

      // Remove the other-side meshes by their world-bbox center X sign.
      const toRemove = [];
      const meshInfo = [];
      root.traverse((c) => {
        if (!c.isMesh) return;
        const cx = new THREE.Box3().setFromObject(c).getCenter(new THREE.Vector3()).x;
        meshInfo.push({ name: c.name, cx: +cx.toFixed(3) });
        if (cx !== 0 && Math.sign(cx) !== wantSign) toRemove.push(c);
      });
      console.log(`[controller_view:${handed}] meshes`, meshInfo);
      for (const m of toRemove) m.parent.remove(m);

      // Override the FBX's bundled materials (which read as nearly black
      // because their textures don't resolve when the FBX is served from
      // a sibling directory). Light-grey standard material across the
      // whole body matches what the user asked for.
      const overrideMat = new THREE.MeshStandardMaterial({
        color: bodyColor, roughness: bodyRoughness, metalness: bodyMetalness,
      });
      root.traverse((c) => {
        if (c.isMesh) c.material = overrideMat;
      });

      // Center the surviving mesh on the inner-group origin so the outer
      // group's position/quaternion (= grip pose) controls where the
      // controller appears, with the inner offset for fine tuning.
      this._inner.add(root);
      root.updateMatrixWorld(true);
      const box = new THREE.Box3().setFromObject(root);
      const c = box.getCenter(new THREE.Vector3());
      root.position.sub(c);
      console.log(`[controller_view:${handed}] kept ${root.children.length} child(ren); size`,
        box.getSize(new THREE.Vector3()).toArray());
    }, undefined, (err) => console.warn('[controller_view] load failed', err));
  }

  update(sample) {
    if (!sample || !sample.position) { this._group.visible = false; return; }
    this._group.visible = true;
    this._group.position.set(sample.position[0], sample.position[1], sample.position[2]);
    if (sample.orientation) {
      const q = sample.orientation;
      this._group.quaternion.set(q[0], q[1], q[2], q[3]);
    }
  }

  setOffset(position, quaternion) {
    if (position) this._inner.position.set(...position);
    if (quaternion) this._inner.quaternion.set(...quaternion);
  }
}
