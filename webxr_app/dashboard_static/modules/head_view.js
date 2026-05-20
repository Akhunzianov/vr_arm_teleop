import * as THREE from 'three';

// Wireframe camera-style frustum: apex at the head position, base extending
// along the head's forward axis (-Z in WebXR head-pose local frame). The
// helmet->three coordinate mapping is identity for our axes (see
// dashboard_scene.robotToThreeVector + helmetToRobotPosition compose to a
// no-op on basis vectors), so we can use the raw WebXR head orientation
// quaternion directly to point the frustum.
const DEPTH = 0.16;
const HALF_W = 0.10;
const HALF_H = 0.06;

function buildFrustumGeometry() {
  const apex = [0, 0, 0];
  const ftl = [-HALF_W,  HALF_H, -DEPTH];
  const ftr = [ HALF_W,  HALF_H, -DEPTH];
  const fbr = [ HALF_W, -HALF_H, -DEPTH];
  const fbl = [-HALF_W, -HALF_H, -DEPTH];
  const verts = [
    ...apex, ...ftl,  ...apex, ...ftr,  ...apex, ...fbr,  ...apex, ...fbl,
    ...ftl,  ...ftr,  ...ftr,  ...fbr,  ...fbr,  ...fbl,  ...fbl,  ...ftl,
  ];
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
  return geom;
}

export class HeadView {
  constructor(parent, { color = 0x49a3ff } = {}) {
    this.group = new THREE.Group();
    this.group.visible = false;
    const mat = new THREE.LineBasicMaterial({ color });
    this._lines = new THREE.LineSegments(buildFrustumGeometry(), mat);
    this.group.add(this._lines);
    parent.add(this.group);
  }

  setPose(position, orientation) {
    this.group.position.set(position.x, position.y, position.z);
    if (orientation && orientation.length === 4) {
      this.group.quaternion.set(
        orientation[0], orientation[1], orientation[2], orientation[3],
      );
    }
  }

  setVisible(v) { this.group.visible = !!v; }
}
