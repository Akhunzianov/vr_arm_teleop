import * as THREE from 'three';
import { robotToThreeVector } from './dashboard_scene.js';

const EDGE_PAIRS = [
  [0, 1], [1, 3], [3, 2], [2, 0],
  [4, 5], [5, 7], [7, 6], [6, 4],
  [0, 4], [1, 5], [2, 6], [3, 7],
];

export class WorkspaceLayer {
  constructor(world) {
    this._geom = new THREE.BufferGeometry();
    this._positions = new Float32Array(EDGE_PAIRS.length * 2 * 3);
    this._geom.setAttribute('position', new THREE.BufferAttribute(this._positions, 3));
    this._mat = new THREE.LineBasicMaterial({ color: 0x49c4b5 });
    this._lines = new THREE.LineSegments(this._geom, this._mat);
    this._lines.visible = false;
    world.add(this._lines);
  }

  setBounds(min, max) {
    const corners = [
      [min[0], min[1], min[2]],
      [max[0], min[1], min[2]],
      [min[0], max[1], min[2]],
      [max[0], max[1], min[2]],
      [min[0], min[1], max[2]],
      [max[0], min[1], max[2]],
      [min[0], max[1], max[2]],
      [max[0], max[1], max[2]],
    ].map(robotToThreeVector);

    let offset = 0;
    for (const [a, b] of EDGE_PAIRS) {
      for (const v of [corners[a], corners[b]]) {
        this._positions[offset++] = v.x;
        this._positions[offset++] = v.y;
        this._positions[offset++] = v.z;
      }
    }
    this._geom.attributes.position.needsUpdate = true;
    this._geom.computeBoundingSphere();
    this._lines.visible = true;
  }

  setVisible(visible) {
    this._lines.visible = !!visible;
  }
}
