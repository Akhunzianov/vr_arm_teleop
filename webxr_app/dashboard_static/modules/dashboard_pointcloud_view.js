import * as THREE from 'three';
import { robotToThreeVector } from './dashboard_scene.js';

const MAX_POINTS = 200_000;
const HEADER_BYTES = 8;

export class DashboardPointCloudView {
  constructor(world, maxPoints = MAX_POINTS) {
    this.lastCount = 0;
    this._max = maxPoints;
    this._positions = new Float32Array(maxPoints * 3);
    this._colors = new Uint8Array(maxPoints * 3);

    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(this._positions, 3));
    geom.setAttribute('color', new THREE.BufferAttribute(this._colors, 3, true));
    geom.setDrawRange(0, 0);
    geom.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 100);

    const mat = new THREE.PointsMaterial({
      size: 0.008,
      vertexColors: true,
      sizeAttenuation: true,
    });

    this._points = new THREE.Points(geom, mat);
    this._geom = geom;
    this._posAttr = geom.getAttribute('position');
    this._colAttr = geom.getAttribute('color');
    world.add(this._points);
  }

  ingest(arrayBuffer) {
    const view = new DataView(arrayBuffer);
    const n = view.getUint32(0, true);
    if (n === 0) {
      this.lastCount = 0;
      this._geom.setDrawRange(0, 0);
      return;
    }
    const count = Math.min(n, this._max);
    const xs = new Int16Array(arrayBuffer, HEADER_BYTES, n);
    const ys = new Int16Array(arrayBuffer, HEADER_BYTES + 2 * n, n);
    const zs = new Int16Array(arrayBuffer, HEADER_BYTES + 4 * n, n);
    const rs = new Uint8Array(arrayBuffer, HEADER_BYTES + 6 * n, n);
    const gs = new Uint8Array(arrayBuffer, HEADER_BYTES + 7 * n, n);
    const bs = new Uint8Array(arrayBuffer, HEADER_BYTES + 8 * n, n);

    for (let i = 0; i < count; i++) {
      const v = robotToThreeVector([xs[i] * 0.001, ys[i] * 0.001, zs[i] * 0.001]);
      const p = i * 3;
      this._positions[p + 0] = v.x;
      this._positions[p + 1] = v.y;
      this._positions[p + 2] = v.z;
      this._colors[p + 0] = rs[i];
      this._colors[p + 1] = gs[i];
      this._colors[p + 2] = bs[i];
    }

    this.lastCount = count;
    this._posAttr.needsUpdate = true;
    this._colAttr.needsUpdate = true;
    this._geom.setDrawRange(0, count);
  }

  setVisible(visible) {
    this._points.visible = !!visible;
  }
}
