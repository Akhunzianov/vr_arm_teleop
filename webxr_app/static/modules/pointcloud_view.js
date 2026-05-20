// Renders the fused point cloud streamed from the server.
//
// Owns a pre-allocated THREE.Points with capacity for MAX_POINTS, plus
// the BufferAttributes used to update positions and colors each frame.
// ingest(ArrayBuffer) parses the binary wire format documented in
// teleop_core/point_cloud.py:
//
//   [uint32 N][uint32 reserved=0]
//   [N * int16 x_mm][N * int16 y_mm][N * int16 z_mm]
//   [N * uint8 r][N * uint8 g][N * uint8 b]

import * as THREE from 'three';

const MAX_POINTS = 200_000;
const HEADER_BYTES = 8;

export class PointCloudView {
  constructor(scene, maxPoints = MAX_POINTS) {
    this._max = maxPoints;
    this._positions = new Float32Array(maxPoints * 3);
    this._colors = new Uint8Array(maxPoints * 3);

    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(this._positions, 3));
    // normalized=true so the shader sees uint8 0-255 as float 0-1.
    geom.setAttribute('color', new THREE.BufferAttribute(this._colors, 3, true));
    geom.setDrawRange(0, 0);
    // Wide bounding sphere so the renderer never frustum-culls us before
    // the first real frame; cheap to compute properly later.
    geom.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 100);

    const mat = new THREE.PointsMaterial({
      size: 0.008,                 // 8 mm
      vertexColors: true,
      sizeAttenuation: true,
    });

    this._points = new THREE.Points(geom, mat);
    this._geom = geom;
    this._mat = mat;
    this._posAttr = geom.getAttribute('position');
    this._colAttr = geom.getAttribute('color');
    scene.add(this._points);
  }

  setVisible(v) {
    this._points.visible = !!v;
  }

  toggleVisible() {
    this._points.visible = !this._points.visible;
    return this._points.visible;
  }

  ingest(arrayBuffer) {
    const view = new DataView(arrayBuffer);
    const n = view.getUint32(0, true);
    if (n === 0) {
      this._geom.setDrawRange(0, 0);
      return;
    }
    if (n > this._max) {
      console.warn(`PointCloudView: dropping ${n - this._max} points beyond capacity`);
    }
    const count = Math.min(n, this._max);

    const xs = new Int16Array(arrayBuffer, HEADER_BYTES, n);
    const ys = new Int16Array(arrayBuffer, HEADER_BYTES + 2 * n, n);
    const zs = new Int16Array(arrayBuffer, HEADER_BYTES + 4 * n, n);
    const rs = new Uint8Array(arrayBuffer, HEADER_BYTES + 6 * n, n);
    const gs = new Uint8Array(arrayBuffer, HEADER_BYTES + 7 * n, n);
    const bs = new Uint8Array(arrayBuffer, HEADER_BYTES + 8 * n, n);

    const positions = this._positions;
    const colors = this._colors;
    for (let i = 0; i < count; i++) {
      const p = i * 3;
      positions[p + 0] = xs[i] * 0.001;
      positions[p + 1] = ys[i] * 0.001;
      positions[p + 2] = zs[i] * 0.001;
      colors[p + 0] = rs[i];
      colors[p + 1] = gs[i];
      colors[p + 2] = bs[i];
    }

    this._posAttr.needsUpdate = true;
    this._colAttr.needsUpdate = true;
    this._geom.setDrawRange(0, count);
  }

  setPointSize(meters) {
    this._mat.size = meters;
  }
}
