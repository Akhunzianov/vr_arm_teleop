import * as THREE from 'three';
import { robotToThreeVector } from './dashboard_scene.js';

function transformPoint(matrix, point) {
  const [x, y, z] = point;
  return [
    matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + matrix[0][3],
    matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + matrix[1][3],
    matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + matrix[2][3],
  ];
}

function line(points, color) {
  const geom = new THREE.BufferGeometry().setFromPoints(points);
  const mat = new THREE.LineBasicMaterial({ color });
  return new THREE.LineSegments(geom, mat);
}

function disposeObject(obj) {
  if (obj.geometry) obj.geometry.dispose();
  if (obj.material) obj.material.dispose();
}

export class CalibrationView {
  constructor(world) {
    this.group = new THREE.Group();
    world.add(this.group);
  }

  update(calibration) {
    this.group.traverse(disposeObject);
    this.group.clear();
    if (!calibration) return;

    const transforms = calibration.world_from_camera || {};
    const cameras = calibration.cameras || [];
    for (const camera of cameras) {
      const matrix = transforms[camera.name];
      if (!matrix) continue;
      const color = camera.anchor ? 0x5bd8ff : 0xffc857;
      this.group.add(this._frustum(matrix, color));
    }

    const boards = calibration.world_from_board || {};
    for (const [cameraName, matrix] of Object.entries(boards)) {
      const detected = cameras.find(camera => camera.name === cameraName)?.detection?.detected;
      this.group.add(this._boardMarker(matrix, detected ? 0x73e58c : 0xff8a7a));
    }
  }

  setVisible(visible) {
    this.group.visible = !!visible;
  }

  _frustum(matrix, color) {
    const z = 0.16;
    const w = 0.075;
    const h = 0.055;
    const local = [
      [0, 0, 0],
      [-w, -h, z],
      [w, -h, z],
      [w, h, z],
      [-w, h, z],
    ].map(point => robotToThreeVector(transformPoint(matrix, point)));
    const [o, a, b, c, d] = local;
    return line([o, a, o, b, o, c, o, d, a, b, b, c, c, d, d, a], color);
  }

  _boardMarker(matrix, color) {
    const size = 0.06;
    const local = [
      [-size, -size, 0],
      [size, -size, 0],
      [size, size, 0],
      [-size, size, 0],
    ].map(point => robotToThreeVector(transformPoint(matrix, point)));
    const [a, b, c, d] = local;
    return line([a, b, b, c, c, d, d, a], color);
  }
}
