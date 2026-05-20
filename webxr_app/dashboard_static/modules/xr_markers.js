import * as THREE from 'three';
import { robotToThreeVector } from './dashboard_scene.js';

export function helmetToRobotPosition(position, origin) {
  const dx = position[0] - origin[0];
  const dy = position[1] - origin[1];
  const dz = position[2] - origin[2];
  return [dx, -dz, dy];
}

function makeMarker(color, radius) {
  const geom = new THREE.SphereGeometry(radius, 24, 16);
  const mat = new THREE.MeshStandardMaterial({
    color,
    roughness: 0.45,
    metalness: 0.0,
  });
  return new THREE.Mesh(geom, mat);
}

export class XRMarkers {
  constructor(world) {
    this.enabled = true;
    this._aligned = false;
    this.group = new THREE.Group();
    this.head = makeMarker(0x49a3ff, 0.045);
    this.wrist = makeMarker(0xffc857, 0.032);
    this.group.add(this.head);
    this.group.add(this.wrist);
    this.group.visible = false;
    world.add(this.group);
  }

  update(xr) {
    if (!xr || !xr.aligned || !xr.anchor || !xr.head || !xr.right_wrist) {
      this._aligned = false;
      this.group.visible = false;
      return;
    }
    const origin = xr.anchor.vr_position_of_robot_origin;
    const head = robotToThreeVector(helmetToRobotPosition(xr.head.position, origin));
    const wrist = robotToThreeVector(
      helmetToRobotPosition(xr.right_wrist.position, origin),
    );
    this.head.position.copy(head);
    this.wrist.position.copy(wrist);
    this._aligned = true;
    this.group.visible = this.enabled;
  }

  setVisible(visible) {
    this.enabled = !!visible;
    this.group.visible = this.enabled && this._aligned;
  }
}
