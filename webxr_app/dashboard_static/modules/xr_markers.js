import * as THREE from 'three';
import { robotToThreeVector } from './dashboard_scene.js';
import { HeadView } from './head_view.js';
import { DashboardHandView } from './dashboard_hand_view.js';

export function helmetToRobotPosition(position, origin) {
  const dx = position[0] - origin[0];
  const dy = position[1] - origin[1];
  const dz = position[2] - origin[2];
  return [dx, -dz, dy];
}

// helmet->robot->three composes to identity on basis vectors (see
// dashboard_scene.robotToThreeVector + helmetToRobotPosition above), so the
// raw helmet-frame WebXR orientation quaternion is the correct rotation in
// three world space too. No quat conversion needed.

export class XRMarkers {
  constructor(world) {
    this.enabled = true;
    this._aligned = false;
    this.group = new THREE.Group();
    this.group.visible = false;
    world.add(this.group);
    this.head = new HeadView(this.group);
    this.hand = new DashboardHandView(this.group);
    this.head.setVisible(true);
    this.hand.setVisible(true);
  }

  update(xr) {
    if (!xr || !xr.aligned || !xr.anchor || !xr.head || !xr.right_wrist) {
      this._aligned = false;
      this.group.visible = false;
      return;
    }
    const origin = xr.anchor.vr_position_of_robot_origin;
    const headPos = robotToThreeVector(helmetToRobotPosition(xr.head.position, origin));
    const wristPos = robotToThreeVector(
      helmetToRobotPosition(xr.right_wrist.position, origin),
    );
    this.head.setPose(headPos, xr.head.orientation);
    this.hand.setPose(wristPos, xr.right_wrist.orientation);
    if (xr.right_wrist.curls) {
      this.hand.driveCurls(xr.right_wrist.curls);
    }
    this._aligned = true;
    this.group.visible = this.enabled;
  }

  setVisible(visible) {
    this.enabled = !!visible;
    this.group.visible = this.enabled && this._aligned;
  }
}
