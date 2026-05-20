// Wireframe box that draws the allowed workspace in VR.
//
// The bounds are in *robot-world* coordinates. To put the box in the
// right spot in the headset, we need the mapping from robot-world to
// the VR play_space frame -- that mapping is only known after the
// operator engages tracking (server sends an `anchor` message with
// the position the robot-world origin maps to in VR). Until then the
// box stays hidden, so we don't draw it at the wrong place.
//
// setBounds([x,y,z], [x,y,z]) re-shapes the box in robot-world coords.
// setOrigin([x,y,z])          places robot-world origin in VR space.
// setHighlight(true) flips the color to red for safety alerts.

import * as THREE from 'three';

const COLOR_NORMAL = 0x66ccff;
const COLOR_ALERT = 0xff4444;

export class WorkspaceView {
  constructor(scene) {
    const geom = new THREE.BoxGeometry(1, 1, 1);
    const edges = new THREE.EdgesGeometry(geom);
    geom.dispose();

    const mat = new THREE.LineBasicMaterial({
      color: COLOR_NORMAL, transparent: true, opacity: 0.6,
    });
    this._lines = new THREE.LineSegments(edges, mat);
    this._lines.visible = false;
    scene.add(this._lines);

    this._center = new THREE.Vector3();  // box center in helmet coords (rel. origin)
    this._half = new THREE.Vector3();    // box half-extents in helmet coords
    this._origin = null;                 // robot-world origin in VR coords
    this._haveBounds = false;
  }

  // True iff `point` (VR/helmet world coords) lies inside the workspace box.
  // Returns null when we don't yet have both bounds and the anchor.
  containsPoint(point) {
    if (!this._haveBounds || this._origin === null || !point) return null;
    const dx = Math.abs(point[0] - (this._origin.x + this._center.x));
    const dy = Math.abs(point[1] - (this._origin.y + this._center.y));
    const dz = Math.abs(point[2] - (this._origin.z + this._center.z));
    return dx <= this._half.x && dy <= this._half.y && dz <= this._half.z;
  }

  setBounds(min, max) {
    // Bounds arrive in robot frame (x=right, y=forward, z=up). The
    // scene renders in helmet frame (x=right, y=up, z=back). The
    // change of basis robot -> helmet maps (x,y,z) -> (x, z, -y),
    // which keeps the box axis-aligned in helmet space, just with
    // its dimensions swapped/flipped.
    const [minXr, minYr, minZr] = min;
    const [maxXr, maxYr, maxZr] = max;
    const minXh = minXr,        maxXh = maxXr;
    const minYh = minZr,        maxYh = maxZr;       // helmet Y  <- robot Z
    const minZh = -maxYr,       maxZh = -minYr;      // helmet Z  <- -robot Y
    this._center.set(
      (minXh + maxXh) * 0.5,
      (minYh + maxYh) * 0.5,
      (minZh + maxZh) * 0.5,
    );
    this._half.set(
      (maxXh - minXh) * 0.5,
      (maxYh - minYh) * 0.5,
      (maxZh - minZh) * 0.5,
    );
    this._lines.scale.set(maxXh - minXh, maxYh - minYh, maxZh - minZh);
    this._haveBounds = true;
    this._refresh();
  }

  setOrigin(vrPositionOfRobotOrigin) {
    const [x, y, z] = vrPositionOfRobotOrigin;
    this._origin = new THREE.Vector3(x, y, z);
    this._refresh();
  }

  setHighlight(active) {
    this._lines.material.color.setHex(active ? COLOR_ALERT : COLOR_NORMAL);
  }

  _refresh() {
    if (!this._haveBounds || this._origin === null) {
      this._lines.visible = false;
      return;
    }
    this._lines.position.set(
      this._origin.x + this._center.x,
      this._origin.y + this._center.y,
      this._origin.z + this._center.z,
    );
    this._lines.visible = true;
  }
}
