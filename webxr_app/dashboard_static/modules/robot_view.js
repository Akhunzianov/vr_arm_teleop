import * as THREE from 'three';
import URDFLoader from 'urdf-loader';

export class RobotView {
  constructor(world) {
    this.group = new THREE.Group();
    this.group.rotation.x = -Math.PI / 2;
    this.robot = null;
    world.add(this.group);
  }

  load(model) {
    const loader = new URDFLoader();
    loader.workingPath = model.urdf_assets_url;
    loader.load(model.urdf_url, robot => {
      this.group.clear();
      this.robot = robot;
      robot.traverse(obj => {
        if (obj.isMesh && obj.material) {
          obj.material.side = THREE.DoubleSide;
        }
      });
      this.group.add(robot);
    });
  }

  applyJoints(joints) {
    if (!this.robot || !joints) return;
    for (const [name, value] of Object.entries(joints)) {
      if (this.robot.joints && this.robot.joints[name]) {
        this.robot.setJointValue(name, value);
      }
    }
  }

  setVisible(visible) {
    this.group.visible = !!visible;
  }
}
