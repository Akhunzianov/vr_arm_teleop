import * as THREE from 'three';
import URDFLoader from 'urdf-loader';

class CameraFeedPlane {
  constructor(feed) {
    this.feed = feed;
    this.group = new THREE.Group();
    this._canvas = document.createElement('canvas');
    this._canvas.width = Math.max(1, Number(feed.width) || 640);
    this._canvas.height = Math.max(1, Number(feed.height) || 480);
    this._ctx = this._canvas.getContext('2d');
    this._texture = new THREE.CanvasTexture(this._canvas);
    this._texture.colorSpace = THREE.SRGBColorSpace;
    const aspect = this._canvas.width / this._canvas.height;
    const width = 0.16;
    const height = width / Math.max(0.1, aspect);
    const plane = new THREE.Mesh(
      new THREE.PlaneGeometry(width, height),
      new THREE.MeshBasicMaterial({
        map: this._texture,
        side: THREE.DoubleSide,
        toneMapped: false,
      }),
    );
    plane.position.z = 0.12;
    this.group.add(plane);
    this._drawPlaceholder();
    this._inFlight = false;
    this._timer = window.setInterval(() => this._refresh(), 120);
    this._refresh();
  }

  dispose() {
    window.clearInterval(this._timer);
    this._texture.dispose();
  }

  _drawPlaceholder() {
    this._ctx.fillStyle = '#111820';
    this._ctx.fillRect(0, 0, this._canvas.width, this._canvas.height);
    this._ctx.strokeStyle = '#3c586b';
    this._ctx.lineWidth = 8;
    this._ctx.strokeRect(4, 4, this._canvas.width - 8, this._canvas.height - 8);
    this._texture.needsUpdate = true;
  }

  async _refresh() {
    if (this._inFlight || !this.feed.url) return;
    this._inFlight = true;
    try {
      const joiner = this.feed.url.includes('?') ? '&' : '?';
      const res = await fetch(`${this.feed.url}${joiner}t=${Date.now()}`, {
        cache: 'no-store',
      });
      if (!res.ok) return;
      const blob = await res.blob();
      const bitmap = await createImageBitmap(blob);
      if (
        bitmap.width !== this._canvas.width
        || bitmap.height !== this._canvas.height
      ) {
        this._canvas.width = bitmap.width;
        this._canvas.height = bitmap.height;
      }
      this._ctx.drawImage(bitmap, 0, 0, this._canvas.width, this._canvas.height);
      bitmap.close();
      this._texture.needsUpdate = true;
    } catch (err) {
      console.warn('[robot_view] camera feed refresh failed', err);
    } finally {
      this._inFlight = false;
    }
  }
}

export class RobotView {
  constructor(world) {
    this.group = new THREE.Group();
    this.group.rotation.x = -Math.PI / 2;
    this.robot = null;
    this._feedViews = [];
    world.add(this.group);
  }

  load(model, onReady) {
    const loader = new URDFLoader();
    loader.workingPath = model.urdf_assets_url;
    loader.load(model.urdf_url, robot => {
      this._disposeFeeds();
      this.group.clear();
      this.robot = robot;
      robot.traverse(obj => {
        if (obj.isMesh && obj.material) {
          obj.material.side = THREE.DoubleSide;
        }
      });
      this.group.add(robot);
      // 2D camera feed planes intentionally disabled: the point cloud
      // from the wrist camera carries the same info in 3D and the plane
      // floating in front of the wrist was visually noisy.
      if (onReady) onReady(robot);
    });
  }

  findLink(name) {
    return this._findLink(name);
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

  _attachCameraFeeds(feeds) {
    for (const feed of feeds) {
      const link = this._findLink(feed.urdf_link);
      if (!link) {
        console.warn('[robot_view] URDF camera link not found', feed.urdf_link);
        continue;
      }
      const view = new CameraFeedPlane(feed);
      link.add(view.group);
      this._feedViews.push(view);
    }
  }

  _findLink(name) {
    if (!this.robot || !name) return null;
    if (this.robot.links && this.robot.links[name]) return this.robot.links[name];
    return this.robot.getObjectByName(name);
  }

  _disposeFeeds() {
    for (const view of this._feedViews) view.dispose();
    this._feedViews = [];
  }
}
