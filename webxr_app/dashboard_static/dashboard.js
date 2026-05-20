import { DashboardComms } from './modules/dashboard_comms.js';
import { DashboardScene } from './modules/dashboard_scene.js';
import { RobotView } from './modules/robot_view.js';
import { DashboardPointCloudView } from './modules/dashboard_pointcloud_view.js';
import { WorkspaceLayer } from './modules/workspace_layer.js';
import { XRMarkers } from './modules/xr_markers.js';
import { StatusPanel } from './modules/status_panel.js';

const scene = new DashboardScene(document.getElementById('viewport'));
const robot = new RobotView(scene.world);
const cloud = new DashboardPointCloudView(scene.world);
const workspace = new WorkspaceLayer(scene.world);
const xr = new XRMarkers(scene.world);
const status = new StatusPanel(
  document.getElementById('status'),
  document.getElementById('connection'),
);
const comms = new DashboardComms('/ws');

let modelLoaded = false;

comms.onConnectionState = state => status.setConnectionState(state);
comms.onJson = msg => {
  if (msg.type !== 'snapshot') return;
  if (!modelLoaded) {
    robot.load(msg.model);
    workspace.setBounds(msg.workspace.min, msg.workspace.max);
    modelLoaded = true;
  }
  robot.applyJoints(msg.robot.joints);
  xr.update(msg.xr);
  status.update(msg);
};
comms.onBinary = buf => cloud.ingest(buf);

function bindToggle(id, layer) {
  const el = document.getElementById(id);
  if (!el) return;
  layer.setVisible(el.checked);
  el.addEventListener('change', () => layer.setVisible(el.checked));
}

bindToggle('toggle-robot', robot);
bindToggle('toggle-cloud', cloud);
bindToggle('toggle-workspace', workspace);
bindToggle('toggle-xr', xr);
