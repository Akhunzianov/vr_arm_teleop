function escapeHtml(value) {
  return String(value)
    .replaceAll('&', '&amp;')
    .replaceAll('<', '&lt;')
    .replaceAll('>', '&gt;')
    .replaceAll('"', '&quot;')
    .replaceAll("'", '&#039;');
}

function fmtNumber(value, digits = 3) {
  if (value === null || value === undefined) return 'none';
  return Number(value).toFixed(digits);
}

function row(key, value, className = '') {
  const cls = className ? ` ${className}` : '';
  return `
    <div class="status-row">
      <div class="status-key">${escapeHtml(key)}</div>
      <div class="status-value${cls}">${escapeHtml(value)}</div>
    </div>
  `;
}

export class StatusPanel {
  constructor(statusEl, connectionEl) {
    this._status = statusEl;
    this._connection = connectionEl;
    this.setConnectionState('connecting');
  }

  setConnectionState(state) {
    this._connection.textContent = state.charAt(0).toUpperCase() + state.slice(1);
    this._connection.className = state;
  }

  update(snapshot) {
    const model = snapshot.model || {};
    const robot = snapshot.robot || {};
    const cloud = snapshot.pointcloud || {};
    const xr = snapshot.xr || {};
    const status = snapshot.status || {};
    const jointCount = robot.joints ? Object.keys(robot.joints).length : 0;
    const xrState = xr.aligned ? 'Aligned' : 'XR unaligned';

    const robotError = robot.error || 'none';
    const cloudError = cloud.error || 'none';
    this._status.innerHTML = `
      <div class="status-group">
        ${row('URDF', model.urdf_url || 'none')}
        ${row('Robot joints', jointCount)}
        ${row('Robot Hz', fmtNumber(status.robot_hz, 1))}
      </div>
      <div class="status-group">
        ${row('Cloud points', cloud.n_points || 0)}
        ${row('Cloud seq', cloud.sequence || 0)}
        ${row('Cloud Hz', fmtNumber(status.pointcloud_hz, 1))}
      </div>
      <div class="status-group">
        ${row('XR', xrState, xr.aligned ? '' : 'warn')}
        ${row('Head pose', xr.head ? 'present' : 'none')}
        ${row('Right wrist', xr.right_wrist ? 'present' : 'none')}
      </div>
      <div class="status-group">
        ${row('Robot error', robotError, robot.error ? 'error' : '')}
        ${row('Cloud error', cloudError, cloud.error ? 'error' : '')}
      </div>
    `;
  }
}
