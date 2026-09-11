// Shared rendering of a calibration result.
//
// Live and offline solves run the same C++ binary and produce the same YAML, so
// they share one renderer — keeping the two modes from drifting apart in how
// they present the same numbers.

import { fmtNum, fmtVec } from '/app.js';

function renderConvention(block) {
  if (!block) return '(missing)';
  const lines = [
    'translation_m   : ' + fmtVec(block.translation_m),
    'rotation_rpy_rad: ' + fmtVec(block.rotation_rpy_rad),
    'quaternion_xyzw : ' + fmtVec(block.quaternion_xyzw),
  ];
  if (block.rotation_matrix) {
    lines.push('rotation_matrix :');
    for (const row of block.rotation_matrix) lines.push('  ' + fmtVec(row));
  }
  return lines.join('\n');
}

function drawChart(canvas, residuals, existing) {
  if (existing) existing.destroy();
  return new Chart(canvas.getContext('2d'), {
    type: 'bar',
    data: {
      labels: residuals.map((_, i) => String(i)),
      datasets: [{ label: 'Residual (m)', data: residuals, backgroundColor: '#4fb3ff' }],
    },
    options: {
      responsive: true,
      plugins: { legend: { display: false } },
      scales: {
        x: { ticks: { color: '#6f7d8d' }, grid: { color: '#232d3a' } },
        y: {
          ticks: { color: '#6f7d8d' }, grid: { color: '#232d3a' },
          title: { display: true, text: 'metres', color: '#6f7d8d' },
        },
      },
    },
  });
}

/**
 * Render a result into `host`, replacing anything already there.
 * Returns the Chart instance so the caller can dispose of it later.
 */
export function renderResult(host, result, logs, previousChart) {
  const node = document.getElementById('results-template').content.cloneNode(true);
  const f = (name) => node.querySelector(`[data-f="${name}"]`);

  f('poses').textContent = result.poses_used ?? '—';
  f('cost').textContent = fmtNum(result.final_cost, 6);
  f('mean').textContent = fmtNum(result.mean_residual_m, 6) + ' m';
  f('max').textContent = fmtNum(result.max_residual_m, 6) + ' m';
  f('l2c').textContent = renderConvention(result.lidar_to_camera);
  f('c2l').textContent = renderConvention(result.camera_to_lidar);
  f('logs').textContent = logs || '(no output)';

  const canvas = f('chart');
  host.innerHTML = '';
  host.appendChild(node);

  return drawChart(canvas, result.per_pose_residuals_m || [], previousChart);
}
