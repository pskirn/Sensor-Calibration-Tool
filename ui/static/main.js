// Tiny vanilla-JS frontend. Talks to /api/calibrate, renders the YAML.

const $ = (id) => document.getElementById(id);
let chart = null;

function setStatus(msg, kind) {
  const el = $('status');
  el.textContent = msg;
  el.className = 'status ' + kind;
}

function fmtNum(n, digits = 4) {
  if (n === null || n === undefined || Number.isNaN(n)) return '—';
  return Number(n).toFixed(digits);
}

function fmtVec(v, digits = 4) {
  if (!v) return '—';
  return '[' + v.map((x) => fmtNum(x, digits)).join(', ') + ']';
}

function renderConvention(block) {
  if (!block) return '(missing)';
  const lines = [];
  lines.push('translation_m   : ' + fmtVec(block.translation_m));
  lines.push('rotation_rpy_rad: ' + fmtVec(block.rotation_rpy_rad));
  lines.push('quaternion_xyzw : ' + fmtVec(block.quaternion_xyzw));
  if (block.rotation_matrix) {
    lines.push('rotation_matrix :');
    for (const row of block.rotation_matrix) {
      lines.push('  ' + fmtVec(row));
    }
  }
  return lines.join('\n');
}

function renderResult(res) {
  $('s-poses').textContent = res.poses_used ?? '—';
  $('s-cost').textContent  = fmtNum(res.final_cost, 6);
  $('s-mean').textContent  = fmtNum(res.mean_residual_m, 6) + ' m';
  $('s-max').textContent   = fmtNum(res.max_residual_m, 6) + ' m';

  $('conv-l2c').textContent = renderConvention(res.lidar_to_camera);
  $('conv-c2l').textContent = renderConvention(res.camera_to_lidar);

  $('summary').classList.remove('hidden');
  $('conventions').classList.remove('hidden');
  $('chart-card').classList.remove('hidden');

  const residuals = res.per_pose_residuals_m || [];
  drawChart(residuals);
}

function drawChart(residuals) {
  const ctx = $('residual-chart').getContext('2d');
  const labels = residuals.map((_, i) => String(i));
  const data = {
    labels,
    datasets: [{
      label: 'Residual (m)',
      data: residuals,
      backgroundColor: '#4fb3ff',
    }],
  };
  const opts = {
    responsive: true,
    plugins: { legend: { display: false } },
    scales: {
      x: { ticks: { color: '#8a96a4' }, grid: { color: '#2a3340' } },
      y: { ticks: { color: '#8a96a4' }, grid: { color: '#2a3340' },
           title: { display: true, text: 'metres', color: '#8a96a4' } },
    },
  };
  if (chart) chart.destroy();
  chart = new Chart(ctx, { type: 'bar', data, options: opts });
}

async function runCalibration() {
  const btn = $('run-btn');
  btn.disabled = true;
  setStatus('Running…', 'running');
  $('logs-card').classList.add('hidden');

  try {
    const resp = await fetch('/api/calibrate', { method: 'POST' });
    const body = await resp.json();
    if (!resp.ok) {
      setStatus('Failed', 'err');
      $('logs').textContent =
        (body.detail && body.detail.stderr) || JSON.stringify(body, null, 2);
      $('logs-card').classList.remove('hidden');
      return;
    }
    setStatus('Done', 'ok');
    renderResult(body.result);
    $('logs').textContent = body.stdout;
    $('logs-card').classList.remove('hidden');
  } catch (e) {
    setStatus('Error', 'err');
    $('logs').textContent = String(e);
    $('logs-card').classList.remove('hidden');
  } finally {
    btn.disabled = false;
  }
}

async function loadExistingResult() {
  try {
    const resp = await fetch('/api/result');
    if (!resp.ok) return;
    const res = await resp.json();
    renderResult(res);
    setStatus('Loaded cached result', 'ok');
  } catch (_) { /* nothing yet, that's fine */ }
}

$('run-btn').addEventListener('click', runCalibration);
loadExistingResult();
