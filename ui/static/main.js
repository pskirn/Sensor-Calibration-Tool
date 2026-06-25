// Orchestration: talks to the FastAPI backend, drives the viewer + chart.

import { PlaneViewer } from '/viewer.js';

const $ = (id) => document.getElementById(id);
let chart = null;
let viewer = null;
let currentPoses = null;     // last loaded input data
let currentDataset = null;   // dataset name
let lastResult = null;       // last calibration result

// --- generic helpers ---

function setStatus(msg, kind) {
  const el = $('status');
  el.textContent = msg;
  el.className = 'status ' + kind;
}
function fmtNum(n, d = 4) {
  if (n === null || n === undefined || Number.isNaN(n)) return '—';
  return Number(n).toFixed(d);
}
function fmtVec(v, d = 4) {
  if (!v) return '—';
  return '[' + v.map((x) => fmtNum(x, d)).join(', ') + ']';
}

// --- dataset list / selection ---

async function refreshDatasets() {
  const sel = $('dataset-select');
  const prev = sel.value;
  const resp = await fetch('/api/datasets');
  const data = await resp.json();
  sel.innerHTML = '';
  for (const d of data.datasets) {
    const opt = document.createElement('option');
    opt.value = d.name;
    opt.textContent = `${d.name}  (${d.num_poses} poses)`;
    opt.dataset.numPoses = d.num_poses;
    sel.appendChild(opt);
  }
  if (data.datasets.length === 0) {
    const opt = document.createElement('option');
    opt.textContent = '— no datasets found in data/ —';
    opt.disabled = true;
    sel.appendChild(opt);
    return;
  }
  if (prev && [...sel.options].some((o) => o.value === prev)) {
    sel.value = prev;
  }
  await onDatasetChange();
}

async function onDatasetChange() {
  const sel = $('dataset-select');
  if (!sel.value) return;
  currentDataset = sel.value;
  const numPoses = sel.selectedOptions[0]?.dataset.numPoses;
  $('dataset-info').textContent = numPoses ? `${numPoses} pose pairs in ${currentDataset}/poses.csv` : '';
  await loadInputs();
}

async function loadInputs() {
  if (!currentDataset) return;
  const resp = await fetch(`/api/poses?dataset=${encodeURIComponent(currentDataset)}`);
  if (!resp.ok) {
    setStatus('Failed to load poses', 'err');
    return;
  }
  const data = await resp.json();
  currentPoses = data.poses;
  viewer.renderInputs(currentPoses);
  // After switching datasets, the prior calibration no longer applies.
  $('after-hint').textContent = '— run calibration first';
  document.querySelector('input[name="view-mode"][value="after"]').disabled = true;
}

// --- upload ---

function setupDropzone() {
  const dz = $('dropzone');
  const fi = $('file-input');
  dz.addEventListener('click', () => fi.click());
  fi.addEventListener('change', () => fi.files[0] && uploadFile(fi.files[0]));
  ['dragenter', 'dragover'].forEach((ev) =>
    dz.addEventListener(ev, (e) => { e.preventDefault(); dz.classList.add('drag'); }));
  ['dragleave', 'drop'].forEach((ev) =>
    dz.addEventListener(ev, (e) => { e.preventDefault(); dz.classList.remove('drag'); }));
  dz.addEventListener('drop', (e) => {
    const f = e.dataTransfer.files[0];
    if (f) uploadFile(f);
  });
}

async function uploadFile(file) {
  const msg = $('upload-msg');
  msg.textContent = `Uploading ${file.name}…`;
  const fd = new FormData();
  fd.append('file', file);
  const resp = await fetch('/api/upload', { method: 'POST', body: fd });
  const body = await resp.json();
  if (!resp.ok) {
    msg.textContent = 'Upload failed: ' + (body.detail || 'unknown');
    return;
  }
  msg.textContent = `Saved as ${body.name} (${body.num_poses} poses).`;
  await refreshDatasets();
  $('dataset-select').value = body.name;
  await onDatasetChange();
}

// --- calibration ---

function renderConvention(block) {
  if (!block) return '(missing)';
  const lines = [];
  lines.push('translation_m   : ' + fmtVec(block.translation_m));
  lines.push('rotation_rpy_rad: ' + fmtVec(block.rotation_rpy_rad));
  lines.push('quaternion_xyzw : ' + fmtVec(block.quaternion_xyzw));
  if (block.rotation_matrix) {
    lines.push('rotation_matrix :');
    for (const row of block.rotation_matrix) lines.push('  ' + fmtVec(row));
  }
  return lines.join('\n');
}

function renderResult(res) {
  lastResult = res;
  $('s-poses').textContent = res.poses_used ?? '—';
  $('s-cost').textContent  = fmtNum(res.final_cost, 6);
  $('s-mean').textContent  = fmtNum(res.mean_residual_m, 6) + ' m';
  $('s-max').textContent   = fmtNum(res.max_residual_m, 6) + ' m';

  $('conv-l2c').textContent = renderConvention(res.lidar_to_camera);
  $('conv-c2l').textContent = renderConvention(res.camera_to_lidar);

  $('summary').classList.remove('hidden');
  $('conventions').classList.remove('hidden');
  $('chart-card').classList.remove('hidden');

  drawChart(res.per_pose_residuals_m || []);

  // Enable "after" view and render the transformed lidar planes.
  if (currentPoses && res.lidar_to_camera) {
    viewer.renderAligned(currentPoses, {
      R: res.lidar_to_camera.rotation_matrix,
      t: res.lidar_to_camera.translation_m,
    });
    document.querySelector('input[name="view-mode"][value="after"]').disabled = false;
    $('after-hint').textContent = '';
  }
}

function drawChart(residuals) {
  const ctx = $('residual-chart').getContext('2d');
  const labels = residuals.map((_, i) => String(i));
  if (chart) chart.destroy();
  chart = new Chart(ctx, {
    type: 'bar',
    data: { labels, datasets: [{ label: 'Residual (m)', data: residuals, backgroundColor: '#4fb3ff' }] },
    options: {
      responsive: true,
      plugins: { legend: { display: false } },
      scales: {
        x: { ticks: { color: '#8a96a4' }, grid: { color: '#2a3340' } },
        y: { ticks: { color: '#8a96a4' }, grid: { color: '#2a3340' },
             title: { display: true, text: 'metres', color: '#8a96a4' } },
      },
    },
  });
}

async function runCalibration() {
  if (!currentDataset) return;
  const btn = $('run-btn');
  btn.disabled = true;
  setStatus('Running…', 'running');
  $('logs-card').classList.add('hidden');
  try {
    const resp = await fetch('/api/calibrate', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ dataset: currentDataset }),
    });
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

function setupViewMode() {
  document.querySelectorAll('input[name="view-mode"]').forEach((r) => {
    r.addEventListener('change', () => viewer.setMode(r.value));
  });
}

// --- init ---

window.addEventListener('DOMContentLoaded', async () => {
  viewer = new PlaneViewer($('viewer-container'));
  setupDropzone();
  setupViewMode();
  $('dataset-select').addEventListener('change', onDatasetChange);
  $('refresh-datasets').addEventListener('click', refreshDatasets);
  $('run-btn').addEventListener('click', runCalibration);
  await refreshDatasets();
});
