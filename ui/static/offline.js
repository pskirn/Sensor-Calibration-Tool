// Offline mode: pick or upload a recorded poses.csv, inspect it, solve it.

import { $, api, postJSON, setStatus, toast } from '/app.js';
import { renderResult } from '/results.js';
import { PlaneViewer } from '/viewer.js';

let viewer = null;
let chart = null;
let currentPoses = null;
let currentDataset = null;

async function refreshDatasets() {
  const sel = $('dataset-select');
  const previous = sel.value;
  const { datasets } = await api('/api/datasets');

  sel.innerHTML = '';
  if (!datasets.length) {
    const opt = document.createElement('option');
    opt.textContent = '— no datasets with a poses.csv under data/ —';
    opt.disabled = true;
    sel.appendChild(opt);
    $('dataset-info').textContent = '';
    return;
  }
  for (const d of datasets) {
    const opt = document.createElement('option');
    opt.value = d.name;
    opt.textContent = `${d.name}  (${d.num_poses} poses)`;
    opt.dataset.numPoses = d.num_poses;
    sel.appendChild(opt);
  }
  if (previous && [...sel.options].some((o) => o.value === previous)) sel.value = previous;
  await onDatasetChange();
}

async function onDatasetChange() {
  const sel = $('dataset-select');
  if (!sel.value) return;
  currentDataset = sel.value;
  const n = sel.selectedOptions[0]?.dataset.numPoses;
  $('dataset-info').textContent = n ? `${n} pose pairs in ${currentDataset}/poses.csv` : '';
  await loadInputs();
}

async function loadInputs() {
  if (!currentDataset) return;
  try {
    const data = await api(`/api/poses?dataset=${encodeURIComponent(currentDataset)}`);
    currentPoses = data.poses;
    viewer.renderInputs(currentPoses);
    // A previous solve does not apply to a newly selected dataset.
    $('after-hint').textContent = '— solve first';
    document.querySelector('input[name="view-mode"][value="after"]').disabled = true;
    document.querySelector('input[name="view-mode"][value="before"]').checked = true;
    viewer.setMode('before');
  } catch (e) {
    toast('Could not load poses: ' + e.message, 'err');
  }
}

function setupDropzone() {
  const dz = $('dropzone');
  const input = $('file-input');
  dz.addEventListener('click', () => input.click());
  input.addEventListener('change', () => input.files[0] && upload(input.files[0]));
  ['dragenter', 'dragover'].forEach((ev) =>
    dz.addEventListener(ev, (e) => { e.preventDefault(); dz.classList.add('drag'); }));
  ['dragleave', 'drop'].forEach((ev) =>
    dz.addEventListener(ev, (e) => { e.preventDefault(); dz.classList.remove('drag'); }));
  dz.addEventListener('drop', (e) => e.dataTransfer.files[0] && upload(e.dataTransfer.files[0]));
}

async function upload(file) {
  const msg = $('upload-msg');
  msg.textContent = `Uploading ${file.name}…`;
  const form = new FormData();
  form.append('file', file);
  try {
    const body = await api('/api/upload', { method: 'POST', body: form });
    msg.textContent = `Saved as ${body.name} (${body.num_poses} poses).`;
    await refreshDatasets();
    $('dataset-select').value = body.name;
    await onDatasetChange();
    toast(`Loaded ${body.num_poses} poses from ${file.name}`, 'ok');
  } catch (e) {
    msg.textContent = 'Upload failed: ' + e.message;
    toast('Upload failed: ' + e.message, 'err');
  }
}

async function runCalibration() {
  if (!currentDataset) return;
  const btn = $('run-btn');
  btn.disabled = true;
  setStatus($('status'), 'Running…', 'running');
  try {
    const body = await postJSON('/api/calibrate', { dataset: currentDataset });
    setStatus($('status'), 'Done', 'ok');
    chart = renderResult($('offline-results'), body.result, body.stdout, chart);

    if (currentPoses && body.result.lidar_to_camera) {
      viewer.renderAligned(currentPoses, {
        R: body.result.lidar_to_camera.rotation_matrix,
        t: body.result.lidar_to_camera.translation_m,
      });
      document.querySelector('input[name="view-mode"][value="after"]').disabled = false;
      $('after-hint').textContent = '';
    }
    toast(`Solved ${body.result.poses_used} poses · mean residual ` +
          `${(body.result.mean_residual_m * 1000).toFixed(2)} mm`, 'ok');
  } catch (e) {
    setStatus($('status'), 'Failed', 'err');
    const detail = e.detail;
    $('offline-results').innerHTML =
      `<pre>${(detail?.stderr || detail?.stdout || e.message)}</pre>`;
    toast('Calibration failed: ' + e.message, 'err');
  } finally {
    btn.disabled = false;
  }
}

export function initOffline() {
  viewer = new PlaneViewer($('viewer-container'));
  setupDropzone();
  document.querySelectorAll('input[name="view-mode"]').forEach((radio) =>
    radio.addEventListener('change', () => viewer.setMode(radio.value)));
  $('dataset-select').addEventListener('change', onDatasetChange);
  $('refresh-datasets').addEventListener('click', refreshDatasets);
  $('run-btn').addEventListener('click', runCalibration);
  refreshDatasets();
}
