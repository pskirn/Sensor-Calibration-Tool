// Live mode: pick sensors, stream them, capture poses with feedback, solve.

import { $, api, postJSON, setStatus, toast } from '/app.js';
import { renderResult } from '/results.js';
import { CloudViewer } from '/viewer.js';

const POLL_MS = 500;          // status polling; the MJPEG stream carries the video
const CLOUD_MS = 1000;        // the 3D cloud is heavier, so refresh it less often
const RECOMMENDED_POSES = 9;
const MIN_SPAN = 0.15;

let cloudViewer = null;
let chart = null;
let statusTimer = null;
let cloudTimer = null;
let running = false;

// --- setup form -----------------------------------------------------------

async function refreshSources() {
  const data = await api('/api/live/sources');
  const fill = (select, items) => {
    const previous = select.value;
    select.innerHTML = '';
    if (!items.length) {
      const opt = document.createElement('option');
      opt.textContent = '— none found —';
      opt.disabled = true;
      select.appendChild(opt);
      return;
    }
    for (const s of items) {
      const opt = document.createElement('option');
      opt.value = s.id;
      opt.textContent = `${s.label}  ·  ${s.backend}`;
      opt.dataset.detail = s.detail || '';
      select.appendChild(opt);
    }
    if (previous && [...select.options].some((o) => o.value === previous)) {
      select.value = previous;
    }
  };

  fill($('camera-source'), data.camera);
  fill($('lidar-source'), data.lidar);
  updateHints();

  $('sim-callout').classList.toggle(
    'hidden', data.camera.length > 1 && data.lidar.length > 1 && data.ros2_available);

  for (const note of data.notes || []) {
    if (note.includes('rclpy')) continue;   // shown in the sidebar already
    toast(note, 'warn', 8000);
  }
  await loadDefaults();
}

function updateHints() {
  const hint = (selectId, hintId) => {
    const sel = $(selectId);
    $(hintId).textContent = sel.selectedOptions[0]?.dataset.detail || '';
  };
  hint('camera-source', 'camera-hint');
  hint('lidar-source', 'lidar-hint');
}

async function loadDefaults() {
  const camera = $('camera-source').value || '';
  const lidar = $('lidar-source').value || '';
  if (!camera && !lidar) return;
  try {
    const d = await api(
      `/api/live/defaults?camera_source=${encodeURIComponent(camera)}` +
      `&lidar_source=${encodeURIComponent(lidar)}`);

    $('board-cols').value = d.board.cols;
    $('board-rows').value = d.board.rows;
    $('board-square').value = d.board.square_size;
    $('accum-window').value = d.accumulation_window ?? 1.0;
    const [xn, yn, zn] = d.roi_min;
    const [xx, yx, zx] = d.roi_max;
    $('roi-xmin').value = xn; $('roi-ymin').value = yn; $('roi-zmin').value = zn;
    $('roi-xmax').value = xx; $('roi-ymax').value = yx; $('roi-zmax').value = zx;

    $('setup-note').textContent = d.note || '';
    $('intrinsics-note').textContent = d.intrinsics_source
      ? `Intrinsics: ${d.intrinsics_source}` : '';
    // Remember the board panel size the defaults implied, so the simulated rig's
    // exact panel is not lost when the form only exposes corner counts.
    $('board-square').dataset.width = d.board.board_width ?? '';
    $('board-square').dataset.height = d.board.board_height ?? '';
  } catch (e) {
    $('setup-note').textContent = 'Could not load defaults: ' + e.message;
  }
}

function readForm() {
  const num = (id) => Number($(id).value);
  const width = $('board-square').dataset.width;
  const height = $('board-square').dataset.height;
  return {
    camera_source: $('camera-source').value,
    lidar_source: $('lidar-source').value,
    board: {
      cols: num('board-cols'),
      rows: num('board-rows'),
      square_size: num('board-square'),
      board_width: width ? Number(width) : null,
      board_height: height ? Number(height) : null,
    },
    roi_min: [num('roi-xmin'), num('roi-ymin'), num('roi-zmin')],
    roi_max: [num('roi-xmax'), num('roi-ymax'), num('roi-zmax')],
    accumulation_window: num('accum-window'),
  };
}

// --- network diagnostics --------------------------------------------------

async function diagnose() {
  const btn = $('diagnose-btn');
  const host = $('diagnostics');
  btn.disabled = true;
  btn.textContent = 'Scanning the wire…';
  host.classList.remove('hidden');
  host.innerHTML = '<p class="muted small">Listening on known LiDAR ports…</p>';

  try {
    const report = await api('/api/live/diagnose?seconds=1.5');
    host.innerHTML = '';

    for (const finding of report.findings) {
      const card = document.createElement('div');
      card.className = 'finding ' + finding.level;

      const title = document.createElement('div');
      title.className = 'finding-title';
      title.textContent = finding.title;
      card.appendChild(title);

      if (finding.detail) {
        const detail = document.createElement('div');
        detail.className = 'finding-detail';
        detail.textContent = finding.detail;
        card.appendChild(detail);
      }

      if (finding.action) {
        const wrap = document.createElement('div');
        wrap.className = 'finding-action';
        const pre = document.createElement('pre');
        pre.textContent = finding.action;
        const copy = document.createElement('button');
        copy.className = 'copy-btn';
        copy.textContent = 'copy';
        copy.addEventListener('click', async () => {
          try {
            await navigator.clipboard.writeText(finding.action);
            copy.textContent = 'copied';
            setTimeout(() => { copy.textContent = 'copy'; }, 1500);
          } catch { toast('Clipboard unavailable — select the text manually', 'warn'); }
        });
        wrap.append(pre, copy);
        card.appendChild(wrap);
      }
      host.appendChild(card);
    }

    if (report.interfaces.length) {
      const table = document.createElement('div');
      table.className = 'iface-table';
      table.textContent = 'Wired interfaces: ' + report.interfaces
        .map((i) => `${i.name} [${i.state}]${i.addresses.length ? ' ' + i.addresses.join(',') : ''}`)
        .join('   ');
      host.appendChild(table);
    }

    // A driver may have been started since the page loaded.
    await refreshSources();
  } catch (e) {
    host.innerHTML = '';
    toast('Network scan failed: ' + e.message, 'err');
  } finally {
    btn.disabled = false;
    btn.textContent = 'LiDAR not listed? Scan the network';
  }
}

// --- session lifecycle ----------------------------------------------------

const SETUP_FIELDS = [
  'camera-source', 'lidar-source', 'board-cols', 'board-rows', 'board-square',
  'accum-window', 'roi-xmin', 'roi-xmax', 'roi-ymin', 'roi-ymax',
  'roi-zmin', 'roi-zmax', 'start-live', 'refresh-sources', 'diagnose-btn',
];

/** Freeze the setup form while a session owns the devices. */
function setSetupEnabled(enabled) {
  for (const id of SETUP_FIELDS) {
    const el = $(id);
    if (el) el.disabled = !enabled;
  }
}

async function start() {
  const payload = readForm();
  if (!payload.camera_source || !payload.lidar_source) {
    toast('Pick both a camera and a LiDAR source', 'warn');
    return;
  }
  const btn = $('start-live');
  btn.disabled = true;
  setStatus($('live-status'), 'Starting…', 'running');
  try {
    const res = await postJSON('/api/live/start', payload);
    running = true;
    setStatus($('live-status'), 'Streaming', 'ok');
    $('live-capture').classList.remove('hidden');
    $('stop-live').classList.remove('hidden');
    $('intrinsics-note').textContent = `Intrinsics: ${res.intrinsics_source}`;
    // Only one session may exist, so a second start would just 409. Lock the
    // controls that define the session instead of letting it fail.
    setSetupEnabled(false);

    // Cache-busting query keeps a stopped-and-restarted stream from being
    // served the old, now-dead connection.
    $('preview').src = `/api/live/preview.mjpg?t=${Date.now()}`;
    cloudViewer.resetView();

    statusTimer = setInterval(pollState, POLL_MS);
    cloudTimer = setInterval(pollCloud, CLOUD_MS);
    pollState();
    pollCloud();
  } catch (e) {
    setStatus($('live-status'), 'Failed', 'err');
    toast('Could not start: ' + e.message, 'err', 9000);
    setSetupEnabled(true);
  } finally {
    if (!running) btn.disabled = false;
  }
}

async function stop() {
  clearInterval(statusTimer);
  clearInterval(cloudTimer);
  running = false;
  // Dropping the src closes the MJPEG connection; without this the generator
  // keeps producing frames for a stream nobody is watching.
  $('preview').removeAttribute('src');
  try { await postJSON('/api/live/stop', {}); } catch { /* already gone */ }
  setStatus($('live-status'), 'Stopped', 'idle');
  $('stop-live').classList.add('hidden');
  setSetupEnabled(true);
}

// --- polling --------------------------------------------------------------

async function pollState() {
  if (!running) return;
  try {
    const s = await api('/api/live/state');
    renderLive(s.live);
    renderReadiness(s.readiness);
    renderPoses(s.poses);
  } catch (e) {
    if (e.status === 409) await stop();
  }
}

async function pollCloud() {
  if (!running) return;
  try {
    cloudViewer.render(await api('/api/live/cloud'));
  } catch { /* transient; the next tick will retry */ }
}

function badge(el, ok, okText, badText) {
  el.textContent = ok ? okText : badText;
  el.className = 'badge ' + (ok ? 'ok' : 'err');
}

function renderLive(live) {
  badge($('cam-badge'), live.camera_ok,
        `board found · ${live.reprojection_error.toFixed(2)} px`, 'no board');
  badge($('lid-badge'), live.lidar_ok,
        `plane found · ${live.lidar_points} pts`, 'no plane');

  const guidance = $('guidance');
  guidance.innerHTML = '';

  const add = (kind, icon, text) => {
    const div = document.createElement('div');
    div.className = 'guide ' + kind;
    div.innerHTML = `<span class="guide-icon">${icon}</span><span></span>`;
    div.lastChild.textContent = text;
    guidance.appendChild(div);
  };

  for (const reason of live.reasons) add('block', '!', reason);
  for (const hint of live.hints) add('hint', '~', hint);
  if (live.can_capture && !live.reasons.length && !live.hints.length) {
    add('good', '✓',
        `Both sensors see the board — ${live.novelty_deg >= 180
          ? 'ready to capture' : `${live.novelty_deg.toFixed(0)}° from your nearest pose`}`);
  }

  $('capture-btn').disabled = !live.can_capture;
}

function meter(barId, valueId, fraction, text, good) {
  $(valueId).textContent = text;
  const bar = $(barId);
  bar.style.width = `${Math.min(100, Math.max(0, fraction * 100))}%`;
  bar.className = 'fill ' + (good ? 'ok' : fraction > 0.4 ? 'warn' : '');
}

function renderReadiness(r) {
  $('capture-count').textContent = `${r.num_poses} captured`;
  meter('b-poses', 'm-poses', r.num_poses / RECOMMENDED_POSES,
        String(r.num_poses), r.num_poses >= RECOMMENDED_POSES);
  // Scaled against 0.5, which is a comfortably well-conditioned span in practice.
  meter('b-span', 'm-span', r.normal_span / 0.5,
        r.normal_span.toFixed(2), r.normal_span >= 0.3);
  meter('b-cov', 'm-cov', r.coverage,
        `${Math.round(r.coverage * 100)}%`, r.coverage >= 0.5);

  $('finish-btn').disabled = !r.ready;
  $('finish-hint').textContent = r.ready
    ? (r.advice[0] || 'Ready to solve.')
    : (r.blocking[0] || '');
}

function renderPoses(poses) {
  const strip = $('pose-strip');
  strip.innerHTML = '';
  for (const p of poses) {
    const chip = document.createElement('div');
    chip.className = 'pose-chip';

    const idx = document.createElement('span');
    idx.className = 'idx';
    idx.textContent = `#${p.index + 1}`;

    const meta = document.createElement('span');
    meta.className = 'meta';
    meta.textContent = `${p.cam_distance.toFixed(2)}m · ${p.reprojection_error.toFixed(2)}px`;

    const del = document.createElement('button');
    del.className = 'btn-tiny';
    del.textContent = '×';
    del.title = 'Discard this pose';
    del.addEventListener('click', async () => {
      try {
        await api(`/api/live/pose/${p.index}`, { method: 'DELETE' });
        pollState();
      } catch (e) { toast('Could not drop pose: ' + e.message, 'err'); }
    });

    chip.append(idx, meta, del);
    strip.appendChild(chip);
  }
}

// --- actions --------------------------------------------------------------

async function capture() {
  const btn = $('capture-btn');
  btn.disabled = true;
  try {
    const res = await postJSON('/api/live/capture', {});
    if (res.captured) {
      toast(`Captured pose ${res.index + 1} · orientation span ${res.normal_span}`, 'ok', 2600);
      for (const hint of res.hints || []) toast(hint, 'warn', 6000);
    } else {
      toast((res.reasons || ['Could not capture']).join(' · '), 'warn');
    }
    await pollState();
  } catch (e) {
    toast('Capture failed: ' + e.message, 'err');
  } finally {
    btn.disabled = false;
  }
}

async function finish() {
  const btn = $('finish-btn');
  btn.disabled = true;
  setStatus($('live-status'), 'Solving…', 'running');
  try {
    const res = await postJSON('/api/live/finish', {
      name: $('session-name').value || 'live_session',
      solve: true,
    });
    setStatus($('live-status'), 'Solved', 'ok');

    let host = $('live-results');
    if (!host) {
      host = document.createElement('div');
      host.id = 'live-results';
      $('live-capture').appendChild(host);
    }
    const payload = res.result;
    chart = renderResult(host, payload.result, payload.stdout, chart);
    host.scrollIntoView({ behavior: 'smooth', block: 'nearest' });

    toast(`Solved ${payload.result.poses_used} poses · mean residual ` +
          `${(payload.result.mean_residual_m * 1000).toFixed(2)} mm · ` +
          `saved as ${res.dataset}`, 'ok', 9000);
  } catch (e) {
    setStatus($('live-status'), 'Solve failed', 'err');
    const blocking = e.detail?.blocking;
    toast(blocking ? blocking.join(' · ') : ('Solve failed: ' + e.message), 'err', 9000);
  } finally {
    btn.disabled = false;
  }
}

// --- init -----------------------------------------------------------------

export function initLive() {
  cloudViewer = new CloudViewer($('cloud-view'));

  $('refresh-sources').addEventListener('click', refreshSources);
  $('diagnose-btn').addEventListener('click', diagnose);
  $('camera-source').addEventListener('change', () => { updateHints(); loadDefaults(); });
  $('lidar-source').addEventListener('change', () => { updateHints(); loadDefaults(); });
  $('start-live').addEventListener('click', start);
  $('stop-live').addEventListener('click', stop);
  $('capture-btn').addEventListener('click', capture);
  $('finish-btn').addEventListener('click', finish);

  // Space is the natural shortcut when you are holding a board in both hands.
  document.addEventListener('keydown', (e) => {
    if (e.code !== 'Space' || !running) return;
    if (['INPUT', 'SELECT', 'TEXTAREA', 'BUTTON'].includes(e.target.tagName)) return;
    e.preventDefault();
    if (!$('capture-btn').disabled) capture();
  });

  // Streams must be released when the tab closes, or the camera device stays
  // busy and the next session cannot open it.
  window.addEventListener('beforeunload', () => {
    if (running) navigator.sendBeacon('/api/live/stop');
  });

  refreshSources();
}
