// Shell: mode switching, environment banner, shared helpers.

import { initOffline } from '/offline.js';
import { initLive } from '/live.js';

export const $ = (id) => document.getElementById(id);

export function fmtNum(n, d = 4) {
  if (n === null || n === undefined || Number.isNaN(n)) return '—';
  return Number(n).toFixed(d);
}

export function fmtVec(v, d = 4) {
  if (!v) return '—';
  return '[' + v.map((x) => fmtNum(x, d)).join(', ') + ']';
}

export function setStatus(el, msg, kind) {
  if (!el) return;
  el.textContent = msg;
  el.className = 'status ' + (kind || 'idle');
}

export function toast(message, kind = '', ttl = 5200) {
  const host = $('toasts');
  const el = document.createElement('div');
  el.className = 'toast ' + kind;
  el.textContent = message;
  host.appendChild(el);
  setTimeout(() => el.remove(), ttl);
}

/** fetch + JSON, surfacing FastAPI's `detail` field as the error message. */
export async function api(path, options = {}) {
  const resp = await fetch(path, options);
  let body = null;
  try { body = await resp.json(); } catch { /* empty or non-JSON body */ }
  if (!resp.ok) {
    const detail = body?.detail;
    const message = typeof detail === 'string'
      ? detail
      : detail?.message || JSON.stringify(detail || body || resp.statusText);
    const error = new Error(message);
    error.detail = detail;
    error.status = resp.status;
    throw error;
  }
  return body;
}

export function postJSON(path, payload) {
  return api(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload ?? {}),
  });
}

// --- mode switching -------------------------------------------------------

function setMode(mode) {
  document.querySelectorAll('.nav-item').forEach((b) =>
    b.classList.toggle('active', b.dataset.mode === mode));
  document.querySelectorAll('.mode').forEach((s) =>
    s.classList.toggle('active', s.id === `mode-${mode}`));
  // Viewers sized while hidden come out at zero; nudge them once visible.
  window.dispatchEvent(new Event('resize'));
}

// --- environment banner ---------------------------------------------------

async function loadHealth() {
  try {
    const h = await api('/api/health');
    const set = (id, ok, okText, badText, kind = 'err') => {
      const el = $(id);
      el.textContent = ok ? okText : badText;
      el.className = 'pill ' + (ok ? 'ok' : kind);
    };
    set('env-binary', h.binary_built, 'built', 'missing');
    set('env-ros', h.ros2_available, 'available', 'not sourced', 'warn');
    set('env-intrinsics', h.intrinsics_file, 'found', 'none', 'warn');

    if (!h.binary_built) {
      toast('Solver binary not built. Run: cmake --build build -j', 'err', 12000);
    }
  } catch (e) {
    toast('Could not reach the backend: ' + e.message, 'err');
  }
}

// --- init -----------------------------------------------------------------

window.addEventListener('DOMContentLoaded', async () => {
  document.querySelectorAll('.nav-item').forEach((btn) =>
    btn.addEventListener('click', () => setMode(btn.dataset.mode)));

  await loadHealth();
  initLive();
  initOffline();
});
