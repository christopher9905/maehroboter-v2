// MV2 Feldkommando — Web UI client
'use strict';

// ── Map ───────────────────────────────────────────────────────────────────
const map = L.map('map', { zoomControl: true, attributionControl: true })
  .setView([48.5, 11.0], 18);

L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
  attribution: '© <a href="https://carto.com/">CARTO</a> © <a href="https://www.openstreetmap.org/copyright">OSM</a>',
  subdomains: 'abcd',
  maxZoom: 22,
}).addTo(map);

// Custom robot marker: pulsing dot
const robotIcon = L.divIcon({
  className: '',
  html: '<div class="r-wrap"><div class="r-ping"></div><div class="r-dot"></div></div>',
  iconSize: [14, 14],
  iconAnchor: [7, 7],
});

const robotMarker = L.marker([48.5, 11.0], { icon: robotIcon }).addTo(map);

// ── State ─────────────────────────────────────────────────────────────────
let currentState = 'IDLE';

const STATE_DISPLAY = {
  IDLE:               'IDLE',
  TEACH_IN:           'TEACH IN',
  MOWING:             'MOWING',
  OBSTACLE_AVOIDANCE: 'OBSTACLE',
  RETURNING:          'RETURNING',
  DOCKING:            'DOCKING',
  CHARGING:           'CHARGING',
  ERROR:              'ERROR',
};

function updateUI(state) {
  if (state === currentState && document.documentElement.dataset.state === state) return;
  currentState = state;

  // Shift the whole UI's accent colour
  document.documentElement.dataset.state = state;

  // Animate state name
  const nameEl = document.getElementById('state-name');
  nameEl.classList.remove('pop');
  void nameEl.offsetWidth; // force reflow
  nameEl.classList.add('pop');
  const label = STATE_DISPLAY[state] || state;
  nameEl.textContent = label;
  document.getElementById('header-state').textContent = label;

  // Button enable/disable
  d('btn-start',       state !== 'IDLE');
  d('btn-stop',        state !== 'MOWING' && state !== 'OBSTACLE_AVOIDANCE');
  d('btn-teach-start', state !== 'IDLE');
  d('btn-teach-stop',  state !== 'TEACH_IN');
  d('btn-reset',       state !== 'ERROR');
}

function d(id, disabled) {
  const el = document.getElementById(id);
  if (el) el.disabled = disabled;
}

// ── WebSocket ─────────────────────────────────────────────────────────────
const pip = document.getElementById('ws-pip');

function connectWS() {
  const ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onopen = () => {
    pip.className = 'ws-pip on';
  };

  ws.onclose = () => {
    pip.className = 'ws-pip';
    setTimeout(connectWS, 3000);
  };

  ws.onerror = () => {
    pip.className = 'ws-pip err';
  };

  ws.onmessage = ({ data }) => {
    let msg;
    try { msg = JSON.parse(data); } catch { return; }

    if (msg.type === 'state') {
      updateUI(msg.state);
    }

    if (msg.type === 'pose') {
      robotMarker.setLatLng([msg.lat, msg.lon]);
      document.getElementById('header-coords').textContent =
        `${msg.lat.toFixed(6)}, ${msg.lon.toFixed(6)}`;
      document.getElementById('tel-speed').textContent =
        (msg.speed_mps * 3.6).toFixed(1);
      if (msg.heading_deg != null) {
        document.getElementById('tel-heading').textContent =
          Math.round(msg.heading_deg) + '°';
      }
    }

    if (msg.type === 'soc') {
      const pct = msg.soc_percent;
      document.getElementById('tel-soc').textContent = pct;
      const fill = document.getElementById('bat-fill');
      fill.style.width = pct + '%';
      // Colour: red < 20%, orange < 40%, else accent via CSS
      fill.style.background = pct < 20 ? '#f87171' : pct < 40 ? '#fb923c' : '';
    }

    if (msg.type === 'gps') {
      const q = msg.fix_quality;
      document.getElementById('tel-gps').textContent =
        q === 4 ? 'RTK Fix' : q === 5 ? 'RTK Float' : q === 1 ? 'GPS' : 'No Fix';
    }

    if (msg.type === 'error') {
      logError(msg.reason);
    }
  };
}

connectWS();

// ── Error log ──────────────────────────────────────────────────────────────
function logError(msg) {
  const el = document.getElementById('err-log');
  el.textContent = msg + (el.textContent ? '\n' + el.textContent : '');
  el.classList.add('visible');
}

function clearErrors() {
  const el = document.getElementById('err-log');
  el.textContent = '';
  el.classList.remove('visible');
}

// ── API calls ──────────────────────────────────────────────────────────────
async function apiPost(path) {
  try {
    const resp = await fetch(path, { method: 'POST' });
    if (!resp.ok) {
      const err = await resp.json().catch(() => ({ detail: resp.statusText }));
      logError(err.detail || resp.statusText);
    }
    return resp;
  } catch (e) {
    logError('Verbindungsfehler: ' + e.message);
  }
}

// ── Button wiring ──────────────────────────────────────────────────────────
document.getElementById('btn-start')
  .addEventListener('click', () => apiPost('/api/mission/start'));

document.getElementById('btn-stop')
  .addEventListener('click', () => apiPost('/api/mission/stop'));

document.getElementById('btn-teach-start')
  .addEventListener('click', () => apiPost('/api/teach-in/start'));

document.getElementById('btn-teach-stop')
  .addEventListener('click', () => apiPost('/api/teach-in/stop'));

document.getElementById('btn-reset')
  .addEventListener('click', () => {
    apiPost('/api/reset');
    clearErrors();
  });

// ── Init ───────────────────────────────────────────────────────────────────
updateUI('IDLE');
