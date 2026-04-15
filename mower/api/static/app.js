// Mähroboter V2 — Web UI client
'use strict';

const map = L.map('map').setView([48.5, 11.0], 18);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '© OpenStreetMap contributors',
  maxZoom: 22,
}).addTo(map);

const robotMarker = L.marker([48.5, 11.0], { title: 'Mähroboter' }).addTo(map);

let currentState = 'IDLE';

function updateUI(state) {
  currentState = state;
  const badge = document.getElementById('status-badge');
  badge.textContent = state;
  badge.className = '';
  badge.classList.add(state);
  document.getElementById('btn-start').disabled       = state !== 'IDLE';
  document.getElementById('btn-stop').disabled        = !['MOWING', 'OBSTACLE_AVOIDANCE'].includes(state);
  document.getElementById('btn-teach-start').disabled = state !== 'IDLE';
  document.getElementById('btn-teach-stop').disabled  = state !== 'TEACH_IN';
  document.getElementById('btn-reset').disabled       = state !== 'ERROR';
}

function connectWS() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  const indicator = document.getElementById('ws-indicator');
  ws.onopen  = () => { indicator.textContent = '\uD83D\uDFE2'; };
  ws.onclose = () => { indicator.textContent = '\uD83D\uDD34'; setTimeout(connectWS, 3000); };
  ws.onerror = () => { indicator.textContent = '\uD83D\uDFE0'; };
  ws.onmessage = (evt) => {
    const msg = JSON.parse(evt.data);
    if (msg.type === 'state') updateUI(msg.state);
    if (msg.type === 'pose') {
      robotMarker.setLatLng([msg.lat, msg.lon]);
      document.getElementById('tel-speed').textContent = (msg.speed_mps * 3.6).toFixed(1) + ' km/h';
    }
    if (msg.type === 'soc')  document.getElementById('tel-soc').textContent  = msg.soc_percent + ' %';
    if (msg.type === 'gps')  document.getElementById('tel-gps').textContent  = msg.fix_quality === 4 ? 'RTK Fix' : msg.fix_quality === 5 ? 'RTK Float' : 'No Fix';
    if (msg.type === 'error') {
      const log = document.getElementById('error-log');
      log.textContent = msg.reason + '\n' + log.textContent;
    }
  };
}

connectWS();

async function apiPost(path) {
  const resp = await fetch(path, { method: 'POST' });
  if (!resp.ok) {
    const err = await resp.json().catch(() => ({ detail: resp.statusText }));
    document.getElementById('error-log').textContent = err.detail || resp.statusText;
  }
  return resp;
}

document.getElementById('btn-start').addEventListener('click',       () => apiPost('/api/mission/start'));
document.getElementById('btn-stop').addEventListener('click',        () => apiPost('/api/mission/stop'));
document.getElementById('btn-teach-start').addEventListener('click', () => apiPost('/api/teach-in/start'));
document.getElementById('btn-teach-stop').addEventListener('click',  () => apiPost('/api/teach-in/stop'));
document.getElementById('btn-reset').addEventListener('click',       () => apiPost('/api/reset'));

updateUI('IDLE');
