// MV2 Feldkommando — App JS
'use strict';

// ═══════════════════════════════════════════════════════════════
// MAP
// ═══════════════════════════════════════════════════════════════
const map = L.map('map', { zoomControl: true, attributionControl: true })
  .setView([48.5, 11.0], 17);

// Satellite layer (Esri World Imagery, free, no API key)
const satelliteLayer = L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  { attribution: 'Tiles © Esri — Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community', maxZoom: 22 }
);
// Dark fallback for non-home views
const darkLayer = L.tileLayer(
  'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',
  { attribution: '© CARTO © OSM', subdomains: 'abcd', maxZoom: 22 }
);

satelliteLayer.addTo(map);

// Custom robot marker with two ping rings
const robotIcon = L.divIcon({
  className: '',
  html: `<div class="r-wrap">
    <div class="r-ring"></div>
    <div class="r-ring r-ring2"></div>
    <div class="r-dot"></div>
  </div>`,
  iconSize: [16, 16],
  iconAnchor: [8, 8],
});
const robotMarker = L.marker([48.5, 11.0], { icon: robotIcon }).addTo(map);

// ═══════════════════════════════════════════════════════════════
// VIEW ROUTER
// ═══════════════════════════════════════════════════════════════
const VIEWS = ['v-home', 'v-schedule', 'v-history', 'v-settings'];
let currentView = 'v-home';

function showView(id) {
  currentView = id;
  VIEWS.forEach(v => {
    document.getElementById(v).classList.toggle('active', v === id);
  });
  // Nav buttons
  document.querySelectorAll('.nav-btn').forEach(btn => {
    btn.classList.toggle('active', btn.dataset.view === id);
  });
  // On non-home views swap to dark map
  if (id === 'v-home') {
    if (!map.hasLayer(satelliteLayer)) { map.removeLayer(darkLayer); satelliteLayer.addTo(map); }
  } else {
    if (!map.hasLayer(darkLayer)) { map.removeLayer(satelliteLayer); darkLayer.addTo(map); }
  }
  // Render dynamic views on show
  if (id === 'v-schedule') renderSchedule();
  if (id === 'v-history')  renderHistory();
}

document.querySelectorAll('.nav-btn[data-view]').forEach(btn => {
  btn.addEventListener('click', () => showView(btn.dataset.view));
});

// ═══════════════════════════════════════════════════════════════
// STATE MACHINE UI
// ═══════════════════════════════════════════════════════════════
const STATE_LABELS = {
  IDLE: 'Bereit', TEACH_IN: 'Teach-In', MOWING: 'Mähen',
  OBSTACLE_AVOIDANCE: 'Hindernis', RETURNING: 'Rückkehr',
  DOCKING: 'Andocken', CHARGING: 'Laden', ERROR: 'Fehler',
};

let currentState = 'IDLE';

function updateUI(state) {
  if (state === currentState && document.documentElement.dataset.state) return;
  currentState = state;
  document.documentElement.dataset.state = state;

  // State card text
  const el = document.getElementById('sc-state');
  el.classList.remove('pop');
  void el.offsetWidth;
  el.classList.add('pop');
  el.textContent = STATE_LABELS[state] || state;

  // Action buttons in state card
  renderStateActions(state);
}

function renderStateActions(state) {
  const wrap = document.getElementById('sc-actions');
  wrap.innerHTML = '';

  const add = (text, cls, fn) => {
    const b = document.createElement('button');
    b.className = `sc-btn ${cls}`;
    b.textContent = text;
    b.onclick = fn;
    wrap.appendChild(b);
    return b;
  };

  switch (state) {
    case 'IDLE':
      add('▶  Mission starten', 'sc-btn-primary', () => apiPost('/api/mission/start'));
      add('✏  Teach-In', 'sc-btn-ghost', () => openTeachWizard());
      break;
    case 'MOWING':
      add('⏹  Stopp', 'sc-btn-danger', () => apiPost('/api/mission/stop'));
      break;
    case 'OBSTACLE_AVOIDANCE':
      add('⏹  Manuell stoppen', 'sc-btn-danger', () => apiPost('/api/mission/stop'));
      break;
    case 'RETURNING':
    case 'DOCKING':
    case 'CHARGING':
      // no manual action; show info
      const info = document.createElement('div');
      info.style.cssText = 'font-size:12px;color:var(--text-3);text-align:center;padding:6px 0';
      info.textContent = { RETURNING: 'Roboter kehrt zur Basis zurück…', DOCKING: 'Dockt an Ladestation an…', CHARGING: 'Wird geladen…' }[state];
      wrap.appendChild(info);
      break;
    case 'TEACH_IN':
      add('⏹  Teach-In beenden', 'sc-btn-danger', () => apiPost('/api/teach-in/stop'));
      break;
    case 'ERROR':
      add('↺  Fehler zurücksetzen', 'sc-btn-danger', () => apiPost('/api/reset'));
      break;
    default:
      break;
  }
}

// ═══════════════════════════════════════════════════════════════
// WEBSOCKET
// ═══════════════════════════════════════════════════════════════
const wsDot   = document.getElementById('ws-dot');
const topConn = document.getElementById('top-conn-label');

function connectWS() {
  const ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onopen = () => {
    wsDot.className = 'ws-dot on';
    topConn.textContent = 'Verbunden';
  };
  ws.onclose = () => {
    wsDot.className = 'ws-dot';
    topConn.textContent = 'Getrennt';
    setTimeout(connectWS, 3000);
  };
  ws.onerror = () => {
    wsDot.className = 'ws-dot err';
    topConn.textContent = 'Fehler';
  };

  ws.onmessage = ({ data }) => {
    let msg;
    try { msg = JSON.parse(data); } catch { return; }

    if (msg.type === 'state') updateUI(msg.state);

    if (msg.type === 'pose') {
      robotMarker.setLatLng([msg.lat, msg.lon]);
      document.getElementById('tel-speed').textContent   = (msg.speed_mps * 3.6).toFixed(1);
      if (msg.heading_deg != null)
        document.getElementById('tel-heading').textContent = Math.round(msg.heading_deg) + '°';
    }

    if (msg.type === 'soc') {
      const v = msg.soc_percent;
      document.getElementById('tel-soc').textContent    = v;
      document.getElementById('top-soc-val').textContent = v + ' %';
      addHistoryPoint(v); // track for stats
    }

    if (msg.type === 'gps') {
      const q = msg.fix_quality;
      document.getElementById('tel-gps').textContent =
        q === 4 ? 'RTK Fix' : q === 5 ? 'RTK Float' : q === 1 ? 'GPS' : 'No Fix';
    }

    if (msg.type === 'error') logSessionError(msg.reason);
  };
}
connectWS();

// ═══════════════════════════════════════════════════════════════
// API CALLS
// ═══════════════════════════════════════════════════════════════
async function apiPost(path) {
  try {
    const r = await fetch(path, { method: 'POST' });
    if (!r.ok) {
      const e = await r.json().catch(() => ({ detail: r.statusText }));
      console.warn('API error:', e.detail);
    }
    return r;
  } catch (e) { console.error('Network error:', e); }
}

// ═══════════════════════════════════════════════════════════════
// TEACH-IN WIZARD
// ═══════════════════════════════════════════════════════════════
const WIZ_STEPS = 4;
let wizStep = 1;
let recTimer = null;
let recSecs  = 0;

function openTeachWizard() {
  wizStep = 1;
  renderWizDots();
  showWizStep(1);
  document.getElementById('teach-wizard').classList.add('open');
  // reset step 2
  document.getElementById('wiz-rec-indicator').classList.add('hidden');
  document.getElementById('wiz-s2-actions').classList.remove('hidden');
  document.getElementById('wiz-s2-stop').classList.add('hidden');
  clearInterval(recTimer);
  recSecs = 0;
}

function wizClose() {
  document.getElementById('teach-wizard').classList.remove('open');
  clearInterval(recTimer);
  // If teaching was active, stop it
  if (currentState === 'TEACH_IN') apiPost('/api/teach-in/stop');
}

function wizNext() {
  wizStep++;
  if (wizStep > WIZ_STEPS) { wizClose(); return; }
  renderWizDots();
  showWizStep(wizStep);
  // Step 4: fill success text
  if (wizStep === 4) {
    const name = document.getElementById('zone-name-input').value || 'Unbekannt';
    document.getElementById('wiz-success-body').textContent =
      `Der Bereich „${name}" wurde erfolgreich gespeichert. Du kannst jetzt eine Mission starten.`;
    saveZone(name);
  }
}

function showWizStep(n) {
  for (let i = 1; i <= WIZ_STEPS; i++)
    document.getElementById(`wiz-s${i}`).classList.toggle('active', i === n);
}

function renderWizDots() {
  const wrap = document.getElementById('wiz-dots');
  wrap.innerHTML = '';
  for (let i = 1; i <= WIZ_STEPS; i++) {
    const d = document.createElement('div');
    d.className = 'wiz-step-dot';
    d.style.width = i < wizStep ? '24px' : i === wizStep ? '16px' : '8px';
    if (i < wizStep)     d.classList.add('done');
    else if (i === wizStep) d.classList.add('curr');
    wrap.appendChild(d);
  }
}

async function wizStartRecording() {
  await apiPost('/api/teach-in/start');
  document.getElementById('wiz-rec-indicator').classList.remove('hidden');
  document.getElementById('wiz-s2-actions').classList.add('hidden');
  document.getElementById('wiz-s2-stop').classList.remove('hidden');
  recSecs = 0;
  recTimer = setInterval(() => {
    recSecs++;
    const m = String(Math.floor(recSecs / 60)).padStart(2,'0');
    const s = String(recSecs % 60).padStart(2,'0');
    document.getElementById('wiz-rec-time').textContent = `${m}:${s}`;
  }, 1000);
}

async function wizStopRecording() {
  clearInterval(recTimer);
  await apiPost('/api/teach-in/stop');
  wizNext(); // → step 3 (name)
}

// Teach-In shortcut buttons
document.getElementById('teach-fab-btn').addEventListener('click', openTeachWizard);
document.getElementById('teach-nav-btn').addEventListener('click', () => { showView('v-home'); openTeachWizard(); });

// ═══════════════════════════════════════════════════════════════
// SCHEDULE (localStorage)
// ═══════════════════════════════════════════════════════════════
const DAYS = ['Mo','Di','Mi','Do','Fr','Sa','So'];

function getSchedules() {
  try { return JSON.parse(localStorage.getItem('mv2_schedules') || '[]'); } catch { return []; }
}
function saveSchedules(list) { localStorage.setItem('mv2_schedules', JSON.stringify(list)); }

function renderSchedule() {
  const list = getSchedules();
  const wrap = document.getElementById('schedule-list');
  wrap.innerHTML = '';

  if (list.length === 0) {
    const e = document.createElement('div');
    e.className = 'empty-state';
    e.innerHTML = '<div class="empty-state-icon">📅</div>Kein Zeitplan vorhanden.<br>Füge einen neuen Zeitplan hinzu.';
    wrap.appendChild(e);
  } else {
    list.forEach((s, idx) => {
      const card = document.createElement('div');
      card.className = 'schedule-card';
      card.innerHTML = `
        <div class="sched-top">
          <div>
            <div class="sched-time">${s.hour}:${s.min}</div>
          </div>
          <div class="sched-info">
            <div class="sched-days">
              ${DAYS.map(d => `<div class="day-chip ${s.days.includes(d)?'on':''}">${d}</div>`).join('')}
            </div>
          </div>
          <button class="sched-delete" onclick="deleteSchedule(${idx})">
            <svg viewBox="0 0 24 24"><polyline points="3 6 5 6 21 6"/><path d="M19 6v14a2 2 0 01-2 2H7a2 2 0 01-2-2V6m3 0V4a1 1 0 011-1h4a1 1 0 011 1v2"/></svg>
          </button>
        </div>
        <div class="sched-footer">
          <div class="sched-badge">Aktiv</div>
          <div class="sched-footer-label">Dauer: ${s.dur >= 60 ? (s.dur/60 % 1 === 0 ? s.dur/60 + ' h' : (s.dur/60).toFixed(1) + ' h') : s.dur + ' min'}</div>
        </div>
      `;
      wrap.appendChild(card);
    });
  }

  // Add button always at bottom
  const addBtn = document.createElement('button');
  addBtn.className = 'add-schedule-btn';
  addBtn.innerHTML = `<svg viewBox="0 0 24 24"><line x1="12" y1="5" x2="12" y2="19"/><line x1="5" y1="12" x2="19" y2="12"/></svg> Zeitplan hinzufügen`;
  addBtn.onclick = openSchedModal;
  wrap.appendChild(addBtn);
}

function deleteSchedule(idx) {
  const list = getSchedules();
  list.splice(idx, 1);
  saveSchedules(list);
  renderSchedule();
}

document.getElementById('add-sched-btn').addEventListener('click', openSchedModal);

// Populate hour options
(function() {
  const sel = document.getElementById('m-hour');
  for (let h = 6; h <= 20; h++) {
    const o = document.createElement('option');
    o.value = String(h).padStart(2,'0');
    o.textContent = String(h).padStart(2,'0');
    if (h === 9) o.selected = true;
    sel.appendChild(o);
  }
})();

function openSchedModal() {
  // reset day chips to Mo–Fr
  document.querySelectorAll('#m-days .day-chip').forEach(c => {
    c.classList.toggle('on', ['Mo','Di','Mi','Do','Fr'].includes(c.dataset.day));
  });
  document.getElementById('sched-modal').classList.add('open');
}
function closeSchedModal() { document.getElementById('sched-modal').classList.remove('open'); }

// Day chip toggle in modal
document.querySelectorAll('#m-days .day-chip').forEach(c => {
  c.addEventListener('click', () => c.classList.toggle('on'));
});

// Duration option
document.querySelectorAll('#m-dur .dur-opt').forEach(o => {
  o.addEventListener('click', () => {
    document.querySelectorAll('#m-dur .dur-opt').forEach(x => x.classList.remove('on'));
    o.classList.add('on');
  });
});

function saveSchedule() {
  const hour = document.getElementById('m-hour').value;
  const min  = document.getElementById('m-min').value;
  const days = [...document.querySelectorAll('#m-days .day-chip.on')].map(c => c.dataset.day);
  const dur  = parseInt(document.querySelector('#m-dur .dur-opt.on')?.dataset.val || '90');
  if (!days.length) { alert('Bitte mindestens einen Tag wählen.'); return; }
  const list = getSchedules();
  list.push({ hour, min, days, dur });
  saveSchedules(list);
  closeSchedModal();
  renderSchedule();
}

// ═══════════════════════════════════════════════════════════════
// HISTORY (localStorage)
// ═══════════════════════════════════════════════════════════════
let _mowingStart = null;

// Track mowing sessions via state changes
function trackSession(old, newState) {
  if (newState === 'MOWING') _mowingStart = Date.now();
  if ((old === 'MOWING' || old === 'RETURNING') && (newState === 'IDLE' || newState === 'CHARGING' || newState === 'ERROR')) {
    if (_mowingStart) {
      const dur = Math.round((Date.now() - _mowingStart) / 60000); // minutes
      addSession({ date: new Date().toISOString(), dur, status: newState === 'ERROR' ? 'error' : 'ok', area: Math.round(dur * 35) });
      _mowingStart = null;
    }
  }
}

function getSessions() { try { return JSON.parse(localStorage.getItem('mv2_sessions') || '[]'); } catch { return []; } }
function addSession(s) {
  const list = getSessions();
  list.unshift(s); // newest first
  if (list.length > 50) list.splice(50);
  localStorage.setItem('mv2_sessions', JSON.stringify(list));
}
function addHistoryPoint() {} // hook for future live data

function renderHistory() {
  const sessions = getSessions();

  // Stats
  const totalMins = sessions.reduce((a, s) => a + (s.dur || 0), 0);
  const totalArea = sessions.reduce((a, s) => a + (s.area || 0), 0);
  document.getElementById('stats-row').innerHTML = `
    <div class="stat-card"><div class="stat-val">${sessions.length}</div><div class="stat-lbl">Sitzungen</div></div>
    <div class="stat-card"><div class="stat-val">${totalMins >= 60 ? (totalMins/60).toFixed(1)+'h' : totalMins+'m'}</div><div class="stat-lbl">Gesamtzeit</div></div>
    <div class="stat-card"><div class="stat-val">${totalArea >= 1000 ? (totalArea/1000).toFixed(2)+'k' : totalArea}</div><div class="stat-lbl">m² gemäht</div></div>
  `;

  const list = document.getElementById('session-list');
  if (!sessions.length) {
    list.innerHTML = '<div class="empty-state"><div class="empty-state-icon">📊</div>Noch keine Mähsitzungen.<br>Starte eine Mission, um den Verlauf aufzuzeichnen.</div>';
    // Show demo data
    renderDemoHistory(list);
    return;
  }

  list.innerHTML = '';
  sessions.forEach(s => {
    const d = new Date(s.date);
    const dateStr = d.toLocaleDateString('de-DE', { weekday:'short', day:'2-digit', month:'short' });
    const timeStr = d.toLocaleTimeString('de-DE', { hour:'2-digit', minute:'2-digit' });
    const iconCls = s.status === 'error' ? 'err' : s.status === 'warn' ? 'warn' : 'ok';
    const icon    = s.status === 'error' ? errIcon() : s.status === 'warn' ? warnIcon() : checkIcon();
    list.innerHTML += `
      <div class="session-card">
        <div class="session-icon ${iconCls}">${icon}</div>
        <div class="session-info">
          <div class="session-date">${dateStr}</div>
          <div class="session-meta">${timeStr} Uhr · ${s.dur} min</div>
        </div>
        <div class="session-right">
          <div class="session-area">${s.area}</div>
          <div class="session-dur">m²</div>
        </div>
      </div>`;
  });
}

function renderDemoHistory(wrap) {
  const demo = [
    { date: '2026-04-14T09:12:00', dur: 42, status: 'ok',   area: 1470 },
    { date: '2026-04-12T10:05:00', dur: 55, status: 'ok',   area: 1925 },
    { date: '2026-04-10T08:30:00', dur: 38, status: 'warn', area: 1330 },
    { date: '2026-04-08T09:45:00', dur: 61, status: 'ok',   area: 2135 },
  ];
  document.getElementById('stats-row').innerHTML = `
    <div class="stat-card"><div class="stat-val">4</div><div class="stat-lbl">Sitzungen</div></div>
    <div class="stat-card"><div class="stat-val">3.3h</div><div class="stat-lbl">Gesamtzeit</div></div>
    <div class="stat-card"><div class="stat-val">6.8k</div><div class="stat-lbl">m² gemäht</div></div>
  `;
  wrap.innerHTML = '';
  demo.forEach(s => {
    const d = new Date(s.date);
    const dateStr = d.toLocaleDateString('de-DE', { weekday:'short', day:'2-digit', month:'short' });
    const timeStr = d.toLocaleTimeString('de-DE', { hour:'2-digit', minute:'2-digit' });
    const iconCls = s.status === 'err' ? 'err' : s.status === 'warn' ? 'warn' : 'ok';
    const icon    = s.status === 'err' ? errIcon() : s.status === 'warn' ? warnIcon() : checkIcon();
    wrap.innerHTML += `
      <div class="session-card">
        <div class="session-icon ${iconCls}">${icon}</div>
        <div class="session-info">
          <div class="session-date">${dateStr}</div>
          <div class="session-meta">${timeStr} Uhr · ${s.dur} min</div>
        </div>
        <div class="session-right">
          <div class="session-area">${s.area}</div>
          <div class="session-dur">m²</div>
        </div>
      </div>`;
  });
}

function checkIcon() { return '<svg viewBox="0 0 24 24" style="width:20px;height:20px;stroke:currentColor;fill:none;stroke-width:2.5;stroke-linecap:round;stroke-linejoin:round"><polyline points="20 6 9 17 4 12"/></svg>'; }
function warnIcon()  { return '<svg viewBox="0 0 24 24" style="width:20px;height:20px;stroke:currentColor;fill:none;stroke-width:2;stroke-linecap:round"><path d="M10.29 3.86L1.82 18a2 2 0 001.71 3h16.94a2 2 0 001.71-3L13.71 3.86a2 2 0 00-3.42 0z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/></svg>'; }
function errIcon()   { return '<svg viewBox="0 0 24 24" style="width:20px;height:20px;stroke:currentColor;fill:none;stroke-width:2;stroke-linecap:round"><circle cx="12" cy="12" r="10"/><line x1="15" y1="9" x2="9" y2="15"/><line x1="9" y1="9" x2="15" y2="15"/></svg>'; }

function logSessionError(reason) { console.warn('Session error:', reason); }

// ═══════════════════════════════════════════════════════════════
// SETTINGS (localStorage)
// ═══════════════════════════════════════════════════════════════
function getSettings() {
  const def = { lane:38, speed:5, tilt:30, rainwait:30, rain:true, geo:true, robotName:'MV2-Alpha' };
  try { return Object.assign(def, JSON.parse(localStorage.getItem('mv2_settings') || '{}')); }
  catch { return def; }
}
function saveSettings(s) { localStorage.setItem('mv2_settings', JSON.stringify(s)); }

function initSettings() {
  const s = getSettings();

  // Lane spacing
  const laneEl = document.getElementById('s-lane');
  laneEl.value = s.lane;
  document.getElementById('s-lane-val').textContent = s.lane + ' cm';
  laneEl.addEventListener('input', () => {
    document.getElementById('s-lane-val').textContent = laneEl.value + ' cm';
    saveSettings(Object.assign(getSettings(), { lane: +laneEl.value }));
  });

  // Speed
  const speedEl = document.getElementById('s-speed');
  speedEl.value = s.speed;
  document.getElementById('s-speed-val').textContent = s.speed;
  speedEl.addEventListener('input', () => {
    document.getElementById('s-speed-val').textContent = speedEl.value;
    saveSettings(Object.assign(getSettings(), { speed: +speedEl.value }));
  });

  // Tilt
  const tiltEl = document.getElementById('s-tilt');
  tiltEl.value = s.tilt;
  document.getElementById('s-tilt-val').textContent = s.tilt + '°';
  tiltEl.addEventListener('input', () => {
    document.getElementById('s-tilt-val').textContent = tiltEl.value + '°';
    saveSettings(Object.assign(getSettings(), { tilt: +tiltEl.value }));
  });

  // Rain wait
  const rainwaitEl = document.getElementById('s-rainwait');
  rainwaitEl.value = s.rainwait;
  document.getElementById('s-rainwait-val').textContent = s.rainwait + ' min';
  rainwaitEl.addEventListener('input', () => {
    document.getElementById('s-rainwait-val').textContent = rainwaitEl.value + ' min';
    saveSettings(Object.assign(getSettings(), { rainwait: +rainwaitEl.value }));
  });

  // Toggles
  const rainEl = document.getElementById('s-rain');
  rainEl.checked = s.rain;
  rainEl.addEventListener('change', () => saveSettings(Object.assign(getSettings(), { rain: rainEl.checked })));

  const geoEl = document.getElementById('s-geo');
  geoEl.checked = s.geo;
  geoEl.addEventListener('change', () => saveSettings(Object.assign(getSettings(), { geo: geoEl.checked })));

  // Robot name
  const namePreview = document.getElementById('robot-name-preview');
  namePreview.textContent = s.robotName;
  document.getElementById('robot-name-input').value = s.robotName;

  document.getElementById('card-robot-name').addEventListener('click', openNameModal);
  document.getElementById('reset-btn-settings').addEventListener('click', () => apiPost('/api/reset'));
}

function openNameModal() {
  document.getElementById('robot-name-input').value = getSettings().robotName;
  document.getElementById('name-modal').classList.add('open');
}
function closeNameModal() { document.getElementById('name-modal').classList.remove('open'); }

function saveRobotName() {
  const name = document.getElementById('robot-name-input').value.trim() || 'MV2-Alpha';
  saveSettings(Object.assign(getSettings(), { robotName: name }));
  document.getElementById('robot-name-preview').textContent = name;
  closeNameModal();
}

// ═══════════════════════════════════════════════════════════════
// ZONES (localStorage, minimal)
// ═══════════════════════════════════════════════════════════════
function saveZone(name) {
  const zones = JSON.parse(localStorage.getItem('mv2_zones') || '[]');
  zones.push({ name, date: new Date().toISOString() });
  localStorage.setItem('mv2_zones', JSON.stringify(zones));
}

// ═══════════════════════════════════════════════════════════════
// INIT
// ═══════════════════════════════════════════════════════════════
updateUI('IDLE');
initSettings();
renderWizDots();

// Wire robot name card click (need id on card-row)
document.querySelectorAll('.card-row').forEach(row => {
  const title = row.querySelector('.row-title');
  if (title && title.textContent === 'Roboter-Name') row.id = 'card-robot-name';
});
