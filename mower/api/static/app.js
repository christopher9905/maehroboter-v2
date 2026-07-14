// MV2 — App Controller
'use strict';

// ═══════════════════════════════════════════════════════════════
// MAP
// ═══════════════════════════════════════════════════════════════
const map = L.map('map', { zoomControl: false, attributionControl: true })
  .setView([48.5, 11.0], 17);

L.control.zoom({ position: 'bottomright' }).addTo(map);

const satelliteLayer = L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  { attribution: 'Tiles © Esri', maxZoom: 22 }
);
const streetLayer = L.tileLayer(
  'https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png',
  { attribution: '© CARTO © OSM', subdomains: 'abcd', maxZoom: 22 }
);

satelliteLayer.addTo(map);
let isSatellite = true;

// Robot marker
const robotIcon = L.divIcon({
  className: '',
  html: `<div class="robot-marker">
    <div class="robot-marker-ring"></div>
    <div class="robot-marker-body"></div>
  </div>`,
  iconSize: [28, 28],
  iconAnchor: [14, 14],
});
const robotMarker = L.marker([48.5, 11.0], { icon: robotIcon }).addTo(map);

// Map control buttons
document.getElementById('map-center-btn')?.addEventListener('click', () => {
  map.setView(robotMarker.getLatLng(), 18, { animate: true });
});
document.getElementById('map-layer-btn')?.addEventListener('click', () => {
  isSatellite = !isSatellite;
  if (isSatellite) {
    map.removeLayer(streetLayer);
    satelliteLayer.addTo(map);
  } else {
    map.removeLayer(satelliteLayer);
    streetLayer.addTo(map);
  }
});

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
  document.querySelectorAll('.nav-btn').forEach(btn => {
    btn.classList.toggle('active', btn.dataset.view === id);
  });
  if (id === 'v-schedule') { renderSchedule(); renderWeekGrid(); updateNextMow(); }
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
const STATE_SUBS = {
  IDLE: 'Bereit zum Starten',
  TEACH_IN: 'Grenze wird aufgezeichnet…',
  MOWING: 'Mission aktiv',
  OBSTACLE_AVOIDANCE: 'Hindernis wird umfahren…',
  RETURNING: 'Kehrt zur Basis zurück…',
  DOCKING: 'Dockt an Ladestation an…',
  CHARGING: 'Wird geladen…',
  ERROR: 'Eingriff erforderlich',
};
const STATE_TOASTS = {
  MOWING:             { msg: 'Mission gestartet', type: 'success' },
  CHARGING:           { msg: 'Ladevorgang gestartet ⚡', type: 'info' },
  ERROR:              { msg: 'Fehler aufgetreten!', type: 'error' },
  OBSTACLE_AVOIDANCE: { msg: 'Hindernis erkannt', type: 'warning' },
  RETURNING:          { msg: 'Rückkehr zur Basis…', type: 'info' },
};

let currentState = 'IDLE';

function updateUI(state) {
  if (state === currentState && document.documentElement.dataset.state) return;
  const old = currentState;
  currentState = state;
  document.documentElement.dataset.state = state;

  // Track mowing session
  trackSession(old, state);

  // Toast on important transitions
  const toast = STATE_TOASTS[state];
  if (toast && old !== state) showToast(toast.msg, toast.type);

  // State label (with pop animation)
  const el = document.getElementById('sc-state');
  el.classList.remove('pop');
  void el.offsetWidth;
  el.classList.add('pop');
  el.textContent = STATE_LABELS[state] || state;

  // Sub label
  const subEl = document.getElementById('state-sub');
  if (subEl) subEl.textContent = STATE_SUBS[state] || '';

  // Action buttons
  renderStateActions(state);

  // Refresh zones
  renderZones();
}

function renderStateActions(state) {
  const wrap = document.getElementById('sc-actions');
  wrap.innerHTML = '';

  const addBtn = (text, cls, fn, icon = '') => {
    const b = document.createElement('button');
    b.className = `ha-btn ${cls}`;
    b.innerHTML = `<span class="ha-btn-icon">${icon}</span><span>${text}</span>`;
    b.onclick = fn;
    wrap.appendChild(b);
    return b;
  };

  const mowerIcon = `<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="10"/><line x1="10" y1="15" x2="10" y2="9"/><line x1="14" y1="9" x2="14" y2="15"/></svg>`;
  const playIcon  = `<svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor"><polygon points="5 3 19 12 5 21 5 3"/></svg>`;
  const stopIcon  = `<svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor"><rect x="3" y="3" width="18" height="18" rx="2"/></svg>`;
  const editIcon  = `<svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><path d="M12 20h9"/><path d="M16.5 3.5a2.121 2.121 0 013 3L7 19l-4 1 1-4L16.5 3.5z"/></svg>`;
  const resetIcon = `<svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><polyline points="1 4 1 10 7 10"/><path d="M3.51 15a9 9 0 102.13-9.36L1 10"/></svg>`;

  switch (state) {
    case 'IDLE':
      addBtn('Mission starten', 'ha-primary', () => apiPost('/api/mission/start'), playIcon);
      addBtn('Teach-In aufnehmen', 'ha-ghost', () => openTeachWizard(), editIcon);
      break;
    case 'MOWING':
      addBtn('Mission stoppen', 'ha-danger', () => apiPost('/api/mission/stop'), stopIcon);
      break;
    case 'OBSTACLE_AVOIDANCE':
      addBtn('Manuell stoppen', 'ha-danger', () => apiPost('/api/mission/stop'), stopIcon);
      break;
    case 'RETURNING':
    case 'DOCKING':
    case 'CHARGING': {
      const info = document.createElement('div');
      info.className = 'ha-info';
      info.textContent = STATE_SUBS[state];
      wrap.appendChild(info);
      break;
    }
    case 'TEACH_IN':
      addBtn('Teach-In beenden', 'ha-danger', () => apiPost('/api/teach-in/stop'), stopIcon);
      break;
    case 'ERROR':
      addBtn('Fehler zurücksetzen', 'ha-danger', () => apiPost('/api/reset'), resetIcon);
      break;
  }
}

// ═══════════════════════════════════════════════════════════════
// BATTERY RING
// ═══════════════════════════════════════════════════════════════
// r=26 in viewBox 66×66 → circumference = 2π×26 ≈ 163.4
const SOC_CIRC = 163.4;

function updateSocRing(pct) {
  const ring = document.getElementById('soc-ring');
  if (!ring) return;
  const p = Math.max(0, Math.min(100, pct));
  ring.style.strokeDasharray  = SOC_CIRC;
  ring.style.strokeDashoffset = SOC_CIRC * (1 - p / 100);
}

// ═══════════════════════════════════════════════════════════════
// TOAST NOTIFICATIONS
// ═══════════════════════════════════════════════════════════════
function showToast(msg, type = 'info') {
  const stack = document.getElementById('toast-stack');
  if (!stack) return;
  const t = document.createElement('div');
  t.className = `toast ${type}`;
  t.innerHTML = `<div class="toast-dot"></div><span>${msg}</span>`;
  stack.appendChild(t);
  requestAnimationFrame(() => { requestAnimationFrame(() => t.classList.add('show')); });
  setTimeout(() => {
    t.classList.remove('show');
    setTimeout(() => t.remove(), 350);
  }, 3500);
}

// ═══════════════════════════════════════════════════════════════
// WEBSOCKET
// ═══════════════════════════════════════════════════════════════
const wsDot  = document.getElementById('ws-dot');
const topConn = document.getElementById('top-conn-label');

function connectWS() {
  const ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onopen = () => {
    wsDot.className = 'ws-dot on';
    topConn.textContent = 'Verbunden';
    showToast('Verbunden ✓', 'success');
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
      map.setView([msg.lat, msg.lon], map.getZoom());
      const el = document.getElementById('tel-speed');
      if (el) el.textContent = (msg.speed_mps * 3.6).toFixed(1);
      const hEl = document.getElementById('tel-heading');
      if (hEl && msg.heading_deg != null) hEl.textContent = Math.round(msg.heading_deg);
    }

    if (msg.type === 'soc') {
      const v = msg.soc_percent;
      const socEl = document.getElementById('tel-soc');
      if (socEl) socEl.textContent = v;
      const topSoc = document.getElementById('top-soc-val');
      if (topSoc) topSoc.textContent = v + ' %';
      updateSocRing(v);
      addHistoryPoint(v);
    }

    if (msg.type === 'gps') {
      const q = msg.fix_quality;
      const gpsEl = document.getElementById('tel-gps');
      if (gpsEl) gpsEl.textContent =
        q === 4 ? 'RTK Fix' : q === 5 ? 'RTK Float' : q === 1 ? 'GPS' : 'Kein Fix';
      const accEl = document.getElementById('gps-accuracy-display');
      if (accEl) accEl.textContent =
        q === 4 ? 'RTK Fix — ±1 cm' : q === 5 ? 'RTK Float — ±10 cm' : q === 1 ? 'GPS — ±3 m' : '—';
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
      showToast('Fehler: ' + e.detail, 'error');
    }
    return r;
  } catch (e) {
    showToast('Netzwerkfehler', 'error');
    console.error('Network error:', e);
  }
}

// ═══════════════════════════════════════════════════════════════
// ZONES
// ═══════════════════════════════════════════════════════════════
const ZONE_COLORS = ['#16A34A','#2563EB','#D97706','#7C3AED','#DB2777','#0891B2'];

function getZones() {
  try { return JSON.parse(localStorage.getItem('mv2_zones') || '[]'); } catch { return []; }
}

function renderZones() {
  const zones = getZones();
  const list = document.getElementById('zone-list');
  if (!list) return;

  if (!zones.length) {
    list.innerHTML = `<div class="zone-empty">
      <div class="zone-empty-icon">🌿</div>
      <p>Noch keine Bereiche gespeichert.<br>Starte ein Teach-In, um den Mähbereich abzugrenzen.</p>
    </div>`;
    return;
  }

  list.innerHTML = zones.map((z, i) => {
    const color = ZONE_COLORS[i % ZONE_COLORS.length];
    const d = new Date(z.date);
    const dateStr = d.toLocaleDateString('de-DE', { day: '2-digit', month: 'short', year: 'numeric' });
    return `<div class="zone-item">
      <div class="zone-swatch" style="background:${color}"></div>
      <div class="zone-info">
        <div class="zone-name">${z.name}</div>
        <div class="zone-meta">Aufgezeichnet am ${dateStr}</div>
      </div>
      <button class="zone-play" title="Mission in diesem Bereich" onclick="startMissionInZone('${z.name}')">
        <svg width="12" height="12" viewBox="0 0 24 24" fill="currentColor"><polygon points="5 3 19 12 5 21 5 3"/></svg>
      </button>
    </div>`;
  }).join('');

  // Show last session section if we have sessions
  const sessions = getSessions();
  if (sessions.length) showLastSession(sessions[0]);
}

function startMissionInZone(name) {
  showToast(`Mission in „${name}" wird gestartet…`, 'info');
  apiPost('/api/mission/start');
}

function saveZone(name) {
  const zones = getZones();
  zones.push({ name, date: new Date().toISOString() });
  localStorage.setItem('mv2_zones', JSON.stringify(zones));
  renderZones();
}

// ═══════════════════════════════════════════════════════════════
// LAST SESSION SUMMARY
// ═══════════════════════════════════════════════════════════════
function showLastSession(s) {
  const sec = document.getElementById('last-session-section');
  const card = document.getElementById('last-session-card');
  if (!sec || !card) return;
  sec.style.display = 'block';
  const d = new Date(s.date);
  const dateStr = d.toLocaleDateString('de-DE', { weekday: 'short', day: '2-digit', month: 'short' });
  const timeStr = d.toLocaleTimeString('de-DE', { hour: '2-digit', minute: '2-digit' });
  const iconColor = s.status === 'error' ? '#DC2626' : s.status === 'warn' ? '#D97706' : '#16A34A';
  const iconBg    = s.status === 'error' ? '#FEE2E2' : s.status === 'warn' ? '#FEF3C7' : '#DCFCE7';
  card.innerHTML = `
    <div class="last-sess-icon" style="background:${iconBg};color:${iconColor}">
      ${checkSVG()}
    </div>
    <div class="last-sess-info">
      <div class="last-sess-date">${dateStr}, ${timeStr} Uhr</div>
      <div class="last-sess-meta">${s.dur} min Laufzeit</div>
    </div>
    <div>
      <div class="last-sess-area">${s.area}</div>
      <div class="last-sess-unit">m²</div>
    </div>`;
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
  document.getElementById('wiz-rec-indicator').classList.add('hidden');
  document.getElementById('wiz-s2-actions').classList.remove('hidden');
  document.getElementById('wiz-s2-stop').classList.add('hidden');
  clearInterval(recTimer);
  recSecs = 0;
}

function wizClose() {
  document.getElementById('teach-wizard').classList.remove('open');
  clearInterval(recTimer);
  if (currentState === 'TEACH_IN') apiPost('/api/teach-in/stop');
}

function wizNext() {
  wizStep++;
  if (wizStep > WIZ_STEPS) { wizClose(); return; }
  renderWizDots();
  showWizStep(wizStep);
  if (wizStep === 4) {
    const name = document.getElementById('zone-name-input').value.trim() || 'Bereich ' + (getZones().length + 1);
    document.getElementById('wiz-success-body').textContent =
      `Der Bereich „${name}" wurde erfolgreich gespeichert. Du kannst jetzt eine Mission starten.`;
    saveZone(name);
    showToast(`Bereich „${name}" gespeichert ✓`, 'success');
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
    d.className = 'wiz-dot ' + (i < wizStep ? 'done' : i === wizStep ? 'curr' : 'next');
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
    const m = String(Math.floor(recSecs / 60)).padStart(2, '0');
    const s = String(recSecs % 60).padStart(2, '0');
    document.getElementById('wiz-rec-time').textContent = `${m}:${s}`;
  }, 1000);
}

async function wizStopRecording() {
  clearInterval(recTimer);
  await apiPost('/api/teach-in/stop');
  wizNext();
}

document.getElementById('teach-fab-btn').addEventListener('click', openTeachWizard);
document.getElementById('teach-nav-btn').addEventListener('click', () => {
  showView('v-home');
  openTeachWizard();
});

// ═══════════════════════════════════════════════════════════════
// SCHEDULE (localStorage)
// ═══════════════════════════════════════════════════════════════
const DAYS = ['Mo', 'Di', 'Mi', 'Do', 'Fr', 'Sa', 'So'];

function getSchedules() {
  try { return JSON.parse(localStorage.getItem('mv2_schedules') || '[]'); } catch { return []; }
}
function saveSchedules(list) { localStorage.setItem('mv2_schedules', JSON.stringify(list)); }

function renderSchedule() {
  const list = getSchedules();
  const wrap = document.getElementById('schedule-list');
  wrap.innerHTML = '';

  if (!list.length) {
    const e = document.createElement('div');
    e.className = 'empty-state';
    e.innerHTML = '<div class="empty-icon">📅</div><p>Kein Zeitplan vorhanden.<br>Füge einen Zeitplan hinzu.</p>';
    wrap.appendChild(e);
    return;
  }

  list.forEach((s, idx) => {
    const card = document.createElement('div');
    card.className = 'schedule-card';
    card.style.marginBottom = '8px';
    const durLabel = s.dur >= 60
      ? (s.dur % 60 === 0 ? (s.dur / 60) + ' h' : (s.dur / 60).toFixed(1).replace('.', ',') + ' h')
      : s.dur + ' min';
    card.innerHTML = `
      <div class="sched-top">
        <div class="sched-time-badge">${s.hour}:${s.min}</div>
        <div class="sched-days">
          ${DAYS.map(d => `<div class="day-chip ${s.days.includes(d) ? 'on' : ''}">${d}</div>`).join('')}
        </div>
        <button class="sched-delete" onclick="deleteSchedule(${idx})">
          <svg viewBox="0 0 24 24"><polyline points="3 6 5 6 21 6"/><path d="M19 6v14a2 2 0 01-2 2H7a2 2 0 01-2-2V6m3 0V4a1 1 0 011-1h4a1 1 0 011 1v2"/></svg>
        </button>
      </div>
      <div class="sched-footer">
        <div class="sched-badge">Aktiv</div>
        <div class="sched-dur">Dauer: ${durLabel}</div>
      </div>`;
    wrap.appendChild(card);
  });
}

function deleteSchedule(idx) {
  const list = getSchedules();
  list.splice(idx, 1);
  saveSchedules(list);
  renderSchedule();
  renderWeekGrid();
  updateNextMow();
  showToast('Zeitplan gelöscht', 'warning');
}

// Week grid
function renderWeekGrid() {
  const schedules = getSchedules();
  const grid = document.getElementById('week-grid');
  if (!grid) return;
  const todayIdx = (new Date().getDay() + 6) % 7; // 0=Mo
  grid.innerHTML = DAYS.map((d, i) => {
    const dayScheds = schedules.filter(s => s.days.includes(d));
    const isToday = i === todayIdx;
    return `<div class="day-col ${dayScheds.length ? 'has-sched' : ''} ${isToday ? 'today' : ''}">
      <div class="day-name">${d}</div>
      <div class="day-dot${dayScheds.length ? ' active' : ''}"></div>
      ${dayScheds.slice(0, 2).map(s => `<div class="day-time">${s.hour}</div>`).join('')}
    </div>`;
  }).join('');
}

// Next mow calculation
function updateNextMow() {
  const schedules = getSchedules();
  const timeEl = document.getElementById('next-mow-time');
  const subEl  = document.getElementById('next-mow-sub');
  if (!timeEl || !subEl) return;

  if (!schedules.length) {
    timeEl.textContent = '—';
    subEl.textContent  = 'Kein Zeitplan konfiguriert';
    return;
  }

  const now = new Date();
  const jsDay = [6, 0, 1, 2, 3, 4, 5]; // Mo=0 → js.getDay()=1
  let nearest = null;

  schedules.forEach(s => {
    s.days.forEach(d => {
      const mvIdx = DAYS.indexOf(d); // 0=Mo
      const targetJsDay = (mvIdx + 1) % 7; // 1=Mon…0=Sun
      let diff = (targetJsDay - now.getDay() + 7) % 7;
      const target = new Date(now);
      target.setDate(now.getDate() + diff);
      target.setHours(parseInt(s.hour), parseInt(s.min), 0, 0);
      if (target <= now) target.setDate(target.getDate() + 7);
      if (!nearest || target < nearest.time) nearest = { time: target, s };
    });
  });

  if (nearest) {
    const diffMs = nearest.time - now;
    const diffH  = Math.floor(diffMs / 3600000);
    const diffM  = Math.floor((diffMs % 3600000) / 60000);
    timeEl.textContent = `${nearest.s.hour}:${nearest.s.min} Uhr`;
    subEl.textContent  = diffH > 24
      ? `in ${Math.floor(diffH / 24)} Tagen`
      : diffH > 0 ? `in ${diffH} h ${diffM} min`
      : `in ${diffM} Minuten`;
  }
}

// Schedule modal
document.getElementById('add-sched-btn').addEventListener('click', openSchedModal);

(function buildHourSelect() {
  const sel = document.getElementById('m-hour');
  for (let h = 6; h <= 20; h++) {
    const o = document.createElement('option');
    o.value = String(h).padStart(2, '0');
    o.textContent = String(h).padStart(2, '0');
    if (h === 9) o.selected = true;
    sel.appendChild(o);
  }
})();

function openSchedModal() {
  document.querySelectorAll('#m-days .day-chip').forEach(c => {
    c.classList.toggle('on', ['Mo', 'Di', 'Mi', 'Do', 'Fr'].includes(c.dataset.day));
  });
  document.getElementById('sched-modal').classList.add('open');
}
function closeSchedModal() { document.getElementById('sched-modal').classList.remove('open'); }

document.querySelectorAll('#m-days .day-chip').forEach(c => {
  c.addEventListener('click', () => c.classList.toggle('on'));
});
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
  if (!days.length) { showToast('Bitte mindestens einen Tag wählen', 'warning'); return; }
  const list = getSchedules();
  list.push({ hour, min, days, dur });
  saveSchedules(list);
  closeSchedModal();
  renderSchedule();
  renderWeekGrid();
  updateNextMow();
  showToast('Zeitplan gespeichert ✓', 'success');
}

// ═══════════════════════════════════════════════════════════════
// HISTORY (localStorage)
// ═══════════════════════════════════════════════════════════════
let _mowingStart = null;
let _todayArea   = 0;

function trackSession(oldState, newState) {
  if (newState === 'MOWING') _mowingStart = Date.now();
  if ((oldState === 'MOWING' || oldState === 'RETURNING') &&
      (newState === 'IDLE' || newState === 'CHARGING' || newState === 'ERROR')) {
    if (_mowingStart) {
      const dur  = Math.max(1, Math.round((Date.now() - _mowingStart) / 60000));
      const area = Math.round(dur * 35);
      _todayArea += area;
      const el = document.getElementById('tel-area');
      if (el) el.textContent = _todayArea;
      addSession({ date: new Date().toISOString(), dur, status: newState === 'ERROR' ? 'error' : 'ok', area });
      _mowingStart = null;
    }
  }
}

function getSessions() {
  try { return JSON.parse(localStorage.getItem('mv2_sessions') || '[]'); } catch { return []; }
}
function addSession(s) {
  const list = getSessions();
  list.unshift(s);
  if (list.length > 50) list.splice(50);
  localStorage.setItem('mv2_sessions', JSON.stringify(list));
}
function addHistoryPoint() {} // hook for future live data

function renderHistory() {
  const sessions = getSessions().length ? getSessions() : demoSessions();

  // Stats row
  const totalMins = sessions.reduce((a, s) => a + (s.dur || 0), 0);
  const totalArea = sessions.reduce((a, s) => a + (s.area || 0), 0);
  document.getElementById('stats-row').innerHTML = `
    <div class="stat-card"><div class="stat-val">${sessions.length}</div><div class="stat-lbl">Sitzungen</div></div>
    <div class="stat-card"><div class="stat-val">${totalMins >= 60 ? (totalMins / 60).toFixed(1) + ' h' : totalMins + ' m'}</div><div class="stat-lbl">Laufzeit</div></div>
    <div class="stat-card"><div class="stat-val">${totalArea >= 1000 ? (totalArea / 1000).toFixed(1) + 'k' : totalArea}</div><div class="stat-lbl">m² gemäht</div></div>`;

  // Bar chart
  renderHistoryChart(sessions);

  // Session list
  const list = document.getElementById('session-list');
  list.innerHTML = sessions.map(s => {
    const d = new Date(s.date);
    const dateStr = d.toLocaleDateString('de-DE', { weekday: 'short', day: '2-digit', month: 'short' });
    const timeStr = d.toLocaleTimeString('de-DE', { hour: '2-digit', minute: '2-digit' });
    const cls  = s.status === 'error' ? 'err' : s.status === 'warn' ? 'warn' : 'ok';
    const icon = s.status === 'error' ? errSVG() : s.status === 'warn' ? warnSVG() : checkSVG();
    return `<div class="session-card">
      <div class="sess-icon ${cls}">${icon}</div>
      <div class="sess-info">
        <div class="sess-date">${dateStr}</div>
        <div class="sess-meta">${timeStr} Uhr · ${s.dur} min</div>
      </div>
      <div class="sess-right">
        <div class="sess-area">${s.area}</div>
        <div class="sess-unit">m²</div>
      </div>
    </div>`;
  }).join('');
}

function renderHistoryChart(sessions) {
  const chart = document.getElementById('session-chart');
  if (!chart) return;
  const recent  = [...sessions].slice(0, 8).reverse();
  const maxDur  = Math.max(...recent.map(s => s.dur), 1);
  chart.innerHTML = recent.map(s => {
    const h   = Math.round((s.dur / maxDur) * 100);
    const d   = new Date(s.date);
    const lbl = d.toLocaleDateString('de-DE', { day: '2-digit', month: '2-digit' });
    const barCls = s.status === 'error' ? 'bar-err' : s.status === 'warn' ? 'bar-warn' : 'bar-ok';
    return `<div class="chart-col">
      <div class="chart-bar-wrap">
        <div class="chart-bar ${barCls}" style="height:${h}%"></div>
      </div>
      <div class="chart-lbl">${lbl}</div>
    </div>`;
  }).join('');
}

function demoSessions() {
  return [
    { date: '2026-04-17T09:12:00', dur: 52, status: 'ok',   area: 1820 },
    { date: '2026-04-15T10:05:00', dur: 61, status: 'ok',   area: 2135 },
    { date: '2026-04-13T08:30:00', dur: 38, status: 'warn', area: 1330 },
    { date: '2026-04-11T09:45:00', dur: 67, status: 'ok',   area: 2345 },
    { date: '2026-04-09T08:00:00', dur: 44, status: 'ok',   area: 1540 },
    { date: '2026-04-07T10:20:00', dur: 29, status: 'error',area: 1015 },
    { date: '2026-04-05T09:30:00', dur: 58, status: 'ok',   area: 2030 },
    { date: '2026-04-03T07:50:00', dur: 71, status: 'ok',   area: 2485 },
  ];
}

function logSessionError(reason) { console.warn('Session error:', reason); }

// SVG icon helpers
function checkSVG() { return '<svg viewBox="0 0 24 24" style="width:18px;height:18px;stroke:currentColor;fill:none;stroke-width:2.5;stroke-linecap:round;stroke-linejoin:round"><polyline points="20 6 9 17 4 12"/></svg>'; }
function warnSVG()  { return '<svg viewBox="0 0 24 24" style="width:18px;height:18px;stroke:currentColor;fill:none;stroke-width:2;stroke-linecap:round"><path d="M10.29 3.86L1.82 18a2 2 0 001.71 3h16.94a2 2 0 001.71-3L13.71 3.86a2 2 0 00-3.42 0z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/></svg>'; }
function errSVG()   { return '<svg viewBox="0 0 24 24" style="width:18px;height:18px;stroke:currentColor;fill:none;stroke-width:2;stroke-linecap:round"><circle cx="12" cy="12" r="10"/><line x1="15" y1="9" x2="9" y2="15"/><line x1="9" y1="9" x2="15" y2="15"/></svg>'; }

// ═══════════════════════════════════════════════════════════════
// SETTINGS (localStorage)
// ═══════════════════════════════════════════════════════════════
function getSettings() {
  const def = { lane: 38, speed: 5, tilt: 30, rainwait: 30, rain: true, geo: true, robotName: 'MV2-Alpha' };
  try { return Object.assign(def, JSON.parse(localStorage.getItem('mv2_settings') || '{}')); }
  catch { return def; }
}
function saveSettings(s) { localStorage.setItem('mv2_settings', JSON.stringify(s)); }

function initSettings() {
  const s = getSettings();

  // Update brand subtitle
  const modelEl = document.querySelector('.brand-model');
  if (modelEl) modelEl.textContent = s.robotName;

  const bind = (id, valId, suffix, key, transform) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.value = s[key];
    const valEl = document.getElementById(valId);
    if (valEl) valEl.textContent = (transform ? transform(s[key]) : s[key]) + suffix;
    el.addEventListener('input', () => {
      const v = +el.value;
      if (valEl) valEl.textContent = (transform ? transform(v) : v) + suffix;
      saveSettings(Object.assign(getSettings(), { [key]: v }));
    });
  };

  bind('s-lane',    's-lane-val',    ' cm', 'lane');
  bind('s-speed',   's-speed-val',   '',    'speed');
  bind('s-tilt',    's-tilt-val',    '°',   'tilt');
  bind('s-rainwait','s-rainwait-val',' min','rainwait');

  const rainEl = document.getElementById('s-rain');
  if (rainEl) { rainEl.checked = s.rain; rainEl.addEventListener('change', () => saveSettings(Object.assign(getSettings(), { rain: rainEl.checked }))); }

  const geoEl = document.getElementById('s-geo');
  if (geoEl) { geoEl.checked = s.geo; geoEl.addEventListener('change', () => saveSettings(Object.assign(getSettings(), { geo: geoEl.checked }))); }

  const namePreview = document.getElementById('robot-name-preview');
  if (namePreview) namePreview.textContent = s.robotName;

  // Robot name card click
  const nameCard = document.getElementById('card-robot-name');
  if (nameCard) nameCard.addEventListener('click', openNameModal);

  const resetBtn = document.getElementById('reset-btn-settings');
  if (resetBtn) resetBtn.addEventListener('click', () => {
    apiPost('/api/reset');
    showToast('Fehler zurückgesetzt', 'info');
  });
}

function openNameModal() {
  document.getElementById('robot-name-input').value = getSettings().robotName;
  document.getElementById('name-modal').classList.add('open');
}
function closeNameModal() { document.getElementById('name-modal').classList.remove('open'); }

function saveRobotName() {
  const name = document.getElementById('robot-name-input').value.trim() || 'MV2-Alpha';
  saveSettings(Object.assign(getSettings(), { robotName: name }));
  const preview = document.getElementById('robot-name-preview');
  if (preview) preview.textContent = name;
  const model = document.querySelector('.brand-model');
  if (model) model.textContent = name;
  closeNameModal();
  showToast(`Name geändert zu „${name}"`, 'success');
}

// ═══════════════════════════════════════════════════════════════
// INIT
// ═══════════════════════════════════════════════════════════════
updateUI('IDLE');
initSettings();
renderWizDots();
renderZones();

// Show demo last session on load
const _demoDone = getSessions().length === 0;
if (_demoDone) {
  const demos = demoSessions();
  if (demos.length) showLastSession(demos[0]);
}
