// Mähroboter V2 — real control UI
'use strict';

const $ = (id) => document.getElementById(id);
const DAYS = ['Mo', 'Di', 'Mi', 'Do', 'Fr', 'Sa', 'So'];
const STATE_LABELS = {
  IDLE:'Bereit', TEACH_IN:'Teach-In', MOWING:'Mähen', PAUSED:'Pausiert',
  OBSTACLE_AVOIDANCE:'Hindernis', RETURNING:'Rückkehr', DOCKING:'Andocken',
  CHARGING:'Laden', ERROR:'Fehler',
};
const STATE_SUBS = {
  IDLE:'Bereit zum Starten', TEACH_IN:'Grenze wird aufgezeichnet…', MOWING:'Mission aktiv',
  PAUSED:'Mission sicher angehalten', OBSTACLE_AVOIDANCE:'Hindernis wird umfahren…',
  RETURNING:'Kehrt zur Basis zurück…', DOCKING:'Dockt an der Ladestation an…',
  CHARGING:'Akku wird geladen…', ERROR:'Eingriff erforderlich',
};

let appData = { zones:[], connections:[], schedules:[], settings:{}, home:null, telemetry:null, events:[], route:null, planners:{} };
let currentState = 'IDLE';
let currentErrorReason = '';
let currentPauseReason = '';
let currentGeofenceOverride = false;
let currentZoneId = null;
let authenticated = false;
let commandOnline = false;
let connectionMode = 'connecting';
let ws = null;
let reconnectTimer = null;
let currentPose = null;
let teachPoints = [];
let serverTeachGeometry = null;
let serverTeachPointCount = 0;
let teachRecording = false;
let teachStatus = null;
let teachStatusTimer = null;
let teachStatusLoading = false;
let teachMap = null, teachRobotMarker = null, teachRouteLine = null, teachReturnMarker = null, teachStartMarker = null;
let teachMapFollow = true;
let teachJoystickPointer = null;
let teachJoystickLastSend = 0;
let diagnosticJoystickPointer = null;
let diagnosticJoystickLastSend = 0;
let recTimer = null;
let recSecs = 0;
let wizStep = 1;
let map = null, robotMarker = null, homeMarker = null, trailLine = null, teachLine = null, routeLine = null, headlandLine = null, turnLine = null;
let coverageLayerGroup = null, liveCoverageLayerGroup = null, deckLayerGroup = null, truthLine = null, truthMarker = null, lastActualPose = null;
const liveCoverageTracks = new Map();
const LIVE_COVERAGE_CHUNK_POINTS = 400;
let zoneLayerGroup = null, connectionLayerGroup = null, gpsBadge = null;
let followRobot = true;
let currentLayer = 0;
let mapLayers = [];
let displayedSpeedKmh = null;
let missionStartPending = false;

function escapeHtml(value) {
  return String(value ?? '').replace(/[&<>'"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;',"'":'&#39;','"':'&quot;'}[c]));
}

function showToast(msg, type='info') {
  const stack = $('toast-stack');
  if (!stack) return;
  const t = document.createElement('div');
  t.className = `toast ${type}`;
  t.innerHTML = `<div class="toast-dot"></div><span>${escapeHtml(msg)}</span>`;
  stack.appendChild(t);
  requestAnimationFrame(() => requestAnimationFrame(() => t.classList.add('show')));
  setTimeout(() => { t.classList.remove('show'); setTimeout(() => t.remove(), 350); }, 3800);
}

function setMissionPlanning(active, zoneName='') {
  missionStartPending=Boolean(active);
  const overlay=$('mission-planning-overlay');
  if(overlay){
    overlay.classList.toggle('open',missionStartPending);
    overlay.setAttribute('aria-hidden',String(!missionStartPending));
  }
  if(missionStartPending){
    const nativePlanner=appData.settings.planner_engine==='fields2cover';
    const optimizerSeconds=Number(appData.settings.fields2cover_optimizer_time_s??2);
    const timingNote=nativePlanner&&appData.settings.fields2cover_route_order==='optimized'
      ?` Zeitlimit je Optimierungsversuch: ${optimizerSeconds} s.`:'';
    setText('mission-planning-title','Mission wird geplant…');
    setText('mission-planning-message',`${nativePlanner?'Fields2Cover':'MV2'} berechnet die sichere Route für „${zoneName||'die ausgewählte Zone'}“.${timingNote}`);
  }
  renderPrimaryStatus();
  syncCommandAvailability();
  renderZones();
}

let appDialogResolve = null;
let appDialogSelectedValue = null;
let appDialogPreviousFocus = null;

function finishAppDialog(confirmed) {
  const dialog=$('app-dialog');
  if(!dialog?.classList.contains('open'))return;
  const result=confirmed?(appDialogSelectedValue??true):null;
  dialog.classList.remove('open');
  dialog.setAttribute('aria-hidden','true');
  document.body.classList.remove('app-dialog-open');
  const resolve=appDialogResolve;
  appDialogResolve=null;appDialogSelectedValue=null;
  const previous=appDialogPreviousFocus;appDialogPreviousFocus=null;
  setTimeout(()=>previous?.focus?.({preventScroll:true}),0);
  resolve?.(result);
}

function openAppDialog({title='Aktion bestätigen',message='',icon='?',tone='warning',confirmLabel='Bestätigen',cancelLabel='Abbrechen',choices=null,cancelable=true}={}) {
  if(appDialogResolve)finishAppDialog(false);
  const dialog=$('app-dialog'),choiceWrap=$('app-dialog-choices'),confirmButton=$('app-dialog-confirm'),cancelButton=$('app-dialog-cancel');
  if(!dialog||!choiceWrap||!confirmButton||!cancelButton)return Promise.resolve(null);
  appDialogPreviousFocus=document.activeElement;
  setText('app-dialog-title',title);setText('app-dialog-message',message);setText('app-dialog-icon',icon);
  dialog.dataset.tone=tone;
  confirmButton.textContent=confirmLabel;cancelButton.textContent=cancelLabel;cancelButton.classList.toggle('hidden',!cancelable);
  appDialogSelectedValue=null;choiceWrap.innerHTML='';
  const availableChoices=Array.isArray(choices)?choices:[];
  choiceWrap.classList.toggle('hidden',!availableChoices.length);
  availableChoices.forEach((choice,index)=>{
    const button=document.createElement('button');button.type='button';button.className='app-dialog-choice';button.setAttribute('role','option');button.setAttribute('aria-selected','false');
    button.innerHTML=`<span class="app-dialog-choice-dot"></span><span class="app-dialog-choice-copy"><strong>${escapeHtml(choice.label)}</strong>${choice.meta?`<small>${escapeHtml(choice.meta)}</small>`:''}</span>`;
    button.addEventListener('click',()=>{
      appDialogSelectedValue=choice.value;
      choiceWrap.querySelectorAll('.app-dialog-choice').forEach(item=>{const selected=item===button;item.classList.toggle('selected',selected);item.setAttribute('aria-selected',String(selected));});
      confirmButton.disabled=false;
    });
    choiceWrap.appendChild(button);
    if(index===0)button.click();
  });
  confirmButton.disabled=availableChoices.length>0&&appDialogSelectedValue==null;
  dialog.setAttribute('aria-hidden','false');dialog.classList.add('open');document.body.classList.add('app-dialog-open');
  requestAnimationFrame(()=>requestAnimationFrame(()=>confirmButton.focus({preventScroll:true})));
  return new Promise(resolve=>{appDialogResolve=resolve;});
}

async function askConfirmation(options={}) { return Boolean(await openAppDialog(options)); }
async function showAppMessage(options={}) { await openAppDialog({...options,cancelable:false,confirmLabel:options.confirmLabel||'Schließen'}); }

$('app-dialog-confirm')?.addEventListener('click',()=>finishAppDialog(true));
$('app-dialog-cancel')?.addEventListener('click',()=>finishAppDialog(false));
$('app-dialog')?.addEventListener('click',event=>{if(event.target===$('app-dialog'))finishAppDialog(false);});
document.addEventListener('keydown',event=>{
  if(!$('app-dialog')?.classList.contains('open'))return;
  if(event.key==='Escape'){event.preventDefault();finishAppDialog(false);}
  else if(event.key==='Enter'&&!$('app-dialog-confirm')?.disabled){event.preventDefault();finishAppDialog(true);}
});

function apiErrorMessage(error, fallback='Unbekannter API-Fehler') {
  const detail=error?.detail;
  if(typeof detail==='string') return detail;
  if(Array.isArray(detail)) return detail.map(item=>{
    const location=Array.isArray(item?.loc)?item.loc.filter(part=>part!=='body').join('.'):'', message=item?.msg||'Ungültige Eingabe';
    return location?`${location}: ${message}`:message;
  }).join('; ');
  if(detail&&typeof detail==='object') return detail.message||JSON.stringify(detail);
  return fallback;
}

function renderPrimaryStatus() {
  if (connectionMode === 'offline') {
    setText('sc-state', 'Offline');
    setText('state-sub', 'Keine Serververbindung – Befehle gesperrt');
    return;
  }
  if (connectionMode === 'connecting') {
    setText('sc-state', 'Verbinden…');
    setText('state-sub', 'Roboterstatus wird geladen');
    return;
  }
  if (!authenticated) {
    setText('sc-state', 'Gesperrt');
    setText('state-sub', 'PIN-Anmeldung erforderlich');
    return;
  }
  const missionReason = missionBlockReason();
  if (currentState === 'IDLE' && missionReason) {
    setText('sc-state', 'Nicht startbereit');
    setText('state-sub', missionReason);
    return;
  }
  if (connectionMode === 'stale') {
    setText('sc-state', 'Daten veraltet');
    setText('state-sub', 'Server verbunden · Robotertelemetrie nicht aktuell');
    return;
  }
  setText('sc-state', STATE_LABELS[currentState] || currentState);
  const overrideNote=currentGeofenceOverride
    ?'Geofence-Ausnahme aktiv · Schutz wird beim Wiedereintritt automatisch reaktiviert':'';
  const pauseNote=currentPauseReason.replace('Geofence violation at','Geofence verletzt bei');
  setText('state-sub', overrideNote || currentErrorReason || pauseNote || STATE_SUBS[currentState] || '');
}

function missionBlockReason() {
  if (missionStartPending) return 'Mission wird geplant – bitte warten';
  if (!appData.zones.some(zone => zone.type !== 'no_go')) return 'Zuerst eine Mähzone anlegen oder per Teach-In erfassen';
  if (connectionMode === 'stale') return 'Keine aktuelle Robotertelemetrie – Mission gesperrt';
  return '';
}

function syncCommandAvailability() {
  const missionReason = missionBlockReason();
  document.querySelectorAll('[data-online-command]').forEach(el => {
    el.disabled = !commandOnline || (el.dataset.requiresMissionReady === '1' && Boolean(missionReason));
  });
  const returnHomeButton = $('global-return-home');
  if (returnHomeButton) {
    const activeMission=['MOWING','PAUSED','OBSTACLE_AVOIDANCE'].includes(currentState);
    returnHomeButton.disabled = !commandOnline || !activeMission || !appData.home;
    returnHomeButton.title = !appData.home ? 'Zuerst die Ladestation mit H auf der Karte speichern' : 'Mission beenden und zur Ladestation fahren';
  }
}

function setConnection(online, label) {
  connectionMode = online ? 'online' : label === 'Daten veraltet' ? 'stale' : 'offline';
  commandOnline = authenticated && connectionMode !== 'offline' && connectionMode !== 'connecting';
  document.documentElement.dataset.connection = connectionMode;
  const dot = $('ws-dot');
  const txt = $('top-conn-label');
  if (dot) dot.className = `ws-dot ${online ? 'on' : 'err'}`;
  if (txt) txt.textContent = connectionMode === 'stale' ? 'Verbunden · Daten veraltet' : label || (online ? 'Verbunden' : 'Getrennt');
  $('connection-banner')?.classList.toggle('show', !online && authenticated);
  setText('connection-banner-text', connectionMode === 'stale' ? 'Telemetriedaten veraltet' : 'Verbindung unterbrochen · letzte Daten');
  renderPrimaryStatus();
  syncCommandAvailability();
  renderDiagnostics();
}

async function api(path, options={}) {
  const init = {...options};
  init.headers = {...(init.headers || {})};
  if (init.body && typeof init.body !== 'string') {
    init.headers['Content-Type'] = 'application/json';
    init.body = JSON.stringify(init.body);
  }
  try {
    const response = await fetch(path, init);
    if (response.status === 401) {
      authenticated = false;
      $('login-gate')?.classList.add('open');
      throw new Error('PIN-Anmeldung erforderlich');
    }
    if (!response.ok) {
      const error = await response.json().catch(() => ({detail:response.statusText}));
      throw new Error(apiErrorMessage(error,response.statusText));
    }
    const type = response.headers.get('content-type') || '';
    return type.includes('json') ? response.json() : response.text();
  } catch (error) {
    if (error instanceof TypeError) setConnection(false, 'Offline');
    throw error;
  }
}

async function command(path, body=null, success='') {
  if (!commandOnline) { showToast('Befehl gesperrt: keine Verbindung', 'error'); return null; }
  try {
    const data = await api(path, {method:'POST', body});
    if (data?.state) updateUI(
      data.state,
      data.error_reason||'',
      data.pause_reason||'',
      Boolean(data.geofence_override_active),
    );
    if (success) showToast(success, 'success');
    return data;
  } catch (error) {
    showToast(error.message, 'error');
    return null;
  }
}

async function apiPost(path) { return command(path); }

// ----------------------------------------------------------------------
// Authentication and startup
// ----------------------------------------------------------------------
async function initAuth() {
  const status = await api('/api/auth/status').catch(() => ({required:false, authenticated:true}));
  authenticated = status.authenticated;
  $('login-gate')?.classList.toggle('open', status.required && !authenticated);
  if (authenticated) await loadBootstrap();
}

$('login-form')?.addEventListener('submit', async (event) => {
  event.preventDefault();
  try {
    await api('/api/auth/login', {method:'POST', body:{pin:$('login-pin').value}});
    authenticated = true;
    $('login-gate').classList.remove('open');
    $('login-pin').value = '';
    await loadBootstrap();
  } catch (error) { showToast(error.message, 'error'); }
});

async function loadBootstrap() {
  try {
    const data = await api('/api/bootstrap');
    Object.assign(appData, data);
    currentZoneId = data.active_zone_id;
    updateUI(
      data.state,
      data.error_reason,
      data.pause_reason,
      Boolean(data.geofence_override_active),
    );
    updateTelemetry(data.telemetry);
    renderZones();
    renderMapData();
    if(['MOWING','PAUSED','RETURNING'].includes(data.state))fitMissionRoute();
    renderSchedule();
    initSettings(true);
    initPlanningSettings();
    initTurnSettings();
    initDiagnosticManualSettings();
    renderDiagnostics();
    await loadEvents();
    await loadVersions();
    connectWS();
  } catch (error) { showToast(error.message, 'error'); }
}

// ----------------------------------------------------------------------
// Map
// ----------------------------------------------------------------------
const TRACTOR_SOURCE='/static/tractor.png';
const tractorStateAssets={down:TRACTOR_SOURCE};
let latestTractorPose={heading_deg:0},latestTractorSafety={front_deck_raised:false,rear_deck_raised:false};
let tractorVisibleWidthRatio=.836;
const tractorRenderedHeadings=new WeakMap();

function numericSetting(inputId,key,fallback){
  const input=$(inputId);
  if(input?.dataset.ready==='1'&&Number.isFinite(Number(input.value)))return Number(input.value);
  const stored=Number(appData.settings?.[key]);
  return Number.isFinite(stored)?stored:fallback;
}

function tractorIcon(){
  return L.divIcon({
    className:'smooth-tractor-icon',
    html:`<div class="map-tractor-marker"><img class="map-tractor-img" src="${TRACTOR_SOURCE}" alt="Traktor"></div>`,
    iconSize:[160,160],iconAnchor:[80,80],
  });
}

function tractorStateKey(safety={}){
  const front=!!safety.front_deck_raised,rear=!!safety.rear_deck_raised;
  return `${front?'front':''}${rear?'rear':''}`||'down';
}

function tractorMarkerSize(marker,targetMap){
  const lat=marker?.getLatLng()?.lat??48.5,zoom=targetMap?.getZoom?.()??18;
  const metresPerPixel=156543.03392*Math.cos(lat*Math.PI/180)/Math.pow(2,zoom);
  const workingWidthMetres=Math.max(.05,numericSetting('s-mowing-width','mowing_width_cm',60)/100);
  // The PNG has transparent margins. Scale the visible mower footprint, not
  // the full canvas, to the configured real mowing width.
  return Math.max(8,Math.min(150,workingWidthMetres/metresPerPixel/tractorVisibleWidthRatio));
}

function mowerDeckGeometry(){
  const total=Math.max(.3,numericSetting('s-mowing-width','mowing_width_cm',60)/100),ratio=Math.max(.35,Math.min(1,numericSetting('s-front-width','front_mower_width_percent',60)/100));
  const gap=Math.max(0,numericSetting('s-rear-gap','rear_mower_gap_cm',6)/100),rearWidth=Math.max(.05,(total-gap)/2),rearLeft=(gap+rearWidth)/2,depth=Math.max(.05,numericSetting('s-deck-depth','mower_deck_depth_cm',14)/100),wheelbase=Math.max(.1,numericSetting('s-wheelbase','vehicle_wheelbase_cm',25)/100),frontClearance=numericSetting('s-front-offset','front_mower_offset_cm',35)/100;
  return [
    {name:'front',group:'front',forward_m:wheelbase+frontClearance,clearance_m:frontClearance,left_m:0,width_m:total*ratio,depth_m:depth},
    {name:'rear_left',group:'rear',forward_m:-numericSetting('s-rear-offset','rear_mower_offset_cm',22)/100,left_m:rearLeft,width_m:rearWidth,depth_m:depth},
    {name:'rear_right',group:'rear',forward_m:-numericSetting('s-rear-offset','rear_mower_offset_cm',22)/100,left_m:-rearLeft,width_m:rearWidth,depth_m:depth},
  ];
}

function localOffsetLatLng(pose,forward,left){
  const heading=(Number(pose.heading_deg)||0)*Math.PI/180,latRad=Number(pose.lat)*Math.PI/180;
  const north=forward*Math.cos(heading)+left*Math.sin(heading),east=forward*Math.sin(heading)-left*Math.cos(heading);
  return [Number(pose.lat)+north/111320,Number(pose.lon)+east/(111320*Math.max(.1,Math.cos(latRad)))];
}

function deckLatLngCorners(pose,deck){
  return [[1,1],[1,-1],[-1,-1],[-1,1]].map(([along,across])=>localOffsetLatLng(pose,deck.forward_m+along*deck.depth_m/2,deck.left_m+across*deck.width_m/2));
}

function midpointLatLng(first,second){
  return [(first[0]+second[0])/2,(first[1]+second[1])/2];
}

function latLngDistanceMetres(first,second){
  const meanLat=(first[0]+second[0])*Math.PI/360;
  const north=(second[0]-first[0])*111320,east=(second[1]-first[1])*111320*Math.cos(meanLat);
  return Math.hypot(north,east);
}

function clearLiveCoverage(){
  liveCoverageLayerGroup?.clearLayers();
  liveCoverageTracks.clear();
}

function clearMissionLines(){
  appData.route=null;
  routeLine?.setLatLngs([]);headlandLine?.setLatLngs([]);turnLine?.setLatLngs([]);
  trailLine?.setLatLngs([]);truthLine?.setLatLngs([]);
  coverageLayerGroup?.clearLayers();clearLiveCoverage();lastActualPose=null;
}

function startLiveCoverageTrack(deck,corners,color){
  const layer=L.polygon(corners,{className:`live-coverage-shape live-coverage-${deck.name}`,stroke:false,fillColor:color,fillOpacity:.28,fillRule:'nonzero',interactive:false}).addTo(liveCoverageLayerGroup);
  const track={layer,startCorners:corners,left:[],right:[],lastCenter:null};
  liveCoverageTracks.set(deck.name,track);
  return track;
}

function updateLiveCoverage(pose,outputs,missionMoving){
  if(!liveCoverageLayerGroup||pose?.lat==null||pose?.lon==null)return;
  const colors={front:'#55d98b',rear_left:'#39bf73',rear_right:'#2fa965'};
  mowerDeckGeometry().forEach(deck=>{
    const raised=deck.group==='front'?!!outputs.front_deck_raised:!!outputs.rear_deck_raised;
    const active=missionMoving&&currentState==='MOWING'&&outputs.blade_enabled===true&&!raised;
    const corners=deckLatLngCorners(pose,deck);
    if(active){
      const left=midpointLatLng(corners[0],corners[3]),right=midpointLatLng(corners[1],corners[2]);
      const center=midpointLatLng(left,right);
      let track=liveCoverageTracks.get(deck.name);
      if(!track||track.left.length>=LIVE_COVERAGE_CHUNK_POINTS)track=startLiveCoverageTrack(deck,corners,colors[deck.name]);
      if(track.lastCenter&&latLngDistanceMetres(track.lastCenter,center)<.002)return;
      track.left.push(left);track.right.push(right);track.lastCenter=center;
      const polygon=[
        track.startCorners[3],track.startCorners[0],
        ...track.left,corners[0],corners[1],
        ...track.right.slice().reverse(),
        track.startCorners[2],
      ];
      track.layer.setLatLngs(polygon);
    }else liveCoverageTracks.delete(deck.name);
  });
}

function updateDeckFootprints(pose,outputs={}){
  if(!deckLayerGroup||pose?.lat==null||pose?.lon==null)return;
  deckLayerGroup.clearLayers();
  const visibilityToggle=$('s-mower-footprints');
  const visible=visibilityToggle?.dataset.ready==='1'
    ? visibilityToggle.checked
    : appData.settings?.show_mower_footprints!==false;
  if(!visible)return;
  mowerDeckGeometry().forEach(deck=>{
    const raised=deck.group==='front'?!!outputs.front_deck_raised:!!outputs.rear_deck_raised;
    const color=raised?'#48a9ff':'#f5bd3e',opacity=(outputs.blade_enabled&&!raised) ? .72 : .38;
    const corners=deckLatLngCorners(pose,deck);
    L.polygon(corners,{className:`deck-footprint deck-footprint-${deck.name}`,color,fillColor:color,fillOpacity:opacity,weight:1.5,interactive:false}).addTo(deckLayerGroup);
  });
}

function updateTractorMarker(marker,pose=latestTractorPose,safety=latestTractorSafety,targetMap=map){
  const root=marker?.getElement()?.querySelector('.map-tractor-marker');
  const img=root?.querySelector('.map-tractor-img');
  if(!root||!img)return;
  const rawHeading=(Number(pose?.heading_deg)||0)+180; // source PNG points downwards
  const previousHeading=tractorRenderedHeadings.get(marker);
  const heading=previousHeading==null
    ? rawHeading
    : previousHeading+((rawHeading-previousHeading+540)%360-180);
  tractorRenderedHeadings.set(marker,heading);
  const size=tractorMarkerSize(marker,targetMap);
  root.style.setProperty('--tractor-heading',`${heading}deg`);
  root.style.setProperty('--tractor-size',`${size.toFixed(1)}px`);
  const key=tractorStateKey(safety);
  img.src=tractorStateAssets[key]||TRACTOR_SOURCE;
  img.alt=`Traktor · Frontmähwerk ${safety.front_deck_raised?'oben':'unten'} · Heckmähwerk ${safety.rear_deck_raised?'oben':'unten'}`;
}

function buildTractorStateAssets(){
  const source=new Image();
  source.onload=()=>{
    const measure=document.createElement('canvas');measure.width=source.naturalWidth;measure.height=source.naturalHeight;
    const measureContext=measure.getContext('2d');measureContext.drawImage(source,0,0);
    const sourcePixels=measureContext.getImageData(0,0,measure.width,measure.height).data;
    let minVisibleX=measure.width,maxVisibleX=-1;
    for(let index=0;index<sourcePixels.length;index+=4){
      if(sourcePixels[index+3]>16){const x=(index/4)%measure.width;minVisibleX=Math.min(minVisibleX,x);maxVisibleX=Math.max(maxVisibleX,x);}
    }
    if(maxVisibleX>=minVisibleX)tractorVisibleWidthRatio=(maxVisibleX-minVisibleX+1)/measure.width;
    ['front','rear','frontrear'].forEach(key=>{
      const canvas=document.createElement('canvas');canvas.width=source.naturalWidth;canvas.height=source.naturalHeight;
      const ctx=canvas.getContext('2d');ctx.drawImage(source,0,0);
      const pixels=ctx.getImageData(0,0,canvas.width,canvas.height),data=pixels.data;
      const frontRaised=key.includes('front'),rearRaised=key.includes('rear');
      for(let y=0;y<canvas.height;y++)for(let x=0;x<canvas.width;x++){
        const index=(y*canvas.width+x)*4,r=data[index],g=data[index+1],b=data[index+2];
        const mowerYellow=r>160&&g>75&&g<215&&b<105;
        const raised=(y>canvas.height*.54&&frontRaised)||(y<=canvas.height*.54&&rearRaised);
        if(mowerYellow&&raised){
          const light=Math.max(0,Math.min(1,(r+g-230)/220));
          data[index]=Math.round(45+35*light);data[index+1]=Math.round(145+55*light);data[index+2]=Math.round(215+40*light);
        }
      }
      ctx.putImageData(pixels,0,0);tractorStateAssets[key]=canvas.toDataURL('image/png');
    });
    updateTractorMarker(robotMarker);updateTractorMarker(teachRobotMarker,latestTractorPose,latestTractorSafety,teachMap);renderLaneViz(numericSetting('s-lane','lane_width_cm',38));
  };
  source.src=TRACTOR_SOURCE;
}

function initMap() {
  gpsBadge = $('gps-map-badge');
  if (!window.L) {
    $('map').innerHTML = '<div style="padding:90px 24px;color:#fff">Kartenbibliothek offline noch nicht im Cache. Steuerung und Diagnose bleiben verfügbar.</div>';
    return;
  }
  map = L.map('map', {zoomControl:false, attributionControl:true, maxZoom:23}).setView([48.5, 11.0], 17);
  map.attributionControl.setPrefix(false);
  L.control.zoom({position:'bottomleft'}).addTo(map);
  mapLayers = [
    L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {attribution:'© Esri', maxZoom:23, maxNativeZoom:19}),
    L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {attribution:'© CARTO © OSM', subdomains:'abcd', maxZoom:23, maxNativeZoom:20}),
    L.tileLayer('/tiles/{z}/{x}/{y}.png', {attribution:'Lokale Offline-Karte', maxZoom:23, maxNativeZoom:21}),
  ];
  mapLayers[0].addTo(map);
  robotMarker = L.marker([48.5,11.0], {icon:tractorIcon(),interactive:false,zIndexOffset:900}).addTo(map);
  coverageLayerGroup = L.layerGroup().addTo(map);
  liveCoverageLayerGroup = L.layerGroup().addTo(map);
  deckLayerGroup = L.layerGroup().addTo(map);
  truthLine = L.polyline([], {color:'#ffd35a',weight:2,opacity:.88,dashArray:'3 5',interactive:false}).addTo(map);
  truthMarker = L.circleMarker([48.5,11.0],{radius:3,color:'#ffd35a',fillColor:'#ffd35a',fillOpacity:0,opacity:0,weight:1,interactive:false}).addTo(map);
  trailLine = L.polyline([], {color:'#f4fbff', weight:2.5, opacity:.9, lineCap:'round', lineJoin:'round'}).addTo(map);
  routeLine = L.polyline([], {color:'#43c7ff', weight:3, opacity:.92, lineCap:'round', lineJoin:'round'}).addTo(map);
  headlandLine = L.polyline([], {color:'#42d6a4', weight:3.5, opacity:.96, lineCap:'round', lineJoin:'round'}).addTo(map);
  turnLine = L.polyline([], {color:'#b892ff', weight:3, opacity:.9, dashArray:'7 6', lineCap:'round', lineJoin:'round'}).addTo(map);
  teachLine = L.polyline([], {color:'#a784f2', weight:4, dashArray:'7 7'}).addTo(map);
  zoneLayerGroup = L.layerGroup().addTo(map);
  connectionLayerGroup = L.layerGroup().addTo(map);
  map.on('dragstart zoomstart', () => setFollow(false));
  map.on('zoomend',()=>updateTractorMarker(robotMarker));
  buildTractorStateAssets();
}

function setFollow(on) {
  followRobot = on;
  $('map-follow-btn')?.classList.toggle('map-follow-on', on);
}

$('map-center-btn')?.addEventListener('click', () => {
  if (robotMarker && map) { map.setView(robotMarker.getLatLng(), Math.max(22,map.getZoom())); setFollow(true); }
});
$('map-follow-btn')?.addEventListener('click', () => setFollow(!followRobot));
$('map-layer-btn')?.addEventListener('click', () => {
  if (!map) return;
  map.removeLayer(mapLayers[currentLayer]);
  currentLayer = (currentLayer + 1) % mapLayers.length;
  mapLayers[currentLayer].addTo(map);
  showToast(['Satellitenkarte','Straßenkarte','Lokale Offline-Karte'][currentLayer], 'info');
});
$('map-home-btn')?.addEventListener('click', async () => {
  if (!currentPose?.lat || !currentPose?.lon) return showToast('Noch keine GPS-Position verfügbar', 'warning');
  if (!await askConfirmation({title:'Ladestation speichern?',message:'Die aktuelle Roboterposition wird als neue Home-Position gespeichert.',icon:'⌂',confirmLabel:'Position speichern'})) return;
  try {
    appData.home = await api('/api/home', {method:'PUT', body:{lat:currentPose.lat,lon:currentPose.lon,name:'Ladestation',heading_deg:currentPose.heading_deg}});
    renderMapData();
    syncCommandAvailability();
    showToast('Ladestation gespeichert', 'success');
  } catch (error) { showToast(error.message, 'error'); }
});

function renderMapData({replaceLiveCoverage=false}={}) {
  if (!map || !zoneLayerGroup) return;
  const routeCoordinates=appData.route?.geometry?.coordinates;
  const mowCoordinates=appData.route?.mow_geometry?.coordinates;
  const returning=appData.route?.properties?.mode==='return_home';
  routeLine?.setLatLngs(
    returning
      ? (Array.isArray(routeCoordinates)?routeCoordinates.map(([lon,lat])=>[lat,lon]):[])
      : (Array.isArray(mowCoordinates)?mowCoordinates.map(line=>line.map(([lon,lat])=>[lat,lon])):[])
  );
  const turnCoordinates=appData.route?.turn_geometry?.coordinates;
  turnLine?.setLatLngs(Array.isArray(turnCoordinates)?turnCoordinates.map(line=>line.map(([lon,lat])=>[lat,lon])):[]);
  const headlandCoordinates=appData.route?.headland_geometry?.coordinates;
  headlandLine?.setLatLngs(!returning&&Array.isArray(headlandCoordinates)?headlandCoordinates.map(line=>line.map(([lon,lat])=>[lat,lon])):[]);
  const actualCoordinates=appData.route?.actual_geometry?.coordinates;
  if(Array.isArray(actualCoordinates)&&actualCoordinates.length){
    const actualLatLngs=actualCoordinates.map(([lon,lat])=>[lat,lon]);
    trailLine?.setLatLngs(actualLatLngs);
    lastActualPose=actualLatLngs[actualLatLngs.length-1];
  }
  coverageLayerGroup?.clearLayers();
  const coverage=appData.route?.actual_coverage;
  if(coverageLayerGroup&&coverage?.type==='FeatureCollection'){
    L.geoJSON(coverage,{style:feature=>({className:`server-coverage-shape server-coverage-${feature?.properties?.deck||'deck'}`,color:feature?.properties?.color||'#42d66f',fillColor:feature?.properties?.color||'#42d66f',fillOpacity:.28,opacity:.18,weight:.5,interactive:false})}).addTo(coverageLayerGroup);
  }
  if(replaceLiveCoverage)clearLiveCoverage();
  routeLine?.setStyle({color:returning?'#87adff':'#43c7ff',weight:returning?4:3,dashArray:null,opacity:.92});
  zoneLayerGroup.clearLayers();
  appData.zones.forEach((zone) => {
    const coords = zone.geometry?.coordinates?.[0];
    if (!Array.isArray(coords) || coords.length < 3) return;
    const latlngs = coords.map(([lon,lat]) => [lat,lon]);
    const noGo = zone.type === 'no_go';
    L.polygon(latlngs, {color:noGo?'#f2565b':'#3fc66b', fillColor:noGo?'#f2565b':'#3fc66b', fillOpacity:.16, weight:3, dashArray:noGo?'6 5':null})
      .bindTooltip(`${noGo?'No-Go':'Mähzone'}: ${escapeHtml(zone.name)}`).addTo(zoneLayerGroup);
  });
  connectionLayerGroup?.clearLayers();
  (appData.connections||[]).filter(connection=>connection.enabled!==false).forEach(connection=>{
    const coords=connection.geometry?.coordinates;
    if(!Array.isArray(coords)||coords.length<2)return;
    const dockLink=connection.type==='dock_link';
    L.polyline(coords.map(([lon,lat])=>[lat,lon]),{
      color:dockLink?'#ffc14d':'#45a8ff',weight:4,opacity:.9,dashArray:'9 7',lineCap:'round',lineJoin:'round',
    }).bindTooltip(`${dockLink?'Stationsweg':'Zonenverbindung'}: ${escapeHtml(connection.name)}`).addTo(connectionLayerGroup);
  });
  routeLine?.bringToFront();
  headlandLine?.bringToFront();
  turnLine?.bringToFront();
  connectionLayerGroup?.eachLayer(layer=>layer.bringToFront?.());
  trailLine?.bringToFront();
  truthLine?.bringToFront();
  deckLayerGroup?.eachLayer(layer=>layer.bringToFront?.());
  if (homeMarker) map.removeLayer(homeMarker);
  if (appData.home?.lat != null) {
    homeMarker = L.marker([appData.home.lat, appData.home.lon], {icon:L.divIcon({className:'',html:'<div style="font-size:27px;filter:drop-shadow(0 2px 4px #000)">⌂</div>',iconSize:[30,30],iconAnchor:[15,15]})})
      .bindTooltip(appData.home.name || 'Ladestation').addTo(map);
  }
}

function fitMissionRoute(){
  const coordinates=appData.route?.geometry?.coordinates;
  if(!map||!Array.isArray(coordinates)||coordinates.length<2)return;
  const bounds=L.latLngBounds(coordinates.map(([lon,lat])=>[lat,lon]));
  if(!bounds.isValid())return;
  map.fitBounds(bounds,{padding:[45,45],maxZoom:22,animate:false});
  setFollow(true);
}

// ----------------------------------------------------------------------
// Roborock-style working map editor
// ----------------------------------------------------------------------
let mapEditorMap=null,mapEditorFeatureGroup=null,mapEditorVertexGroup=null,mapEditorDraftGroup=null;
let mapEditorObjects=[],mapEditorSelected=null,mapEditorTool='select',mapEditorDraftPoints=[],mapEditorDirty=false;

function cloneMapValue(value){return JSON.parse(JSON.stringify(value));}
function mapEditorKey(object){return `${object._kind}:${object.id}`;}
function mapEditorColor(object){
  if(object._kind==='home')return '#ffc14d';
  if(object._kind==='connection')return object.type==='dock_link'?'#ffc14d':'#45a8ff';
  return object.type==='no_go'?'#f2565b':'#3fc66b';
}
function mapEditorTypeLabel(object){
  if(object._kind==='home')return 'Ladestation';
  if(object._kind==='connection')return object.type==='dock_link'?'Stationsweg':'Zonenverbindung';
  return object.type==='no_go'?'No-Go-Zone':'Mähzone';
}
function loadMapEditorObjects(){
  mapEditorObjects=[
    ...cloneMapValue(appData.zones||[]).map(object=>({...object,_kind:'zone'})),
    ...cloneMapValue(appData.connections||[]).map(object=>({...object,_kind:'connection'})),
  ];
  if(appData.home)mapEditorObjects.push({...cloneMapValue(appData.home),id:'home',_kind:'home'});
  mapEditorSelected=null;mapEditorDirty=false;mapEditorDraftPoints=[];
}
function initMapEditorMap(){
  if(mapEditorMap||!window.L)return;
  mapEditorMap=L.map('map-editor-map',{zoomControl:false,attributionControl:true,maxZoom:23,doubleClickZoom:false}).setView(currentPose?[currentPose.lat,currentPose.lon]:[48.5,11],19);
  mapEditorMap.attributionControl.setPrefix(false);
  const editorTiles=[
    ['https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}','© Esri',19],
    ['https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png','© CARTO © OSM',20],
    ['/tiles/{z}/{x}/{y}.png','Lokale Offline-Karte',21],
  ][currentLayer]||[];
  L.tileLayer(editorTiles[0],{attribution:editorTiles[1],maxZoom:23,maxNativeZoom:editorTiles[2],subdomains:'abcd'}).addTo(mapEditorMap);
  L.control.zoom({position:'bottomleft'}).addTo(mapEditorMap);
  mapEditorFeatureGroup=L.layerGroup().addTo(mapEditorMap);
  mapEditorDraftGroup=L.layerGroup().addTo(mapEditorMap);
  mapEditorVertexGroup=L.layerGroup().addTo(mapEditorMap);
  mapEditorMap.on('click',event=>handleMapEditorClick(event.latlng));
}
function openMapEditor(selectKey=null){
  if(!window.L)return showToast('Kartenbibliothek ist nicht verfügbar','error');
  loadMapEditorObjects();
  $('map-editor-modal')?.classList.add('open');
  initMapEditorMap();
  setMapEditorTool('select',false);
  if(selectKey)mapEditorSelected=mapEditorObjects.find(object=>mapEditorKey(object)===selectKey)||null;
  renderMapEditorLayers();renderMapEditorObjectList();renderMapEditorForm();
  setTimeout(()=>{mapEditorMap?.invalidateSize();selectKey?fitMapEditorSelection():fitMapEditorAll();},80);
  refreshMapEditorTopology();
}
async function closeMapEditor(){
  if(mapEditorDirty&&!await askConfirmation({title:'Änderungen verwerfen?',message:'Die noch nicht gespeicherten Änderungen an der Arbeitskarte gehen verloren.',icon:'!',tone:'danger',confirmLabel:'Verwerfen'}))return;
  mapEditorCancelDraft(false);$('map-editor-modal')?.classList.remove('open');
}
function setMapEditorHint(message){setText('map-editor-hint',message);}
function setMapEditorTool(tool,clearDraft=true){
  if(clearDraft){mapEditorDraftPoints=[];mapEditorDraftGroup?.clearLayers();}
  mapEditorTool=tool;
  if(tool!=='select'){
    mapEditorSelected=null;
    renderMapEditorLayers();
    renderMapEditorObjectList();
    renderMapEditorForm();
  }
  document.querySelectorAll('[data-map-tool]').forEach(button=>button.classList.toggle('active',button.dataset.mapTool===tool));
  $('map-editor-draw-actions')?.classList.toggle('hidden',!['mowing','no_go','connection'].includes(tool));
  if(mapEditorMap)mapEditorMap.getContainer().style.cursor=tool==='select'?'grab':'crosshair';
  const hints={select:'Objekt anklicken, um Namen oder Eckpunkte zu bearbeiten',mowing:'Eckpunkte der Mähzone nacheinander setzen',no_go:'Eckpunkte der No-Go-Zone nacheinander setzen',connection:'Verlauf des befahrbaren Verbindungswegs setzen',home:'Position der Ladestation auf der Karte anklicken'};
  setMapEditorHint(hints[tool]||'Karte bearbeiten');
}
function handleMapEditorClick(latlng){
  if(mapEditorTool==='select')return;
  if(mapEditorTool==='home'){
    let home=mapEditorObjects.find(object=>object._kind==='home');
    if(home){home.lat=latlng.lat;home.lon=latlng.lng;}
    else{home={id:'home',_kind:'home',name:'Ladestation',lat:latlng.lat,lon:latlng.lng,heading_deg:currentPose?.heading_deg??0,_new:true};mapEditorObjects.push(home);}
    mapEditorSelected=home;mapEditorDirty=true;setMapEditorTool('select');renderMapEditorLayers();renderMapEditorObjectList();renderMapEditorForm();return;
  }
  mapEditorDraftPoints.push([latlng.lng,latlng.lat]);renderMapEditorDraft();
  setMapEditorHint(`${mapEditorDraftPoints.length} Punkt${mapEditorDraftPoints.length===1?'':'e'} gesetzt`);
}
function renderMapEditorDraft(){
  mapEditorDraftGroup?.clearLayers();if(!mapEditorDraftPoints.length)return;
  const latlngs=mapEditorDraftPoints.map(([lon,lat])=>[lat,lon]);
  L.polyline(latlngs,{color:mapEditorTool==='no_go'?'#f2565b':mapEditorTool==='connection'?'#45a8ff':'#3fc66b',weight:4,dashArray:'7 5'}).addTo(mapEditorDraftGroup);
  latlngs.forEach(point=>L.marker(point,{icon:L.divIcon({className:'map-editor-draft-vertex',iconSize:[9,9]})}).addTo(mapEditorDraftGroup));
}
function mapEditorUndoPoint(){mapEditorDraftPoints.pop();renderMapEditorDraft();setMapEditorHint(`${mapEditorDraftPoints.length} Punkte gesetzt`);}
function mapEditorCancelDraft(select=true){mapEditorDraftPoints=[];mapEditorDraftGroup?.clearLayers();if(select)setMapEditorTool('select',false);}
function mapEditorFinishDraft(){
  const polygon=mapEditorTool==='mowing'||mapEditorTool==='no_go',minimum=polygon?3:2;
  if(mapEditorDraftPoints.length<minimum)return showToast(`Mindestens ${minimum} Punkte setzen`,'warning');
  const id=`draft-${Date.now()}`,number=mapEditorObjects.filter(object=>object._kind===(polygon?'zone':'connection')).length+1;
  let object;
  if(polygon){
    const ring=[...mapEditorDraftPoints,cloneMapValue(mapEditorDraftPoints[0])];
    object={id,_kind:'zone',_new:true,name:`${mapEditorTool==='no_go'?'Sperrbereich':'Mähfläche'} ${number}`,type:mapEditorTool,enabled:true,geometry:{type:'Polygon',coordinates:[ring]}};
  }else{
    const zones=mapEditorObjects.filter(item=>item._kind==='zone'&&item.type!=='no_go');
    const hasHome=mapEditorObjects.some(item=>item._kind==='home');
    object={id,_kind:'connection',_new:true,name:`Verbindungsweg ${number}`,type:hasHome?'dock_link':'zone_link',from_zone_id:hasHome?null:zones[0]?.id,to_zone_id:(hasHome?zones[0]:zones[1])?.id,corridor_width_cm:180,bidirectional:true,enabled:true,geometry:{type:'LineString',coordinates:cloneMapValue(mapEditorDraftPoints)}};
  }
  mapEditorObjects.push(object);mapEditorSelected=object;mapEditorDirty=true;mapEditorDraftPoints=[];mapEditorDraftGroup?.clearLayers();setMapEditorTool('select',false);renderMapEditorLayers();renderMapEditorObjectList();renderMapEditorForm();fitMapEditorSelection();
}
function renderMapEditorLayers(){
  if(!mapEditorMap||!mapEditorFeatureGroup)return;
  mapEditorFeatureGroup.clearLayers();mapEditorVertexGroup?.clearLayers();
  mapEditorObjects.forEach(object=>{
    const selected=object===mapEditorSelected,color=mapEditorColor(object);let layer;
    if(object._kind==='zone'){
      const coords=object.geometry?.coordinates?.[0];if(!Array.isArray(coords)||coords.length<3)return;
      layer=L.polygon(coords.map(([lon,lat])=>[lat,lon]),{color,fillColor:color,fillOpacity:selected ? .22 : .13,weight:selected?5:3,dashArray:object.type==='no_go'?'7 5':null});
    }else if(object._kind==='connection'){
      const coords=object.geometry?.coordinates;if(!Array.isArray(coords)||coords.length<2)return;
      layer=L.polyline(coords.map(([lon,lat])=>[lat,lon]),{color,weight:selected?7:4,opacity:.92,dashArray:'10 7',lineCap:'round',lineJoin:'round'});
    }else{
      layer=L.marker([object.lat,object.lon],{draggable:selected,icon:L.divIcon({className:'',html:'<div class="map-editor-home-icon">⌂</div>',iconSize:[34,34],iconAnchor:[17,17]})});
      if(selected)layer.on('dragend',event=>{const point=event.target.getLatLng();object.lat=point.lat;object.lon=point.lng;mapEditorDirty=true;renderMapEditorLayers();});
    }
    layer.on('click',event=>{
      L.DomEvent.stopPropagation(event);
      if(mapEditorTool==='select')selectMapEditorObject(mapEditorKey(object));
      else handleMapEditorClick(event.latlng);
    });
    layer.bindTooltip(`${mapEditorTypeLabel(object)}: ${escapeHtml(object.name||'Ohne Name')}`);layer.addTo(mapEditorFeatureGroup);object._layer=layer;
  });
  renderMapEditorVertices();
}
function mapEditorEditableCoordinates(object){
  if(object?._kind==='zone')return object.geometry.coordinates[0].slice(0,-1);
  if(object?._kind==='connection')return object.geometry.coordinates;
  return [];
}
function setMapEditorCoordinates(object,coordinates){
  if(object._kind==='zone')object.geometry.coordinates[0]=[...coordinates,cloneMapValue(coordinates[0])];
  else object.geometry.coordinates=coordinates;
}
function renderMapEditorVertices(){
  if(mapEditorTool!=='select')return;
  const object=mapEditorSelected,coordinates=mapEditorEditableCoordinates(object);if(!coordinates.length)return;
  coordinates.forEach((coordinate,index)=>{
    const marker=L.marker([coordinate[1],coordinate[0]],{draggable:true,zIndexOffset:900,icon:L.divIcon({className:'map-editor-vertex',iconSize:[13,13]})}).addTo(mapEditorVertexGroup);
    marker.bindTooltip('Ziehen · Doppelklick entfernt Punkt');
    marker.on('dragend',event=>{const point=event.target.getLatLng(),updated=mapEditorEditableCoordinates(object);updated[index]=[point.lng,point.lat];setMapEditorCoordinates(object,updated);mapEditorDirty=true;renderMapEditorLayers();});
    marker.on('dblclick',event=>{L.DomEvent.stopPropagation(event);const updated=mapEditorEditableCoordinates(object),minimum=object._kind==='zone'?3:2;if(updated.length<=minimum)return showToast(`Mindestens ${minimum} Punkte erforderlich`,'warning');updated.splice(index,1);setMapEditorCoordinates(object,updated);mapEditorDirty=true;renderMapEditorLayers();});
  });
}
function selectMapEditorObject(key){
  mapEditorSelected=mapEditorObjects.find(object=>mapEditorKey(object)===key)||null;setMapEditorTool('select');renderMapEditorLayers();renderMapEditorObjectList();renderMapEditorForm();
}
function renderMapEditorObjectList(){
  const list=$('map-editor-object-list');if(!list)return;
  if(!mapEditorObjects.length){list.innerHTML='<div class="zone-empty"><p>Die Arbeitskarte ist noch leer.</p></div>';return;}
  const order={home:0,zone:1,connection:2};
  list.innerHTML=[...mapEditorObjects].sort((a,b)=>order[a._kind]-order[b._kind]).map(object=>`<button type="button" class="map-editor-list-item${object===mapEditorSelected?' active':''}" data-map-object="${escapeHtml(mapEditorKey(object))}" style="--object-color:${mapEditorColor(object)}"><span class="map-editor-list-dot"></span><span><strong>${escapeHtml(object.name||'Ohne Name')}</strong><small>${mapEditorTypeLabel(object)}</small></span><em>›</em></button>`).join('');
  list.querySelectorAll('[data-map-object]').forEach(button=>button.addEventListener('click',()=>selectMapEditorObject(button.dataset.mapObject)));
}
function populateMapEditorZoneSelects(){
  const zones=mapEditorObjects.filter(object=>object._kind==='zone'&&object.type!=='no_go'&&!object._new);
  const options=zones.map(zone=>`<option value="${zone.id}">${escapeHtml(zone.name)}</option>`).join('');
  $('map-editor-from-zone').innerHTML=options;$('map-editor-to-zone').innerHTML=options;
}
function renderMapEditorForm(){
  const object=mapEditorSelected,form=$('map-editor-object-form');form?.classList.toggle('hidden',!object);if(!object)return;
  populateMapEditorZoneSelects();setText('map-editor-object-icon',object._kind==='home'?'⌂':object._kind==='connection'?'∿':'▱');setText('map-editor-object-title',mapEditorTypeLabel(object));
  $('map-editor-name').value=object.name||'';
  $('map-editor-zone-fields').classList.toggle('hidden',object._kind!=='zone');
  $('map-editor-connection-fields').classList.toggle('hidden',object._kind!=='connection');
  if(object._kind==='zone')$('map-editor-zone-type').value=object.type;
  if(object._kind==='connection'){
    $('map-editor-connection-type').value=object.type;$('map-editor-from-zone').classList.toggle('hidden',object.type==='dock_link');
    if(object.from_zone_id)$('map-editor-from-zone').value=object.from_zone_id;if(object.to_zone_id)$('map-editor-to-zone').value=object.to_zone_id;
    $('map-editor-corridor-width').value=object.corridor_width_cm||180;$('map-editor-bidirectional').checked=object.bidirectional!==false;
  }
  $('map-editor-delete').textContent=object._new?'Entwurf verwerfen':'Löschen';
}
function nearestZoneBoundaryPoint(point,zone){
  const ring=zone?.geometry?.coordinates?.[0];if(!Array.isArray(ring)||ring.length<2)return point;
  let best=point,bestDistance=Infinity;
  for(let i=0;i<ring.length-1;i++){
    const a=ring[i],b=ring[i+1],dx=b[0]-a[0],dy=b[1]-a[1],length=dx*dx+dy*dy||1;
    const t=Math.max(0,Math.min(1,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length)),candidate=[a[0]+t*dx,a[1]+t*dy],distance=(candidate[0]-point[0])**2+(candidate[1]-point[1])**2;
    if(distance<bestDistance){bestDistance=distance;best=candidate;}
  }
  return best;
}
function snapMapEditorConnection(object,payload){
  const coordinates=cloneMapValue(object.geometry.coordinates),zones=appData.zones||[];
  if(payload.type==='dock_link'){
    if(!appData.home&&!mapEditorObjects.some(item=>item._kind==='home'&&!item._new))throw new Error('Zuerst die Ladestation speichern');
    const home=mapEditorObjects.find(item=>item._kind==='home');coordinates[0]=[home.lon,home.lat];
  }else coordinates[0]=nearestZoneBoundaryPoint(coordinates[0],zones.find(zone=>zone.id===payload.from_zone_id));
  coordinates[coordinates.length-1]=nearestZoneBoundaryPoint(coordinates.at(-1),zones.find(zone=>zone.id===payload.to_zone_id));
  object.geometry.coordinates=coordinates;payload.geometry=cloneMapValue(object.geometry);
}
async function saveMapEditorObject(){
  const object=mapEditorSelected;if(!object)return;
  try{
    const name=$('map-editor-name').value.trim();if(!name)throw new Error('Bitte einen Namen eingeben');
    let saved;
    if(object._kind==='home'){
      saved=await api('/api/home',{method:'PUT',body:{lat:object.lat,lon:object.lon,name,heading_deg:object.heading_deg??null}});appData.home=saved;object.id='home';object._new=false;Object.assign(object,saved);
    }else if(object._kind==='zone'){
      const payload={name,type:$('map-editor-zone-type').value,enabled:object.enabled!==false,geometry:cloneMapValue(object.geometry)};
      saved=await api(object._new?'/api/zones':`/api/zones/${object.id}`,{method:object._new?'POST':'PUT',body:payload});
      if(object._new)appData.zones.push(saved);else appData.zones=appData.zones.map(item=>item.id===object.id?saved:item);Object.assign(object,saved,{_kind:'zone',_new:false});
    }else{
      const type=$('map-editor-connection-type').value,payload={name,type,from_zone_id:type==='dock_link'?null:$('map-editor-from-zone').value,to_zone_id:$('map-editor-to-zone').value,corridor_width_cm:Number($('map-editor-corridor-width').value),bidirectional:$('map-editor-bidirectional').checked,enabled:object.enabled!==false,geometry:cloneMapValue(object.geometry)};
      snapMapEditorConnection(object,payload);
      saved=await api(object._new?'/api/connections':`/api/connections/${object.id}`,{method:object._new?'POST':'PUT',body:payload});
      if(object._new)appData.connections.push(saved);else appData.connections=appData.connections.map(item=>item.id===object.id?saved:item);Object.assign(object,saved,{_kind:'connection',_new:false});
    }
    mapEditorDirty=false;renderMapData();renderZones();renderMapEditorLayers();renderMapEditorObjectList();renderMapEditorForm();await refreshMapEditorTopology();showToast('Arbeitskarte gespeichert','success');
  }catch(error){showToast(error.message,'error');}
}
async function deleteMapEditorObject(){
  const object=mapEditorSelected;if(!object)return;
  if(object._new){mapEditorObjects=mapEditorObjects.filter(item=>item!==object);mapEditorSelected=null;mapEditorDirty=false;renderMapEditorLayers();renderMapEditorObjectList();renderMapEditorForm();return;}
  if(!await askConfirmation({title:`${mapEditorTypeLabel(object)} löschen?`,message:`„${object.name}“ wird dauerhaft von der Arbeitskarte entfernt.`,icon:'×',tone:'danger',confirmLabel:'Löschen'}))return;
  try{
    if(object._kind==='zone'){await api(`/api/zones/${object.id}`,{method:'DELETE'});appData.zones=appData.zones.filter(item=>item.id!==object.id);}
    else if(object._kind==='connection'){await api(`/api/connections/${object.id}`,{method:'DELETE'});appData.connections=appData.connections.filter(item=>item.id!==object.id);}
    else{await api('/api/home',{method:'DELETE'});appData.home=null;}
    mapEditorObjects=mapEditorObjects.filter(item=>item!==object);mapEditorSelected=null;mapEditorDirty=false;renderMapData();renderZones();renderMapEditorLayers();renderMapEditorObjectList();renderMapEditorForm();await refreshMapEditorTopology();showToast('Kartenobjekt gelöscht','success');
  }catch(error){showToast(error.message,'error');}
}
function fitMapEditorSelection(){const layer=mapEditorSelected?._layer;if(!layer)return;const bounds=layer.getBounds?.();if(bounds?.isValid())mapEditorMap.fitBounds(bounds,{padding:[45,45],maxZoom:22});else if(layer.getLatLng)mapEditorMap.setView(layer.getLatLng(),22);}
function fitMapEditorAll(){
  if(!mapEditorMap)return;const bounds=L.latLngBounds([]);mapEditorObjects.forEach(object=>{if(object._kind==='home')bounds.extend([object.lat,object.lon]);else{const coords=object._kind==='zone'?object.geometry?.coordinates?.[0]:object.geometry?.coordinates;(coords||[]).forEach(([lon,lat])=>bounds.extend([lat,lon]));}});
  if(bounds.isValid())mapEditorMap.fitBounds(bounds,{padding:[42,42],maxZoom:21});else if(currentPose)mapEditorMap.setView([currentPose.lat,currentPose.lon],20);
}
async function refreshMapEditorTopology(){
  const box=$('map-editor-topology');if(!box)return;
  try{const topology=await api('/api/map/topology');box.classList.toggle('warning',!topology.valid);box.innerHTML=topology.valid?`<strong>Kartenstruktur vollständig</strong><br>${topology.zone_count} Mähzone(n), ${topology.no_go_count} No-Go-Zone(n), ${topology.connection_count} Weg(e)`:`<strong>Hinweis zur Kartenstruktur</strong><br>${topology.warnings.map(escapeHtml).join('<br>')}`;}catch(error){box.classList.add('warning');box.textContent='Kartenstruktur konnte nicht geprüft werden';}
}
$('map-editor-name')?.addEventListener('input',()=>{mapEditorDirty=true;});
$('map-editor-zone-type')?.addEventListener('change',event=>{if(mapEditorSelected?._kind==='zone'){mapEditorSelected.type=event.target.value;mapEditorDirty=true;renderMapEditorLayers();renderMapEditorObjectList();}});
$('map-editor-connection-type')?.addEventListener('change',event=>{$('map-editor-from-zone')?.classList.toggle('hidden',event.target.value==='dock_link');mapEditorDirty=true;});
['map-editor-from-zone','map-editor-to-zone','map-editor-corridor-width','map-editor-bidirectional'].forEach(id=>$(id)?.addEventListener('change',()=>{mapEditorDirty=true;}));

// ----------------------------------------------------------------------
// WebSocket and telemetry
// ----------------------------------------------------------------------
function connectWS() {
  if (!authenticated) return;
  clearTimeout(reconnectTimer);
  if (ws) { ws.onclose = null; ws.close(); }
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${scheme}://${location.host}/ws`);
  ws.onopen = () => setConnection(true, 'Verbunden');
  ws.onclose = () => { setConnection(false, 'Getrennt'); reconnectTimer=setTimeout(connectWS,3000); };
  ws.onerror = () => setConnection(false, 'Fehler');
  ws.onmessage = ({data}) => {
    let msg; try { msg=JSON.parse(data); } catch { return; }
    if (msg.type === 'state') {
      const previousState=currentState;
      currentZoneId = msg.active_zone_id;
      updateUI(
        msg.state,
        msg.error_reason,
        msg.pause_reason,
        Boolean(msg.geofence_override_active),
      );
      if(previousState!==msg.state&&['MOWING','PAUSED','RETURNING','IDLE','CHARGING'].includes(msg.state)){
        // Frischer Missionsstart (auch via API/Zeitplan, nicht nur über den
        // UI-Button): alte Ist-Spuren löschen, sonst überlagern sich die
        // gefahrenen Linien mehrerer Läufe zu einem irreführenden Fahrbild.
        if(msg.state==='MOWING'&&['IDLE','CHARGING','ERROR',null,undefined].includes(previousState)){
          trailLine?.setLatLngs([]);truthLine?.setLatLngs([]);clearLiveCoverage();lastActualPose=null;
        }
        loadRoute({replaceLiveCoverage:msg.state!=='MOWING'});
      }
      loadEvents();
    } else if (msg.type === 'telemetry') updateTelemetry(msg);
  };
}
function reconnectNow(){ connectWS(); loadBootstrap(); }

function setText(id, value) { const el=$(id); if(el) el.textContent=value; }
function valueOrNA(v, unit='') { return v === null || v === undefined ? 'n/v' : `${v}${unit}`; }

function renderActiveSpeedTarget(outputs={}) {
  const active=['MOWING','RETURNING','DOCKING','OBSTACLE_AVOIDANCE'].includes(currentState);
  const target=Number(outputs.target_speed_kmh);
  if(active&&Number.isFinite(target)){
    const labels={headland:'Außenbahn',mow:'Mähbahn',turn:'Wenden',returning:'Rückfahrt',gear_change:'Gangwechsel',deck_transition:'Mähwerkwechsel'};
    const mode=labels[outputs.speed_mode]||'Aktives Ziel';
    setText('tel-speed-target',`${mode} ${target.toFixed(1).replace('.',',')} km/h`);
    return;
  }
  const maximum=Number(appData.settings.speed_kmh??.9);
  setText('tel-speed-target',`Mähbahn max. ${maximum.toFixed(1).replace('.',',')} km/h`);
}

function continuousSpeedKmh(rawSpeedMps, outputs={}) {
  if(rawSpeedMps===null||rawSpeedMps===undefined)return null;
  const raw=Math.abs(Number(rawSpeedMps))*3.6;
  if(!Number.isFinite(raw))return null;
  if(displayedSpeedKmh===null)displayedSpeedKmh=raw;
  else displayedSpeedKmh+=0.22*(raw-displayedSpeedKmh);
  if(raw<0.015&&Math.abs(Number(outputs.speed_command||0))<0.001)displayedSpeedKmh=0;
  return displayedSpeedKmh;
}

function updateTelemetry(data) {
  if (!data) return;
  appData.telemetry = data;
  const pose=data.pose||{}, gps=data.gps||{}, bat=data.battery||{}, safety=data.safety||{}, outputs=data.outputs||{};
  if(typeof safety.geofence_override_active==='boolean'){
    currentGeofenceOverride=safety.geofence_override_active;
  }
  if (pose.lat != null && pose.lon != null) {
    currentPose = pose;
    const ll=[pose.lat,pose.lon];
    robotMarker?.setLatLng(ll);
    latestTractorPose=pose;latestTractorSafety=safety;updateTractorMarker(robotMarker,pose,safety,map);
    const moving=Math.abs(Number(pose.speed_mps||0))>0.005;
    const missionMoving=moving&&['MOWING','RETURNING','DOCKING','OBSTACLE_AVOIDANCE'].includes(currentState);
    if (trailLine&&missionMoving) {
      const points=trailLine.getLatLngs();
      const last=points[points.length-1];
      if (!last || Math.abs(last.lat-pose.lat)>1e-7 || Math.abs(last.lng-pose.lon)>1e-7) {
        points.push(L.latLng(...ll)); if(points.length>2000) points.shift(); trailLine.setLatLngs(points);
      }
    }
    updateDeckFootprints(pose,outputs);
    updateLiveCoverage(pose,outputs,missionMoving);
    const truth=data.simulation_truth||{};
    if(truth.lat!=null&&truth.lon!=null){
      const truthLatLng=[truth.lat,truth.lon];truthMarker?.setLatLng(truthLatLng).setStyle({opacity:1,fillOpacity:1});
      if(truthLine&&missionMoving){const points=truthLine.getLatLngs(),last=points[points.length-1];if(!last||Math.abs(last.lat-truth.lat)>1e-8||Math.abs(last.lng-truth.lon)>1e-8){points.push(L.latLng(...truthLatLng));if(points.length>4000)points.shift();truthLine.setLatLngs(points);}}
    }
    lastActualPose=ll;
    if (teachRecording && !teachStatus?.suspended) {
      const last=teachPoints[teachPoints.length-1];
      if (!last || Math.abs(last[0]-pose.lat)>1e-7 || Math.abs(last[1]-pose.lon)>1e-7) teachPoints.push(ll);
      teachLine?.setLatLngs(teachPoints);
    }
    if (teachRobotMarker) {
      teachRobotMarker.setLatLng(ll);
      updateTractorMarker(teachRobotMarker,pose,safety,teachMap);
      if (teachMapFollow && teachRecording) teachMap?.panTo(ll,{animate:true,duration:.25});
    }
    if (followRobot) map?.panTo(ll, {animate:true,duration:.08,easeLinearity:1,noMoveStart:true});
  }
  const displayedSpeed=continuousSpeedKmh(pose.speed_mps,outputs);
  setText('tel-speed', displayedSpeed==null ? '—' : displayedSpeed.toFixed(1));
  renderActiveSpeedTarget(outputs);
  setText('tel-heading', pose.heading_deg == null ? '—' : Math.round(pose.heading_deg));
  setText('tel-tilt', safety.tilt_deg == null ? '—' : Number(safety.tilt_deg).toFixed(1));
  setText('tel-soc', bat.soc_percent ?? '—'); setText('top-soc-val', bat.soc_percent == null ? '—' : `${bat.soc_percent} %`);
  updateSocRing(bat.soc_percent);
  const gpsLabel=gps.label || ({4:'RTK Fix',5:'RTK Float',1:'GPS'}[gps.fix_quality]||'Kein Fix');
  setText('tel-gps', gpsLabel); setText('gps-accuracy-display', `${gpsLabel}${gps.hdop!=null?' · HDOP '+gps.hdop:''}`);
  if (gpsBadge) { gpsBadge.textContent=`GPS: ${gpsLabel}`; gpsBadge.className=`gps-map-badge ${gps.fix_quality===4?'fix':gps.fix_quality===5?'float':'none'}`; }
  const teachGps=$('teach-map-gps');if(teachGps){teachGps.textContent=`GPS: ${gpsLabel}`;teachGps.className=`teach-map-gps ${gps.fix_quality===4?'fix':gps.fix_quality===5?'float':'none'}`;}
  renderDiagnostics();
  if(currentState==='PAUSED')renderStateActions(currentState);
  renderPrimaryStatus();
  setConnection(!data.stale, data.stale?'Daten veraltet':'Verbunden');
}

const SOC_CIRC=163.4;
function updateSocRing(pct){ const ring=$('soc-ring'); if(!ring||pct==null)return; const p=Math.max(0,Math.min(100,pct)); ring.style.strokeDasharray=SOC_CIRC; ring.style.strokeDashoffset=SOC_CIRC*(1-p/100); }

// ----------------------------------------------------------------------
// Views and operational state
// ----------------------------------------------------------------------
const VIEWS=['v-home','v-schedule','v-history','v-diagnostics','v-settings'];
const VIEW_TITLES={'v-home':'Karte','v-schedule':'Zeitplan','v-history':'Verlauf','v-diagnostics':'Diagnose','v-settings':'Einstellungen'};
function showView(id){ if(id!=='v-diagnostics'&&diagnosticJoystickPointer!==null)stopDiagnosticJoystick(); VIEWS.forEach(v=>$(v)?.classList.toggle('active',v===id)); document.querySelectorAll('.nav-item').forEach(b=>b.classList.toggle('active',b.dataset.view===id)); setText('view-title',VIEW_TITLES[id]||''); closeNavMenu(); if(id==='v-schedule'){renderSchedule();renderWeekGrid();updateNextMow();} if(id==='v-history')renderHistory(); if(id==='v-diagnostics'){renderDiagnostics();loadEvents();} }
document.querySelectorAll('.nav-item[data-view]').forEach(btn=>btn.addEventListener('click',()=>showView(btn.dataset.view)));
function openNavMenu(){ $('topbar')?.classList.add('menu-open'); if($('nav-scrim'))$('nav-scrim').style.display='block'; }
function closeNavMenu(){ $('topbar')?.classList.remove('menu-open'); if($('nav-scrim'))$('nav-scrim').style.display='none'; }
$('nav-toggle')?.addEventListener('click',e=>{e.stopPropagation();$('topbar')?.classList.contains('menu-open')?closeNavMenu():openNavMenu();});
$('nav-scrim')?.addEventListener('click',closeNavMenu);

function updateUI(state, errorReason='', pauseReason='', geofenceOverrideActive=false) {
  currentState=state;
  currentErrorReason=errorReason||'';
  currentPauseReason=pauseReason||'';
  currentGeofenceOverride=Boolean(geofenceOverrideActive);
  document.documentElement.dataset.state=state;
  renderPrimaryStatus();
  renderStateActions(state); renderZones();
  syncCommandAvailability();
  renderDiagnostics();
}

function actionButton(text, cls, handler, requiresMissionReady=false) { const b=document.createElement('button'); b.className=`ha-btn ${cls}`; b.dataset.onlineCommand='1'; if(requiresMissionReady){b.dataset.requiresMissionReady='1';b.title=missionBlockReason();}b.disabled=!commandOnline||(requiresMissionReady&&Boolean(missionBlockReason())); b.textContent=text; b.onclick=handler; return b; }
function renderStateActions(state) {
  const wrap=$('sc-actions'); if(!wrap)return;
  const geofenceStopped=state==='PAUSED'&&currentPauseReason.startsWith('Geofence violation');
  const stillOutside=appData.telemetry?.safety?.geofence_ok===false;
  // Telemetry arrives ten times per second. Replacing the complete action row
  // on every sample detached the button underneath an active pointer press,
  // which made safety actions flicker and occasionally lose the click. Only
  // rebuild when the actual set of actions changes; availability is synced
  // separately without replacing DOM nodes.
  const renderKey=`${state}:${geofenceStopped&&stillOutside?'geofence':'normal'}`;
  if(wrap.dataset.renderKey===renderKey)return;
  wrap.dataset.renderKey=renderKey;
  wrap.replaceChildren();
  if(state==='IDLE') { wrap.append(actionButton('Mission starten','ha-primary',()=>startMissionPrompt(),true),actionButton('Teach-In aufnehmen','ha-ghost',openTeachWizard)); }
  else if(state==='CHARGING') { wrap.append(actionButton('Mission starten','ha-primary',()=>startMissionPrompt(),true)); }
  else if(state==='MOWING'||state==='OBSTACLE_AVOIDANCE') { wrap.append(actionButton('Pausieren','ha-ghost',()=>command('/api/mission/pause')),actionButton('Zur Station','ha-primary',returnHome)); }
  else if(state==='PAUSED') {
    if(geofenceStopped&&stillOutside){
      wrap.append(
        actionButton('Verletzung ignorieren','ha-danger',ignoreGeofenceAndResume),
        actionButton('Zur Station','ha-ghost',returnHome),
      );
    }else{
      wrap.append(
        actionButton('Fortsetzen','ha-primary',()=>command('/api/mission/resume')),
        actionButton('Zur Station','ha-ghost',returnHome),
      );
    }
  }
  else if(state==='TEACH_IN') wrap.append(actionButton('Teach-In beenden','ha-danger',()=>command('/api/teach-in/stop')));
  else if(state==='ERROR') wrap.append(actionButton('Fehler zurücksetzen','ha-danger',resetError));
  else { const info=document.createElement('div'); info.className='ha-info'; info.textContent=STATE_SUBS[state]||state; wrap.append(info); }
}

async function startMissionPrompt(zoneId=null) {
  if(missionStartPending)return showToast('Mission wird bereits geplant','info');
  const mowing=appData.zones.filter(z=>z.type!=='no_go');
  if(!mowing.length) return showToast('Zuerst eine Mähzone anlegen', 'warning');
  if(!zoneId && mowing.length===1) zoneId=mowing[0].id;
  if(!zoneId) {
    zoneId=await openAppDialog({title:'Mähzone auswählen',message:'Welche Fläche soll als Nächstes gemäht werden?',icon:'◎',tone:'info',confirmLabel:'Weiter',choices:mowing.map(zone=>({value:zone.id,label:zone.name,meta:'Mähbereich'}))});
    if(!zoneId)return;
  }
  const zone=mowing.find(z=>z.id===zoneId);
  const departure=currentState==='CHARGING'?' und Ladestation verlassen':'';
  if(!await askConfirmation({title:'Mission starten?',message:`Die Mission in „${zone?.name||'Zone'}“ wird gestartet${departure}.`,icon:'▶',tone:'success',confirmLabel:'Mission starten'}))return;
  clearMissionLines();
  setMissionPlanning(true,zone?.name);
  let startError=null;
  try{
    const started=await api('/api/mission/start',{method:'POST',body:{zone_id:zoneId,confirmed:true}});
    if(started?.state)updateUI(started.state);
    showToast(started?.plan_cache_hit?'Gespeicherter Plan sofort geladen':'Mission neu geplant und gestartet','success');
    await loadRoute();fitMissionRoute();
  } catch(error) {
    startError=error;
  } finally {
    setMissionPlanning(false);
  }
  if(startError){
    await showAppMessage({title:'Mission konnte nicht gestartet werden',message:startError.message,icon:'!',tone:'danger',confirmLabel:'Einstellungen prüfen'});
    if(startError.message.includes('Fields2Cover'))showView('v-settings');
  }
}
async function loadRoute({replaceLiveCoverage=false}={}){const stateAtRequest=currentState;try{appData.route=await api('/api/mission/route');renderMapData({replaceLiveCoverage:replaceLiveCoverage&&currentState===stateAtRequest});}catch(error){console.warn('Route konnte nicht geladen werden',error);}}
async function returnHome(){
  if(!appData.home)return showToast('Bitte zuerst die Ladestation mit H auf der Karte speichern','warning');
  if(!await askConfirmation({title:'Zur Ladestation?',message:'Der laufende Mähvorgang wird beendet und der Roboter fährt zur Ladestation zurück.',icon:'⌂',confirmLabel:'Rückkehr starten'}))return;
  const result=await command('/api/mission/return-home',null,'Rückkehr gestartet');
  if(result?.route){appData.route=result.route;renderMapData({replaceLiveCoverage:true});}
}
async function resetError(){ if(await askConfirmation({title:'Fehler zurücksetzen?',message:'Der Gefahrenbereich muss vorher geprüft und die Fehlerursache behoben sein.',icon:'!',tone:'danger',confirmLabel:'Fehler zurücksetzen'})) await command('/api/reset',{confirmed:true},'Fehler zurückgesetzt'); }
async function ignoreGeofenceAndResume(){
  const confirmed=await askConfirmation({
    title:'Geofence-Verletzung ignorieren?',
    message:'Der Traktor fährt trotz verletzter Grenze weiter. Traktor und Mähwerke können dabei die Arbeitszone verlassen. Beobachte das Fahrzeug unmittelbar; der Schutz wird erst nach vollständigem Wiedereintritt automatisch reaktiviert.',
    icon:'!',tone:'danger',confirmLabel:'Ignorieren und fortsetzen',
  });
  if(!confirmed)return;
  await command(
    '/api/mission/resume-geofence-override',
    {confirmed:true,action:'ignore_current_geofence_violation'},
    'Geofence-Ausnahme aktiv',
  );
}
$('global-soft-stop')?.addEventListener('click',()=>command('/api/soft-stop',null,'Soft-Stop ausgelöst'));
$('global-return-home')?.addEventListener('click',returnHome);
async function triggerEmergencyStop(){if(await askConfirmation({title:'NOT-AUS auslösen?',message:'Motor, Fahrantrieb und Messer werden sofort gestoppt.',icon:'!',tone:'danger',confirmLabel:'NOT-AUS'}))command('/api/estop',{confirmed:true},'Not-Aus ausgelöst');}
$('global-estop')?.addEventListener('click',triggerEmergencyStop);
$('teach-header-estop')?.addEventListener('click',triggerEmergencyStop);
['global-soft-stop','global-return-home','global-estop','teach-header-estop'].forEach(id=>{if($(id))$(id).dataset.onlineCommand='1';});

// ----------------------------------------------------------------------
// Zones and Teach-In
// ----------------------------------------------------------------------
function getZones(){ return appData.zones; }
function renderZones(){ const list=$('zone-list'); if(!list)return; if(!appData.zones.length){list.innerHTML='<div class="zone-empty"><div class="zone-empty-icon">🌿</div><p>Noch keine Bereiche gespeichert.<br>Starte ein Teach-In oder zeichne sie auf der Karte.</p></div>';return;} const offlineDisabled=commandOnline?'':' disabled';const missionReason=missionBlockReason();const missionDisabled=commandOnline&&!missionReason?'':' disabled';list.innerHTML=appData.zones.map(z=>`<div class="zone-item"><div class="zone-swatch" style="background:${z.type==='no_go'?'#f2565b':'#3fc66b'}"></div><div class="zone-info"><div class="zone-name">${escapeHtml(z.name)}</div><div class="zone-meta">${z.type==='no_go'?'No-Go-Zone':'Mähbereich'} · servergespeichert</div></div><button class="zone-play" title="Auf Karte bearbeiten" onclick="openMapEditor('zone:${z.id}')">✎</button>${z.type!=='no_go'?`<button class="zone-play" data-online-command data-requires-mission-ready="1"${missionDisabled} title="${commandOnline&&!missionReason?'Mission starten':escapeHtml(missionReason||'Keine Verbindung – Befehl gesperrt')}" onclick="startMissionPrompt('${z.id}')">▶</button>`:''}<button class="zone-play" data-online-command${offlineDisabled} title="${commandOnline?'Zone löschen':'Keine Verbindung – Befehl gesperrt'}" onclick="deleteZone('${z.id}')">×</button></div>`).join(''); }
async function deleteZone(id){ const zone=appData.zones.find(z=>z.id===id); if(!await askConfirmation({title:'Zone löschen?',message:`„${zone?.name||'Zone'}“ wird dauerhaft gelöscht.`,icon:'×',tone:'danger',confirmLabel:'Zone löschen'}))return; try{await api(`/api/zones/${id}`,{method:'DELETE'});appData.zones=appData.zones.filter(z=>z.id!==id);renderZones();renderMapData();}catch(e){showToast(e.message,'error');} }
function openTeachWizard(){
  wizStep=1;teachPoints=[];teachStatus=null;teachMapFollow=true;$('teach-map-follow')?.classList.add('on');serverTeachGeometry=null;serverTeachPointCount=0;teachLine?.setLatLngs([]);
  renderWizDots();showWizStep(1);$('teach-wizard').classList.add('open');$('wiz-rec-indicator').classList.add('hidden');
  $('wiz-s2-actions').classList.remove('hidden');$('wiz-s2-stop').classList.add('hidden');$('teach-correction')?.classList.add('hidden');
  clearInterval(recTimer);clearInterval(teachStatusTimer);recSecs=0;resetTeachJoystick();
  document.querySelectorAll('input[name="teach-reference"]').forEach(input=>{input.disabled=false;});
}
async function wizClose(){
  $('teach-wizard').classList.remove('open');$('teach-wizard')?.querySelector('.teach-sheet')?.classList.remove('record-mode');
  clearInterval(recTimer);clearInterval(teachStatusTimer);stopTeachJoystick();
  if(teachRecording){teachRecording=false;await command('/api/teach-in/stop');}
}
async function wizNext(){ if(wizStep===3){await saveTeachZone();return;} wizStep++;if(wizStep>4){wizClose();return;}renderWizDots();showWizStep(wizStep); }
function showWizStep(n){
  for(let i=1;i<=4;i++)$(`wiz-s${i}`)?.classList.toggle('active',i===n);
  $('teach-wizard')?.querySelector('.teach-sheet')?.classList.toggle('record-mode',n===2);
  if(n===2) requestAnimationFrame(()=>initTeachMap());
}
function renderWizDots(){const wrap=$('wiz-dots');if(!wrap)return;wrap.innerHTML='';for(let i=1;i<=4;i++){const d=document.createElement('div');d.className='wiz-dot '+(i<wizStep?'done':i===wizStep?'curr':'next');wrap.appendChild(d);}}

function initTeachMap(){
  if(!window.L){setText('teach-map-placeholder','Kartenbibliothek nicht verfügbar');return;}
  if(!teachMap){
    teachMap=L.map('teach-map',{zoomControl:false,attributionControl:true,maxZoom:23}).setView(currentPose?[currentPose.lat,currentPose.lon]:[48.5,11],19);
    teachMap.attributionControl.setPrefix(false);
    L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',{attribution:'© Esri',maxZoom:23,maxNativeZoom:19}).addTo(teachMap);
    L.control.zoom({position:'bottomleft'}).addTo(teachMap);
    teachRobotMarker=L.marker(currentPose?[currentPose.lat,currentPose.lon]:[48.5,11],{icon:tractorIcon(),interactive:false,zIndexOffset:900}).addTo(teachMap);
    teachRouteLine=L.polyline([],{color:'#ffb020',weight:5,opacity:.95}).addTo(teachMap);
    const returnIcon=L.divIcon({className:'',html:'<div class="teach-return-marker">↶</div>',iconSize:[28,28],iconAnchor:[14,14]});
    teachReturnMarker=L.marker([0,0],{icon:returnIcon,zIndexOffset:700,opacity:0}).bindTooltip('Hier wieder in die Aufzeichnung einsteigen').addTo(teachMap);
    const startIcon=L.divIcon({className:'',html:'<div class="teach-start-marker">S</div>',iconSize:[24,24],iconAnchor:[12,12]});
    teachStartMarker=L.marker([0,0],{icon:startIcon,zIndexOffset:500,opacity:0}).bindTooltip('Startpunkt').addTo(teachMap);
    teachMap.on('dragstart zoomstart',()=>{teachMapFollow=false;$('teach-map-follow')?.classList.remove('on');});
    teachMap.on('zoomend',()=>updateTractorMarker(teachRobotMarker,latestTractorPose,latestTractorSafety,teachMap));
  }
  $('teach-map-placeholder')?.classList.add('hidden');
  setTimeout(()=>{teachMap.invalidateSize();if(currentPose&&teachMapFollow)teachMap.setView([currentPose.lat,currentPose.lon],19);},50);
}

$('teach-map-follow')?.addEventListener('click',()=>{teachMapFollow=!teachMapFollow;$('teach-map-follow').classList.toggle('on',teachMapFollow);if(teachMapFollow&&currentPose)teachMap?.setView([currentPose.lat,currentPose.lon],19);});

function haversineMeters(a,b){
  if(!a||!b)return null;const rad=Math.PI/180,dLat=(b[0]-a[0])*rad,dLon=(b[1]-a[1])*rad,lat1=a[0]*rad,lat2=b[0]*rad;
  const h=Math.sin(dLat/2)**2+Math.cos(lat1)*Math.cos(lat2)*Math.sin(dLon/2)**2;return 6371000*2*Math.atan2(Math.sqrt(h),Math.sqrt(1-h));
}
function renderTeachStatus(status){
  const wasSuspended=Boolean(teachStatus?.suspended);teachStatus=status;
  const coords=status.geometry?.coordinates||[],latlngs=coords.map(([lon,lat])=>[lat,lon]);
  teachPoints=latlngs;teachRouteLine?.setLatLngs(latlngs);teachLine?.setLatLngs(latlngs);
  setText('teach-length',`${Number(status.length_m||0).toFixed(1).replace('.',',')} m`);setText('teach-point-count',status.point_count||0);
  const startDistance=currentPose&&latlngs.length?haversineMeters(latlngs[0],[currentPose.lat,currentPose.lon]):null;
  setText('teach-start-distance',startDistance==null?'—':`${startDistance.toFixed(1).replace('.',',')} m`);
  const target=status.correction_target;
  if(latlngs.length&&teachStartMarker)teachStartMarker.setLatLng(latlngs[0]).setOpacity(1);else teachStartMarker?.setOpacity(0);
  if(target&&teachReturnMarker){teachReturnMarker.setLatLng([target[1],target[0]]).setOpacity(1);}else teachReturnMarker?.setOpacity(0);
  $('teach-correction')?.classList.toggle('hidden',!status.suspended);
  if(status.suspended){
    const distance=status.distance_to_target_m;setText('teach-correction-text',`Fahre ohne Aufzeichnung zur violetten Zielmarke zurück${distance==null?'':` · noch ${Number(distance).toFixed(1).replace('.',',')} m`}.`);
    setText('wiz-rec-label','Korrekturfahrt · Aufnahme pausiert');
  }else{
    setText('wiz-rec-label',status.closed?'Grenze automatisch geschlossen':'Aufnahme läuft');
    if(wasSuspended)showToast('Wiedereinstieg erreicht · Aufzeichnung läuft weiter','success');
  }
  document.querySelectorAll('#teach-undo-actions button').forEach(button=>button.disabled=!teachRecording||Number(status.length_m||0)<.5);
}
async function loadTeachStatus(){
  if(!teachRecording||teachStatusLoading)return;teachStatusLoading=true;
  try{renderTeachStatus(await api('/api/teach-in/status'));}catch(error){console.warn(error);}finally{teachStatusLoading=false;}
}
async function teachUndo(distance){
  if(!teachRecording)return;stopTeachJoystick();
  const result=await command('/api/teach-in/undo',{distance_m:distance});if(!result)return;
  renderTeachStatus(result);showToast(`${Number(result.removed_distance_m||0).toFixed(1).replace('.',',')} m verworfen · zur Zielmarke zurückfahren`,'warning');
}
async function teachContinueHere(){
  if(!teachStatus?.suspended)return;
  if(!await askConfirmation({title:'Ab hier fortsetzen?',message:'Die aktuelle Position wird geradlinig mit dem letzten gültigen Teach-In-Punkt verbunden.',icon:'↪',confirmLabel:'Fortsetzen'}))return;
  const result=await command('/api/teach-in/continue-here');if(result)renderTeachStatus(result);
}
async function wizStartRecording(){
  const reference=document.querySelector('input[name="teach-reference"]:checked')?.value||'center';
  const ok=await command('/api/teach-in/start',{reference});if(!ok)return;teachRecording=true;teachPoints=[];teachStatus=null;serverTeachGeometry=null;serverTeachPointCount=0;
  document.querySelectorAll('input[name="teach-reference"]').forEach(input=>{input.disabled=true;});
  $('teach-joystick')?.classList.remove('disabled');$('wiz-rec-indicator').classList.remove('hidden');$('wiz-s2-actions').classList.add('hidden');$('wiz-s2-stop').classList.remove('hidden');
  recSecs=0;clearInterval(recTimer);recTimer=setInterval(()=>{recSecs++;setText('wiz-rec-time',`${String(Math.floor(recSecs/60)).padStart(2,'0')}:${String(recSecs%60).padStart(2,'0')}`);},1000);
  await loadTeachStatus();clearInterval(teachStatusTimer);teachStatusTimer=setInterval(loadTeachStatus,750);
}
async function wizStopRecording(){
  stopTeachJoystick();clearInterval(recTimer);clearInterval(teachStatusTimer);teachRecording=false;$('teach-joystick')?.classList.add('disabled');
  const result=await command('/api/teach-in/stop');if(!result)return;serverTeachGeometry=result.geometry;serverTeachPointCount=result.point_count||0;document.querySelectorAll('input[name="teach-reference"]').forEach(input=>{input.disabled=false;});wizStep=3;renderWizDots();showWizStep(3);
}
async function saveTeachZone(){const name=$('zone-name-input').value.trim()||`Bereich ${appData.zones.length+1}`;const existing=appData.zones.find(z=>z.name.toLowerCase()===name.toLowerCase());if(existing&&!await askConfirmation({title:'Doppelten Namen verwenden?',message:`„${name}“ existiert bereits. Der neue Bereich wird zusätzlich gespeichert.`,icon:'!',confirmLabel:'Trotzdem speichern'}))return;let geometry=serverTeachGeometry;if(!geometry&&teachPoints.length>=3){const ring=teachPoints.map(([lat,lon])=>[lon,lat]);ring.push(ring[0]);geometry={type:'Polygon',coordinates:[ring]};}if(!geometry)return showToast('Zu wenige GPS-Punkte für eine Zone','warning');try{const zone=await api('/api/zones',{method:'POST',body:{name,type:$('zone-type-input').value,geometry}});appData.zones.push(zone);wizStep=4;renderWizDots();showWizStep(4);const count=serverTeachPointCount||teachPoints.length;setText('wiz-success-body',`„${name}“ wurde mit ${count} GPS-Punkten auf dem Server gespeichert.`);renderZones();renderMapData();showToast('Bereich gespeichert','success');}catch(e){showToast(e.message,'error');}}
$('teach-fab-btn')?.addEventListener('click',openTeachWizard);$('teach-nav-btn')?.addEventListener('click',()=>{showView('v-home');openTeachWizard();});

// ----------------------------------------------------------------------
// Scheduling
// ----------------------------------------------------------------------
function getSchedules(){return appData.schedules;}
function dayLabels(s){return (s.days||[]).map(d=>typeof d==='number'?DAYS[d]:d);}
function renderSchedule(){const wrap=$('schedule-list');if(!wrap)return;wrap.innerHTML='';if(!appData.schedules.length){wrap.innerHTML='<div class="empty-state"><div class="empty-icon">📅</div><p>Kein serverseitiger Zeitplan vorhanden.</p></div>';return;}appData.schedules.forEach(s=>{const zone=appData.zones.find(z=>z.id===s.zone_id);const card=document.createElement('div');card.className='schedule-card';card.style.marginBottom='8px';card.innerHTML=`<div class="sched-top"><div class="sched-time-badge">${escapeHtml(s.window_start||'09:00')}–${escapeHtml(s.window_end||'15:00')}</div><div class="sched-days">${DAYS.map(d=>`<div class="day-chip ${dayLabels(s).includes(d)?'on':''}">${d}</div>`).join('')}</div><button class="sched-delete" onclick="deleteSchedule('${s.id}')">×</button></div><div class="sched-footer"><div class="sched-badge">${s.enabled?'Aktiv':'Aus'}</div><div class="sched-dur">${escapeHtml(zone?.name||'Zone fehlt')}</div><button class="schedule-skip" onclick="skipSchedule('${s.id}')">${s.skip_next?'Wird übersprungen':'Nächste überspringen'}</button></div>`;wrap.appendChild(card);});}
function renderWeekGrid(){const grid=$('week-grid');if(!grid)return;const today=(new Date().getDay()+6)%7;grid.innerHTML=DAYS.map((d,i)=>{const list=appData.schedules.filter(s=>dayLabels(s).includes(d));return `<div class="day-col ${list.length?'has-sched':''} ${i===today?'today':''}"><div class="day-name">${d}</div><div class="day-dot${list.length?' active':''}"></div>${list.slice(0,2).map(s=>`<div class="day-time">${(s.window_start||'--').slice(0,2)}</div>`).join('')}</div>`;}).join('');}
function updateNextMow(){const time=$('next-mow-time'),sub=$('next-mow-sub');if(!time||!sub)return;if(!appData.schedules.length){time.textContent='—';sub.textContent='Kein Zeitplan konfiguriert';return;}const now=new Date();let best=null;appData.schedules.filter(s=>s.enabled&&!s.skip_next).forEach(s=>(s.days||[]).forEach(day=>{const idx=typeof day==='number'?day:DAYS.indexOf(day);const target=new Date(now);let diff=((idx+1)%7-now.getDay()+7)%7;target.setDate(now.getDate()+diff);const [h,m]=(s.window_start||'09:00').split(':').map(Number);target.setHours(h,m,0,0);if(target<=now)target.setDate(target.getDate()+7);if(!best||target<best.time)best={time:target,s};}));if(best){time.textContent=`${best.s.window_start} Uhr`;const zone=appData.zones.find(z=>z.id===best.s.zone_id);sub.textContent=`${zone?.name||'Zone'} · ${best.time.toLocaleDateString('de-DE',{weekday:'short',day:'2-digit',month:'2-digit'})}`;}}
function openSchedModal(){const select=$('m-zone');select.innerHTML=appData.zones.filter(z=>z.type!=='no_go').map(z=>`<option value="${z.id}">${escapeHtml(z.name)}</option>`).join('');document.querySelectorAll('#m-days .day-chip').forEach(c=>c.classList.toggle('on',['Mo','Di','Mi','Do','Fr'].includes(c.dataset.day)));$('sched-modal').classList.add('open');}
function closeSchedModal(){$('sched-modal').classList.remove('open');}
async function saveSchedule(){const days=[...document.querySelectorAll('#m-days .day-chip.on')].map(c=>DAYS.indexOf(c.dataset.day));const zoneId=$('m-zone').value;if(!zoneId)return showToast('Bitte zuerst eine Mähzone anlegen','warning');if(!days.length)return showToast('Mindestens einen Tag auswählen','warning');const start=`${$('m-hour').value}:${$('m-min').value}`;try{const s=await api('/api/schedules',{method:'POST',body:{zone_id:zoneId,days,window_start:start,window_end:$('m-window-end').value,duration_min:Number(document.querySelector('#m-dur .dur-opt.on')?.dataset.val||90),enabled:true}});appData.schedules.push(s);closeSchedModal();renderSchedule();renderWeekGrid();updateNextMow();showToast('Zeitplan serverseitig gespeichert','success');}catch(e){showToast(e.message,'error');}}
async function deleteSchedule(id){if(!await askConfirmation({title:'Zeitplan löschen?',message:'Dieser Zeitplan wird dauerhaft entfernt.',icon:'×',tone:'danger',confirmLabel:'Zeitplan löschen'}))return;try{await api(`/api/schedules/${id}`,{method:'DELETE'});appData.schedules=appData.schedules.filter(s=>s.id!==id);renderSchedule();renderWeekGrid();updateNextMow();}catch(e){showToast(e.message,'error');}}
async function skipSchedule(id){try{const updated=await api(`/api/schedules/${id}/skip-next`,{method:'POST'});const i=appData.schedules.findIndex(s=>s.id===id);appData.schedules[i]=updated;renderSchedule();updateNextMow();showToast('Nächste Mission wird übersprungen','success');}catch(e){showToast(e.message,'error');}}
$('add-sched-btn')?.addEventListener('click',openSchedModal);document.querySelectorAll('#m-days .day-chip').forEach(c=>c.addEventListener('click',()=>c.classList.toggle('on')));document.querySelectorAll('#m-dur .dur-opt').forEach(o=>o.addEventListener('click',()=>{document.querySelectorAll('#m-dur .dur-opt').forEach(x=>x.classList.remove('on'));o.classList.add('on');}));
(function buildHours(){const s=$('m-hour');if(!s)return;for(let h=6;h<=20;h++){const o=document.createElement('option');o.value=o.textContent=String(h).padStart(2,'0');if(h===9)o.selected=true;s.appendChild(o);}})();

// ----------------------------------------------------------------------
// Settings, history and diagnostics
// ----------------------------------------------------------------------
let settingsTimer=null;
function uiSettings(){const s=appData.settings;return {mowingWidth:s.mowing_width_cm??60,lane:s.lane_width_cm??38,resolution:Math.min(.9,s.route_resolution_cm??.5),speed:s.speed_kmh??.9,turnRadius:s.turn_radius_cm??25,mowingTurnRadius:s.mowing_turn_radius_cm??50,headland:s.headland_margin_cm??5,turnSpeed:s.turn_speed_kmh??.4,liftSettle:s.deck_lift_settle_ms??700,frontWidth:s.front_mower_width_percent??60,deckDepth:s.mower_deck_depth_cm??14,frontOffset:s.front_mower_offset_cm??35,rearOffset:s.rear_mower_offset_cm??22,rearGap:s.rear_mower_gap_cm??6,showFootprints:s.show_mower_footprints!==false,coverageGrid:s.coverage_grid_resolution_cm??2,wheelbase:s.vehicle_wheelbase_cm??25,tilt:s.tilt_limit_deg??30,rainwait:s.rain_wait_minutes??30,rain:s.rain_enabled!==false,geo:s.geofence_enabled!==false,robotName:s.robot_name||'MV2-Alpha'};}
function saveSettingsPatch(patch){Object.assign(appData.settings,patch);clearTimeout(settingsTimer);settingsTimer=setTimeout(()=>api('/api/settings',{method:'PUT',body:patch}).catch(e=>showToast(e.message,'error')),250);}
function initSlider(id,unit,decimals,onChange){const input=$(id);if(!input||input.dataset.ready)return;input.dataset.ready='1';const wrap=input.closest('.rng'),bubble=wrap?.querySelector('.rng-bubble');const paint=()=>{const v=+input.value,p=((v-input.min)/(input.max-input.min))*100;wrap?.style.setProperty('--pct',p+'%');if(bubble)bubble.textContent=(decimals?v.toFixed(decimals).replace('.',','):v)+unit;};input.addEventListener('input',()=>{paint();onChange?.(+input.value);});paint();}
function syncSliderVisual(id,unit,decimals){
  const input=$(id),wrap=input?.closest('.rng'),bubble=wrap?.querySelector('.rng-bubble');
  if(!input||!wrap)return;
  const value=+input.value,min=+input.min,max=+input.max;
  wrap.style.setProperty('--pct',((value-min)/(max-min))*100+'%');
  if(bubble)bubble.textContent=(decimals?value.toFixed(decimals).replace('.',','):String(value))+unit;
}
function renderLaneViz(value){
  const viz=$('lane-viz'),stripes=viz?.querySelector('.lane-stripes'),robot=viz?.querySelector('.lane-robot');
  const gap=24+(value-20)*1.425;
  viz?.style.setProperty('--lane-gap',gap.toFixed(2)+'px');
  const mowingWidth=numericSetting('s-mowing-width','mowing_width_cm',60),robotSize=Math.max(48,Math.min(150,mowingWidth*1.425/Math.max(.5,tractorVisibleWidthRatio)));
  if(robot){robot.style.width=`${robotSize.toFixed(1)}px`;robot.style.height=`${robotSize.toFixed(1)}px`;}
  setText('lane-caliper-val',`${value} cm`);
  if(stripes){
    let html='';
    for(let offset=gap/2;offset<230;offset+=gap){
      html+=`<i style="left:calc(50% + ${offset.toFixed(1)}px)"></i><i style="left:calc(50% - ${offset.toFixed(1)}px)"></i>`;
    }
    stripes.innerHTML=html;
  }
}
function renderTiltViz(value){
  $('tilt-viz')?.style.setProperty('--tilt-ang',`${value}deg`);
  setText('tilt-angle-label',`${value}°`);
}
function renderSpeedViz(value){
  const segments=[...document.querySelectorAll('#speed-viz span')];
  const active=Math.max(0,Math.min(5,Math.round(((value-.3)/(1.8-.3))*5)));
  segments.forEach((segment,index)=>segment.classList.toggle('on',index<active));
}
function renderTurnRadiusNote(){
  const lane=Number($('s-lane')?.value||uiSettings().lane),radius=Number($('s-turn-radius')?.value||uiSettings().turnRadius),diameter=radius*2;
  const requiresShunts=lane<diameter;
  setText('turn-radius-note',requiresShunts
    ? `Wendekreis ${diameter.toFixed(0)} cm > Bahnabstand ${lane.toFixed(0)} cm: Rückwärts-/Mehrpunktwende erforderlich.`
    : `Wendekreis ${diameter.toFixed(0)} cm: eine durchgehende Vorwärtswende ist möglich.`);
}
function renderMowingTurnRadiusNote(){
  const turnRadius=Number($('s-turn-radius')?.value||uiSettings().turnRadius);
  const input=$('s-mowing-turn-radius');if(!input)return;
  input.min=String(Math.min(145,turnRadius+5));
  if(Number(input.value)<Number(input.min))input.value=input.min;
  syncSliderVisual('s-mowing-turn-radius',' cm',0);
  setText('mowing-turn-radius-note',`${Number(input.value).toFixed(0)} cm mit abgesenktem Mähwerk · enge Wendungen bleiben bei ${turnRadius.toFixed(0)} cm.`);
}
function renderMowingOverlap(){
  const mowing=Number($('s-mowing-width')?.value||uiSettings().mowingWidth);
  const laneInput=$('s-lane');
  if(laneInput)laneInput.max=String(mowing);
  const lane=Math.min(mowing,Number(laneInput?.value||uiSettings().lane));
  if(laneInput){
    if(Number(laneInput.value)!==lane)laneInput.value=String(lane);
    // Changing the mowing width also changes this range's coordinate system.
    // Repaint even when the lane value itself did not need clamping, otherwise
    // thumb, fill and bubble use different percentages until the next input.
    syncSliderVisual('s-lane',' cm',0);
  }
  const overlap=Math.max(0,mowing-lane),percent=mowing>0?overlap/mowing*100:0;
  setText('lane-overlap-note',`Mähbreite ${mowing.toFixed(0)} cm · Überlappung ${overlap.toFixed(0)} cm (${percent.toFixed(0)} %).`);
  return lane;
}
function renderHeadlandPassNote(){
  const strategy=$('s-coverage-strategy')?.value||appData.settings.coverage_strategy||'headland_first';
  const passes=Number($('s-headland-passes')?.value||appData.settings.headland_passes||1);
  const lane=Number($('s-lane')?.value||uiSettings().lane),mowing=Number($('s-mowing-width')?.value||uiSettings().mowingWidth),width=mowing+(passes-1)*lane;
  if($('s-headland-passes'))$('s-headland-passes').disabled=strategy!=='headland_first';
  setText('headland-pass-note',strategy==='headland_first'
    ? `${passes} ${passes===1?'Bahn':'Bahnen'} · ${width.toFixed(0)} cm gemähtes Vorgewende; konstanter Spurabstand auch in Kurven`
    : 'Deaktiviert – Mission beginnt direkt mit den parallelen Innenbahnen');
}
function initSettings(refresh=false){
  const s=uiSettings();
  const seed=(id,value)=>{if($(id))$(id).value=value;};
  seed('s-mowing-width',s.mowingWidth);seed('s-lane',s.lane);seed('s-route-resolution',s.resolution);seed('s-speed',s.speed);seed('s-turn-radius',s.turnRadius);seed('s-mowing-turn-radius',s.mowingTurnRadius);seed('s-headland-margin',s.headland);seed('s-turn-speed',s.turnSpeed);seed('s-lift-settle',s.liftSettle);seed('s-front-width',s.frontWidth);seed('s-deck-depth',s.deckDepth);seed('s-front-offset',s.frontOffset);seed('s-rear-offset',s.rearOffset);seed('s-rear-gap',s.rearGap);seed('s-coverage-grid',s.coverageGrid);seed('s-wheelbase',s.wheelbase);seed('s-tilt',s.tilt);seed('s-rainwait',s.rainwait);
  seed('s-f2c-angle',appData.settings.fields2cover_angle_deg??0);seed('s-f2c-split-angle',appData.settings.fields2cover_split_angle_deg??90);seed('s-f2c-headland-width',appData.settings.fields2cover_headland_width_cm??70);seed('s-f2c-max-diff-curv',appData.settings.fields2cover_max_diff_curvature??.1);seed('s-f2c-min-coverage',appData.settings.fields2cover_minimum_mainland_coverage_percent??90);
  renderMowingOverlap();
  initSlider('s-mowing-width',' cm',0,value=>{const lane=renderMowingOverlap();saveSettingsPatch({mowing_width_cm:value,lane_width_cm:lane});renderLaneViz(lane);renderHeadlandPassNote();renderPlannerSettings();updateTractorMarker(robotMarker);updateDeckFootprints(currentPose,appData.telemetry?.outputs||{});});
  initSlider('s-lane',' cm',0,value=>{saveSettingsPatch({lane_width_cm:value});renderMowingOverlap();renderLaneViz(value);renderTurnRadiusNote();renderHeadlandPassNote();});
  initSlider('s-route-resolution',' cm',1,value=>saveSettingsPatch({route_resolution_cm:value}));
  initSlider('s-speed',' km/h',1,value=>{
    saveSettingsPatch({speed_kmh:value});
    setText('tel-speed-target',`Mähbahn max. ${value.toFixed(1).replace('.',',')} km/h`);
    renderSpeedViz(value);
  });
  initSlider('s-turn-radius',' cm',0,value=>{renderMowingTurnRadiusNote();saveSettingsPatch({turn_radius_cm:value,mowing_turn_radius_cm:Number($('s-mowing-turn-radius')?.value||value+5)});renderTurnRadiusNote();renderPlannerSettings();});
  initSlider('s-mowing-turn-radius',' cm',0,value=>{saveSettingsPatch({mowing_turn_radius_cm:value});renderMowingTurnRadiusNote();});
  renderMowingTurnRadiusNote();
  initSlider('s-headland-margin',' cm',0,value=>saveSettingsPatch({headland_margin_cm:value}));
  initSlider('s-turn-speed',' km/h',1,value=>saveSettingsPatch({turn_speed_kmh:value}));
  initSlider('s-lift-settle',' ms',0,value=>saveSettingsPatch({deck_lift_settle_ms:value}));
  const geometryChanged=(key,value)=>{saveSettingsPatch({[key]:value});updateDeckFootprints(currentPose,appData.telemetry?.outputs||{});};
  initSlider('s-front-width',' %',0,value=>geometryChanged('front_mower_width_percent',value));
  initSlider('s-deck-depth',' cm',0,value=>geometryChanged('mower_deck_depth_cm',value));
  initSlider('s-front-offset',' cm',0,value=>geometryChanged('front_mower_offset_cm',value));
  initSlider('s-rear-offset',' cm',0,value=>geometryChanged('rear_mower_offset_cm',value));
  initSlider('s-rear-gap',' cm',0,value=>geometryChanged('rear_mower_gap_cm',value));
  initSlider('s-coverage-grid',' cm',0,value=>saveSettingsPatch({coverage_grid_resolution_cm:value}));
  initSlider('s-wheelbase',' cm',0,value=>{saveSettingsPatch({vehicle_wheelbase_cm:value});});
  initSlider('s-tilt','°',0,value=>{saveSettingsPatch({tilt_limit_deg:value});renderTiltViz(value);});
  initSlider('s-rainwait',' min',0,value=>saveSettingsPatch({rain_wait_minutes:value}));
  initSlider('s-f2c-angle','°',0,value=>saveSettingsPatch({fields2cover_angle_deg:value}));
  initSlider('s-f2c-split-angle','°',0,value=>saveSettingsPatch({fields2cover_split_angle_deg:value}));
  initSlider('s-f2c-headland-width',' cm',0,value=>{saveSettingsPatch({fields2cover_headland_width_cm:value});renderPlannerSettings();});
  initSlider('s-f2c-max-diff-curv','',2,value=>saveSettingsPatch({fields2cover_max_diff_curvature:value}));
  initSlider('s-f2c-min-coverage',' %',1,value=>saveSettingsPatch({fields2cover_minimum_mainland_coverage_percent:value}));
  renderLaneViz(Math.min(s.lane,s.mowingWidth));renderMowingOverlap();renderTiltViz(s.tilt);renderSpeedViz(s.speed);renderTurnRadiusNote();renderHeadlandPassNote();
  if($('s-mower-footprints')){$('s-mower-footprints').checked=s.showFootprints;if(!$('s-mower-footprints').dataset.ready){$('s-mower-footprints').dataset.ready='1';$('s-mower-footprints').addEventListener('change',()=>{saveSettingsPatch({show_mower_footprints:$('s-mower-footprints').checked});updateDeckFootprints(currentPose,appData.telemetry?.outputs||{});});}updateDeckFootprints(currentPose,appData.telemetry?.outputs||{});}
  if($('s-rain')){$('s-rain').checked=s.rain;if(!$('s-rain').dataset.ready){$('s-rain').dataset.ready='1';$('s-rain').addEventListener('change',()=>saveSettingsPatch({rain_enabled:$('s-rain').checked}));}}
  if($('s-geo')){$('s-geo').checked=s.geo;if(!$('s-geo').dataset.ready){$('s-geo').dataset.ready='1';$('s-geo').addEventListener('change',()=>saveSettingsPatch({geofence_enabled:$('s-geo').checked}));}}
  setText('robot-name-preview',s.robotName);document.querySelector('.brand-model').textContent=s.robotName;
  applyChosenSpeed(s.speed,false);setText('manual-limit-label',`Limit: ${Math.round((appData.settings.manual_speed_limit??.25)*100)} %`);
}
function renderPlannerSettings(){
  const engine=$('s-planner-engine')?.value||appData.settings.planner_engine||'mv2';
  $('fields2cover-settings')?.classList.toggle('hidden',engine!=='fields2cover');
  document.querySelectorAll('.mv2-only-setting').forEach(row=>row.classList.toggle('hidden',engine==='fields2cover'));
  setText('turn-settings-engine-note',engine==='fields2cover'
    ?'Fields2Cover erzeugt Innenbahnen und Kurven. Hardware-Radien, Außenbahnen, Wendegeschwindigkeit und Mähwerk-Hub bleiben gemeinsame Ausführungsgrenzen.'
    :'MV2 erzeugt Bahnen und Wendemanöver selbst; alle folgenden Optionen sind aktiv.');
  const fixed=($('s-f2c-angle-mode')?.value||appData.settings.fields2cover_angle_mode)==='fixed';
  document.querySelectorAll('.f2c-fixed-angle-row').forEach(row=>row.classList.toggle('hidden',!fixed));
  const decomposes=($('s-f2c-decomposition')?.value||appData.settings.fields2cover_decomposition)!=='none';
  document.querySelectorAll('.f2c-decomposition-row').forEach(row=>row.classList.toggle('hidden',!decomposes));
  const spiral=($('s-f2c-route-order')?.value||appData.settings.fields2cover_route_order)==='spiral';
  document.querySelectorAll('.f2c-spiral-row').forEach(row=>row.classList.toggle('hidden',!spiral));
  const headlandMode=$('s-f2c-headland-mode')?.value||appData.settings.fields2cover_headland_mode||'auto';
  document.querySelectorAll('.f2c-manual-headland-row').forEach(row=>row.classList.toggle('hidden',headlandMode!=='manual'));
  const manualHeadland=Number($('s-f2c-headland-width')?.value||appData.settings.fields2cover_headland_width_cm||70);
  const realMachineWidth=Number($('s-mowing-width')?.value||appData.settings.mowing_width_cm||60);
  setText('fields2cover-headland-note',headlandMode==='manual'
    ?`${manualHeadland.toFixed(0)} cm direkter Abstand`
    :`Aus Wenderadius und äußerer ${realMachineWidth.toFixed(0)}-cm-Seitenkontur; innenliegende Anbauteile erzeugen keinen pauschalen Zusatzabstand`);
  const status=appData.planners?.fields2cover,statusNode=$('fields2cover-native-status');
  if(statusNode){
    statusNode.classList.toggle('available',status?.available===true);statusNode.classList.toggle('unavailable',status?.available===false);
    statusNode.textContent=status?.available?`Native Bibliothek bereit · ${status.version||'Fields2Cover 2.x'}`:(status?.detail||'Native Bibliothek nicht verfügbar');
  }
}
function initPlanningSettings(){
  const bind=(id,key,event='change')=>{const el=$(id);if(!el)return;const value=appData.settings[key];if(el.type==='checkbox')el.checked=value!==false;else if(value!=null)el.value=value;if(!el.dataset.ready){el.dataset.ready='1';el.addEventListener(event,()=>saveSettingsPatch({[key]:el.type==='checkbox'?el.checked:el.value}));}};
  bind('s-season','season_mode'); bind('s-weekend','weekend_mode'); bind('s-quiet-start','quiet_hours_start'); bind('s-quiet-end','quiet_hours_end');
}
function initTurnSettings(){
  const bind=(id,key,convert=value=>value,afterChange=null)=>{const el=$(id);if(!el)return;const value=appData.settings[key];if(el.type==='checkbox')el.checked=value!==false;else if(value!=null)el.value=String(value);if(!el.dataset.ready){el.dataset.ready='1';el.addEventListener('change',()=>{saveSettingsPatch({[key]:el.type==='checkbox'?el.checked:convert(el.value)});afterChange?.();});}};
  bind('s-coverage-strategy','coverage_strategy',value=>value,renderHeadlandPassNote);
  bind('s-planner-engine','planner_engine',value=>value,renderPlannerSettings);
  bind('s-f2c-decomposition','fields2cover_decomposition',value=>value,renderPlannerSettings);
  bind('s-f2c-objective','fields2cover_swath_objective');
  bind('s-f2c-angle-mode','fields2cover_angle_mode',value=>value,renderPlannerSettings);
  bind('s-f2c-route-order','fields2cover_route_order',value=>value,renderPlannerSettings);
  bind('s-f2c-spiral-size','fields2cover_spiral_size',Number);
  bind('s-f2c-path-type','fields2cover_path_type');
  bind('s-f2c-turning-backend','fields2cover_turning_backend');
  bind('s-f2c-headland-mode','fields2cover_headland_mode',value=>value,renderPlannerSettings);
  bind('s-f2c-optimizer-time','fields2cover_optimizer_time_s',Number);
  bind('s-f2c-optimum','fields2cover_search_optimum');
  bind('s-headland-passes','headland_passes',Number,renderHeadlandPassNote);
  bind('s-turn-strategy','turn_strategy');
  bind('s-turn-passes','turn_maneuver_passes',Number);
  bind('s-lift-front','lift_front_on_turn');
  bind('s-lift-rear','lift_rear_on_turn');
  renderHeadlandPassNote();renderPlannerSettings();
}
function applyChosenSpeed(v,save=true){v=Math.max(.3,Math.min(1.8,Math.round(v*10)/10));if(save)saveSettingsPatch({speed_kmh:v});setText('tel-speed-target',`Mähbahn max. ${v.toFixed(1).replace('.',',')} km/h`);if($('s-speed')){$('s-speed').value=v;syncSliderVisual('s-speed',' km/h',1);}renderSpeedViz(v);return v;}
$('speed-down')?.addEventListener('click',()=>applyChosenSpeed(uiSettings().speed-.1));$('speed-up')?.addEventListener('click',()=>applyChosenSpeed(uiSettings().speed+.1));
$('card-robot-name')?.addEventListener('click',()=>{$('robot-name-input').value=uiSettings().robotName;$('name-modal').classList.add('open');});function closeNameModal(){$('name-modal').classList.remove('open');}function saveRobotName(){const name=$('robot-name-input').value.trim()||'MV2-Alpha';saveSettingsPatch({robot_name:name});setText('robot-name-preview',name);document.querySelector('.brand-model').textContent=name;closeNameModal();}
$('reset-btn-settings')?.addEventListener('click',resetError);

function renderHistory(){const sessions=appData.events.filter(e=>e.code==='state_change'&&String(e.message).includes('→ MOWING'));const stats=$('stats-row');if(stats)stats.innerHTML=`<div class="stat-card"><div class="stat-val">${sessions.length}</div><div class="stat-lbl">Starts</div></div><div class="stat-card"><div class="stat-val">server</div><div class="stat-lbl">Quelle</div></div><div class="stat-card"><div class="stat-val">0</div><div class="stat-lbl">Demo-Daten</div></div>`;const list=$('session-list');if(list)list.innerHTML=appData.events.slice(0,20).map(e=>`<div class="session-card"><div class="sess-main"><div class="sess-date">${new Date(e.timestamp).toLocaleString('de-DE')}</div><div class="sess-meta">${escapeHtml(e.message)}</div></div></div>`).join('')||'<div class="empty-state"><p>Noch kein echter Missionsverlauf vorhanden.</p></div>';const chart=$('session-chart');if(chart)chart.innerHTML='<div class="empty-state"><p>Flächenstatistik erscheint nach angebundener Wegauswertung.</p></div>';}

function diagClass(v,good=true){if(v==null)return'na';return good?(v?'ok':'bad'):(v?'bad':'ok');}
function diagText(v){return ({available:'verfügbar',unavailable:'nicht verfügbar',unknown:'unbekannt'})[v]||v||'n/v';}
function manualControlAllowed(){return commandOnline&&['IDLE','TEACH_IN','PAUSED'].includes(currentState);}
function renderDiagnostics(){
  const t=appData.telemetry||{},s=t.safety||{},b=t.battery||{},g=t.gps||{},d=t.diagnostics||{},o=t.outputs||{};
  const resume=s.rain_resume_at?new Date(s.rain_resume_at).toLocaleTimeString('de-DE',{hour:'2-digit',minute:'2-digit'})+' Uhr':'—';
  const geofenceLabel=s.geofence_override_active?'Ignoriert':valueOrNA(s.geofence_ok);
  const geofenceClass=s.geofence_override_active?'warn':diagClass(s.geofence_ok);
  const values=[['Hardware',t.hardware_connected?'Verbunden':'Nicht verbunden',diagClass(t.hardware_connected)],['GPS',g.label||'Kein Fix',g.fix_quality===4?'ok':g.fix_quality===5?'warn':'bad'],['IMU',diagText(d.imu),d.imu==='available'?'ok':'na'],['Neigung',valueOrNA(s.tilt_deg,'°'),s.tilt_deg==null?'na':s.tilt_deg<=Number(appData.settings.tilt_limit_deg||30)?'ok':'bad'],['Regen ADC',valueOrNA(s.rain_adc),s.raining==null?'na':s.raining?'bad':'ok'],['Regenpause bis',resume,s.rain_resume_at?'warn':'na'],['Bumper links',valueOrNA(s.bumper_left),s.bumper_left==null?'na':s.bumper_left?'bad':'ok'],['Bumper rechts',valueOrNA(s.bumper_right),s.bumper_right==null?'na':s.bumper_right?'bad':'ok'],['Lift',valueOrNA(s.lifted),s.lifted==null?'na':s.lifted?'bad':'ok'],['Geofence',geofenceLabel,geofenceClass],['Watchdog',valueOrNA(s.watchdog_ok),diagClass(s.watchdog_ok)],['Messer',valueOrNA(s.blade_running),s.blade_running==null?'na':'ok'],['Akku',valueOrNA(b.soc_percent,'%'),b.soc_percent==null?'na':b.soc_percent<=20?'bad':'ok'],['Spannung',valueOrNA(b.voltage_v,' V'),b.voltage_v==null?'na':'ok'],['Strom',valueOrNA(b.current_a,' A'),'na'],['Temperatur',valueOrNA(b.temperature_c,'°C'),'na'],['Messerstrom',diagText(d.blade_current),'na'],['Motorstrom',diagText(d.motor_current),'na']];
  const grid=$('diagnostic-grid');if(grid)grid.innerHTML=values.map(([l,v,c])=>`<div class="diag-card"><div class="diag-label">${l}</div><div class="diag-value ${c}">${escapeHtml(v)}</div></div>`).join('');
  const speed=Number(o.speed_command||0),steering=Number(o.steering_deg||0);
  const outputs=[['Gasstellung',`${speed>=0?'+':''}${Math.round(speed*100)} %`,Math.abs(speed)>0.001?'warn':'ok'],['Fahrtrichtung',speed>0.001?'Vorwärts':speed<-.001?'Rückwärts':'Stopp',Math.abs(speed)>0.001?'warn':'ok'],['Lenkwinkel',`${steering>=0?'+':''}${steering.toFixed(1).replace('.',',')}°`,Math.abs(steering)>0.1?'warn':'ok'],['Messer',o.blade_enabled?'Ein':'Aus',o.blade_enabled?'warn':'ok'],['Frontmähwerk',o.front_deck_raised?'Oben':'Unten',o.front_deck_raised?'warn':'ok'],['Heckmähwerk',o.rear_deck_raised?'Oben':'Unten',o.rear_deck_raised?'warn':'ok'],['Not-Aus-Befehl',o.estop_active?'Aktiv':'Inaktiv',o.estop_active?'bad':'ok']];
  const outputGrid=$('output-grid');if(outputGrid)outputGrid.innerHTML=outputs.map(([l,v,c])=>`<div class="diag-card"><div class="diag-label">${l}</div><div class="diag-value ${c}">${escapeHtml(v)}</div></div>`).join('');

  const allowed=manualControlAllowed(),stateBadge=$('diagnostic-manual-state'),joystick=$('diagnostic-joystick');
  joystick?.classList.toggle('disabled',!allowed);stateBadge?.classList.toggle('ready',allowed);setText('diagnostic-manual-state',allowed?'Bereit':'Gesperrt');
  const front=$('diagnostic-front-deck'),rear=$('diagnostic-rear-deck'),blade=$('diagnostic-blade'),stop=$('diagnostic-manual-stop');
  if(front){front.disabled=!allowed;front.classList.toggle('active',!!o.front_deck_raised);front.textContent=o.front_deck_raised?'Frontmähwerk absenken':'Frontmähwerk anheben';}
  if(rear){rear.disabled=!allowed;rear.classList.toggle('active',!!o.rear_deck_raised);rear.textContent=o.rear_deck_raised?'Heckmähwerk absenken':'Heckmähwerk anheben';}
  if(blade){blade.disabled=!allowed;blade.classList.toggle('active',!!o.blade_enabled);blade.textContent=o.blade_enabled?'Messer ausschalten':'Messer einschalten';}
  if(stop)stop.disabled=!commandOnline;
}
async function loadEvents(){if(!authenticated)return;try{appData.events=await api('/api/events?limit=100');const list=$('event-list');if(list)list.innerHTML=appData.events.map(e=>`<div class="event-row ${e.level}"><div class="event-title">${escapeHtml(e.message)}</div><div class="event-meta">${new Date(e.timestamp).toLocaleString('de-DE')} · ${escapeHtml(e.code)}</div>${e.action?`<div class="event-action">Handlung: ${escapeHtml(e.action)}</div>`:''}</div>`).join('')||'<div class="empty-state"><p>Keine Sicherheitsereignisse.</p></div>';renderHistory();}catch(e){console.warn(e);}}
async function loadVersions(){try{const v=await api('/api/maintenance/versions');setText('version-info',`App ${v.app} · Firmware ${v.firmware}`);}catch(e){console.warn(e);}}
async function startCalibration(kind){if(!await askConfirmation({title:`${kind.toUpperCase()} kalibrieren?`,message:'Der Kalibrierassistent wird gestartet. Folge anschließend den angezeigten Schritten.',icon:'◎',confirmLabel:'Kalibrierung starten'}))return;try{const r=await api(`/api/maintenance/calibrate/${kind}`,{method:'POST',body:{confirmed:true}});await showAppMessage({title:'Kalibrierung gestartet',message:r.steps.map((s,i)=>`${i+1}. ${s}`).join('\n'),icon:'✓',tone:'success'});loadEvents();}catch(e){showToast(e.message,'error');}}
async function runHardwareTest(action){if(!await askConfirmation({title:'Hardwaretest starten?',message:'Gefahrenbereich frei? Der Test bewegt Komponenten kurz und stoppt danach automatisch.',icon:'!',tone:'danger',confirmLabel:'Test starten'}))return;try{await api('/api/maintenance/test',{method:'POST',body:{confirmed:true,action}});showToast('Test gestartet · automatischer Stopp','warning');loadEvents();}catch(e){showToast(e.message,'error');}}
async function stageFirmware(){
  const file=$('firmware-file')?.files?.[0];
  if(!file)return showToast('Bitte eine .hex- oder .bin-Datei auswählen','warning');
  if(!await askConfirmation({title:'Firmware bereitstellen?',message:`Die Datei „${file.name}“ wird auf dem Raspberry Pi geprüft und bereitgestellt. Sie wird noch nicht geflasht.`,icon:'⇧',confirmLabel:'Datei bereitstellen'}))return;
  try{
    const response=await fetch('/api/maintenance/firmware/stage',{method:'POST',headers:{'Content-Type':'application/octet-stream','X-Filename':file.name,'X-MV2-Confirmed':'true'},body:file});
    const result=await response.json();if(!response.ok)throw new Error(result.detail||response.statusText);
    showToast('Firmware geprüft und bereitgestellt · noch nicht geflasht','success');loadEvents();
  }catch(e){showToast(e.message,'error');}
}

// Manual drive: command on press, guaranteed stop on release/cancel.
function shapedJoystickAxis(value,deadzone=.10,exponent=1){
  const magnitude=Math.abs(value);if(magnitude<=deadzone)return 0;
  const normalized=(magnitude-deadzone)/(1-deadzone);
  return Math.sign(value)*Math.pow(normalized,exponent);
}
function manualJoystickValues(nx,ny){
  const speedLimit=Math.max(.05,Math.min(.5,Number(appData.settings.manual_speed_limit??.25)));
  const steeringLimit=Math.max(10,Math.min(45,Number(appData.settings.manual_steering_limit_deg??28)));
  const throttle=shapedJoystickAxis(ny,.10,1.12);
  // Exponent > 1 makes the centre substantially finer without removing the
  // configured maximum steering angle at the outer edge.
  const steeringAxis=shapedJoystickAxis(nx,.10,1.65);
  return {speed:-throttle*speedLimit,steering:-steeringAxis*steeringLimit,speedLimit,steeringLimit};
}
function resetTeachJoystick(){
  teachJoystickPointer=null;$('teach-joystick')?.classList.remove('active');
  if($('teach-joystick-knob'))$('teach-joystick-knob').style.transform='translate(0px,0px)';
  setText('teach-drive-speed','0 %');setText('teach-drive-steering','0°');
}
async function stopTeachJoystick(){
  const wasActive=teachJoystickPointer!==null;resetTeachJoystick();
  if((wasActive||teachRecording)&&authenticated&&commandOnline){try{await api('/api/manual/stop',{method:'POST'});}catch(error){console.warn(error);}}
}
function updateTeachJoystick(event){
  const joystick=$('teach-joystick');if(!joystick||teachJoystickPointer===null)return;
  const rect=joystick.getBoundingClientRect(),cx=rect.left+rect.width/2,cy=rect.top+rect.height/2,maxRadius=Math.max(30,rect.width/2-38);
  let dx=event.clientX-cx,dy=event.clientY-cy;const distance=Math.hypot(dx,dy);if(distance>maxRadius){dx=dx/distance*maxRadius;dy=dy/distance*maxRadius;}
  const nx=dx/maxRadius,ny=dy/maxRadius;
  $('teach-joystick-knob').style.transform=`translate(${dx.toFixed(1)}px,${dy.toFixed(1)}px)`;
  const {speed,steering,speedLimit}=manualJoystickValues(nx,ny);
  setText('teach-drive-speed',`${Math.round(speed/speedLimit*100)} %`);setText('teach-drive-steering',`${Math.round(steering)}°`);
  const now=Date.now();if(now-teachJoystickLastSend<80)return;teachJoystickLastSend=now;
  api('/api/manual/drive',{method:'POST',body:{speed,steering}}).catch(error=>{showToast(error.message,'error');stopTeachJoystick();});
}
function setupTeachJoystick(){
  const joystick=$('teach-joystick');if(!joystick||joystick.dataset.ready)return;joystick.dataset.ready='1';
  joystick.addEventListener('pointerdown',event=>{
    if(!teachRecording||joystick.classList.contains('disabled')||!commandOnline)return;
    event.preventDefault();teachJoystickPointer=event.pointerId;teachJoystickLastSend=0;joystick.setPointerCapture(event.pointerId);joystick.classList.add('active');updateTeachJoystick(event);
  });
  joystick.addEventListener('pointermove',event=>{if(event.pointerId===teachJoystickPointer){event.preventDefault();updateTeachJoystick(event);}});
  const release=event=>{if(teachJoystickPointer===null||event.pointerId!==teachJoystickPointer)return;stopTeachJoystick();};
  joystick.addEventListener('pointerup',release);joystick.addEventListener('pointercancel',release);joystick.addEventListener('lostpointercapture',()=>stopTeachJoystick());
}

function resetDiagnosticJoystick(){
  diagnosticJoystickPointer=null;$('diagnostic-joystick')?.classList.remove('active');
  if($('diagnostic-joystick-knob'))$('diagnostic-joystick-knob').style.transform='translate(0px,0px)';
  setText('diagnostic-drive-speed','0 %');setText('diagnostic-drive-steering','0°');
}
async function stopDiagnosticJoystick(){
  const wasActive=diagnosticJoystickPointer!==null;resetDiagnosticJoystick();
  if(wasActive&&authenticated&&commandOnline){try{await api('/api/manual/stop',{method:'POST'});}catch(error){console.warn(error);}}
}
function updateDiagnosticJoystick(event){
  const joystick=$('diagnostic-joystick');if(!joystick||diagnosticJoystickPointer===null)return;
  const rect=joystick.getBoundingClientRect(),cx=rect.left+rect.width/2,cy=rect.top+rect.height/2,maxRadius=Math.max(30,rect.width/2-38);
  let dx=event.clientX-cx,dy=event.clientY-cy;const distance=Math.hypot(dx,dy);if(distance>maxRadius){dx=dx/distance*maxRadius;dy=dy/distance*maxRadius;}
  const nx=dx/maxRadius,ny=dy/maxRadius;
  $('diagnostic-joystick-knob').style.transform=`translate(${dx.toFixed(1)}px,${dy.toFixed(1)}px)`;
  const {speed,steering,speedLimit}=manualJoystickValues(nx,ny);
  setText('diagnostic-drive-speed',`${Math.round(speed/speedLimit*100)} %`);setText('diagnostic-drive-steering',`${Math.round(steering)}°`);
  const now=Date.now();if(now-diagnosticJoystickLastSend<80)return;diagnosticJoystickLastSend=now;
  api('/api/manual/drive',{method:'POST',body:{speed,steering}}).catch(error=>{showToast(error.message,'error');stopDiagnosticJoystick();});
}
function setupDiagnosticJoystick(){
  const joystick=$('diagnostic-joystick');if(!joystick||joystick.dataset.ready)return;joystick.dataset.ready='1';
  joystick.addEventListener('pointerdown',event=>{
    if(!manualControlAllowed()||joystick.classList.contains('disabled'))return;
    event.preventDefault();diagnosticJoystickPointer=event.pointerId;diagnosticJoystickLastSend=0;joystick.setPointerCapture(event.pointerId);joystick.classList.add('active');updateDiagnosticJoystick(event);
  });
  joystick.addEventListener('pointermove',event=>{if(event.pointerId===diagnosticJoystickPointer){event.preventDefault();updateDiagnosticJoystick(event);}});
  const release=event=>{if(diagnosticJoystickPointer===null||event.pointerId!==diagnosticJoystickPointer)return;stopDiagnosticJoystick();};
  joystick.addEventListener('pointerup',release);joystick.addEventListener('pointercancel',release);joystick.addEventListener('lostpointercapture',()=>stopDiagnosticJoystick());
}
function initDiagnosticManualSettings(){
  const speed=$('diagnostic-speed-limit'),steering=$('diagnostic-steering-limit');
  if(speed){speed.value=String(Math.round(Number(appData.settings.manual_speed_limit??.25)*100));setText('diagnostic-speed-limit-value',`${speed.value} %`);if(!speed.dataset.ready){speed.dataset.ready='1';speed.addEventListener('input',()=>{appData.settings.manual_speed_limit=Number(speed.value)/100;setText('diagnostic-speed-limit-value',`${speed.value} %`);});speed.addEventListener('change',()=>saveSettingsPatch({manual_speed_limit:Number(speed.value)/100}));}}
  if(steering){steering.value=String(Math.round(Number(appData.settings.manual_steering_limit_deg??28)));setText('diagnostic-steering-limit-value',`${steering.value}°`);if(!steering.dataset.ready){steering.dataset.ready='1';steering.addEventListener('input',()=>{appData.settings.manual_steering_limit_deg=Number(steering.value);setText('diagnostic-steering-limit-value',`${steering.value}°`);});steering.addEventListener('change',()=>saveSettingsPatch({manual_steering_limit_deg:Number(steering.value)}));}}
}
async function sendManualImplement(patch){
  if(!manualControlAllowed())return showToast('Manuelle Steuerung ist in diesem Zustand gesperrt','warning');
  const confirmed=patch.blade_enabled===true?await askConfirmation({title:'Messer einschalten?',message:'Gefahrenbereich frei? Das Messer wird im manuellen Diagnosebetrieb eingeschaltet.',icon:'!',tone:'danger',confirmLabel:'Messer einschalten'}):false;
  if(patch.blade_enabled===true&&!confirmed)return;
  const result=await command('/api/manual/implement',{...patch,confirmed});if(!result)return;
  appData.telemetry=appData.telemetry||{};appData.telemetry.outputs=appData.telemetry.outputs||{};Object.assign(appData.telemetry.outputs,result);renderDiagnostics();
}
function setupDiagnosticManualControls(){
  setupDiagnosticJoystick();
  $('diagnostic-front-deck')?.addEventListener('click',()=>sendManualImplement({front_raised:!Boolean(appData.telemetry?.outputs?.front_deck_raised)}));
  $('diagnostic-rear-deck')?.addEventListener('click',()=>sendManualImplement({rear_raised:!Boolean(appData.telemetry?.outputs?.rear_deck_raised)}));
  $('diagnostic-blade')?.addEventListener('click',()=>sendManualImplement({blade_enabled:!Boolean(appData.telemetry?.outputs?.blade_enabled)}));
  $('diagnostic-manual-stop')?.addEventListener('click',()=>command('/api/soft-stop',null,'Fahrt und Messer gestoppt'));
}
window.addEventListener('blur',()=>{resetTeachJoystick();resetDiagnosticJoystick();if(authenticated&&commandOnline)command('/api/manual/stop');});

// ----------------------------------------------------------------------
// Boot
// ----------------------------------------------------------------------
initMap();
setupTeachJoystick();
setupDiagnosticManualControls();
updateUI('IDLE');
renderWizDots();
if('serviceWorker' in navigator) navigator.serviceWorker.register('/static/sw.js').catch(()=>{});
initAuth();
