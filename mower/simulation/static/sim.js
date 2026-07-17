'use strict';

const byId=id=>document.getElementById(id);
let world=null;
let busy=false;

function toast(message,error=false){const el=byId('toast');el.textContent=message;el.className=error?'show error':'show';clearTimeout(toast.timer);toast.timer=setTimeout(()=>el.className='',2600);}
async function request(path,options={}){const init={...options,headers:{...(options.headers||{})}};if(init.body&&typeof init.body!=='string'){init.headers['Content-Type']='application/json';init.body=JSON.stringify(init.body);}const response=await fetch(path,init);if(!response.ok){const detail=await response.json().catch(()=>({detail:response.statusText}));throw new Error(detail.detail||response.statusText);}return response.json();}
function text(id,value){const el=byId(id);if(el)el.textContent=value;}
function gpsLabel(fix){return fix===4?'RTK Fix':fix===5?'RTK Float':fix===1?'GPS':'Kein Fix';}
function formatFactor(value){return `${Number(value).toLocaleString('de-DE',{maximumFractionDigits:2})}×`;}
function render(state,status){world=state;text('position',`${state.x_m.toFixed(2)} / ${state.y_m.toFixed(2)} m`);text('speed',`${(state.speed_mps*3.6).toFixed(2)} km/h`);text('heading',`${((90-state.heading_rad*180/Math.PI)+360)%360|0}°`);text('soc',`${state.soc_percent.toFixed(0)} %`);text('gps',`${gpsLabel(state.gps_fix_quality)} · ${state.gps_hdop}`);text('blade',state.blade_running?'Läuft':'Aus');text('app-state',status.state);text('error-flags',`Flags 0x${state.error_flags.toString(16).padStart(2,'0').toUpperCase()}`);text('soc-input-value',`${Math.round(state.soc_percent)} %`);text('pitch-input-value',`${state.pitch_deg.toFixed(0)}°`);text('roll-input-value',`${state.roll_deg.toFixed(0)}°`);const factor=Number(state.simulation_speed_factor||1),speedInput=byId('simulation-speed-input');text('simulation-speed-value',formatFactor(factor));if(!speedInput.matches(':active'))speedInput.value=String(factor);byId('connection').textContent='Verbunden';byId('connection').className='pill';}
async function refresh(){if(busy)return;try{const [state,status]=await Promise.all([request('/api/simulation/state'),request('/api/status')]);render(state,status);}catch(error){byId('connection').textContent='Getrennt';byId('connection').className='pill pending';}}
async function patch(values,message='Sensorzustand aktualisiert'){busy=true;try{world=await request('/api/simulation/sensors',{method:'PUT',body:values});toast(message);await refresh();}catch(error){toast(error.message,true);}finally{busy=false;}}

document.querySelectorAll('[data-patch]').forEach(button=>button.addEventListener('click',()=>patch(JSON.parse(button.dataset.patch))));
document.querySelectorAll('[data-error]').forEach(button=>button.addEventListener('click',()=>patch({error_flags:(world?.error_flags||0)|Number(button.dataset.error)},'Fehler ausgelöst')));
document.querySelector('[data-toggle="charging"]').addEventListener('click',()=>patch({charging:!world?.charging},world?.charging?'Laden beendet':'Laden gestartet'));
byId('clear-errors').addEventListener('click',()=>patch({error_flags:0,lifted:false,rain_adc:250},'Fehlerbits und Auslöser gelöscht'));
byId('operator-reset').addEventListener('click',async()=>{busy=true;try{await request('/api/reset',{method:'POST',body:{confirmed:true}});toast('Bedienfehler zurückgesetzt');await refresh();}catch(error){toast(error.message,true);}finally{busy=false;}});
byId('reset-world').addEventListener('click',async()=>{busy=true;try{await request('/api/simulation/reset',{method:'POST'});toast('Simulationswelt zurückgesetzt');await refresh();}catch(error){toast(error.message,true);}finally{busy=false;}});
[['soc-input','soc_percent'],['pitch-input','pitch_deg'],['roll-input','roll_deg']].forEach(([id,key])=>{const input=byId(id);input.addEventListener('input',()=>text(`${id}-value`,`${input.value}${key==='soc_percent'?' %':'°'}`));input.addEventListener('change',()=>patch({[key]:Number(input.value)}));});
const simulationSpeedInput=byId('simulation-speed-input');
simulationSpeedInput.addEventListener('input',()=>text('simulation-speed-value',formatFactor(simulationSpeedInput.value)));
simulationSpeedInput.addEventListener('change',async()=>{busy=true;try{world=await request('/api/simulation/speed',{method:'PUT',body:{factor:Number(simulationSpeedInput.value)}});text('simulation-speed-value',formatFactor(world.simulation_speed_factor));toast(`Simulation läuft mit ${formatFactor(world.simulation_speed_factor)}`);}catch(error){toast(error.message,true);}finally{busy=false;}});

refresh();
setInterval(refresh,500);
