#include "web_ui.h"
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "config_manager.h"
#include "wifi_manager.h"
#include "system_state.h"
#include "echolink_client.h"
#include "dtmf_controller.h"
#include "announcer.h"
#include "link_led.h"

static WebServer s_server(80);
static bool s_fs_mounted = false;

// Embedded Lightweight Multi-Tab Web UI (PROGMEM fallback when LittleFS is not uploaded)
// Stripped of heavy CSS transitions, gradients, and bloat to maximize ESP32 stability
static const char EMBEDDED_UI_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>MicroLink Node</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;font-family:sans-serif;}
body{background:#111822;color:#e0e6ed;padding:12px;max-width:520px;margin:auto;font-size:14px;}
header{text-align:center;margin-bottom:12px;}
header h1{color:#00bcd4;font-size:1.5rem;letter-spacing:1px;}
header p{color:#8899a6;font-size:0.8rem;}
nav{display:flex;gap:6px;margin-bottom:12px;}
nav a{flex:1;padding:8px;text-align:center;text-decoration:none;color:#8899a6;background:#18222e;border:1px solid #2d3b4d;border-radius:4px;font-weight:bold;font-size:0.85rem;}
nav a.active{background:#00bcd4;color:#000;border-color:#00bcd4;}
.tab-content{display:none;}
.tab-content.active{display:block;}
.card{background:#18222e;border:1px solid #2d3b4d;border-radius:6px;padding:12px;margin-bottom:12px;}
.card-title{font-size:0.95rem;color:#00bcd4;margin-bottom:10px;border-bottom:1px solid #2d3b4d;padding-bottom:4px;}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;}
.label{font-size:0.7rem;color:#8899a6;}
.val{font-size:0.9rem;font-weight:bold;}
.badge{display:inline-block;padding:2px 6px;border-radius:3px;font-size:0.75rem;font-weight:bold;}
.badge-ok{background:#2e7d32;color:#fff;}
.badge-warn{background:#f57f17;color:#fff;}
.badge-err{background:#c62828;color:#fff;}
.badge-info{background:#0277bd;color:#fff;}
.meter-bg{background:#0c1219;height:10px;border-radius:4px;overflow:hidden;border:1px solid #2d3b4d;margin-top:4px;}
.meter-fill{background:#00bcd4;height:100%;width:0%;}
.form-group{margin-bottom:10px;}
.form-group label{display:block;font-size:0.75rem;color:#8899a6;margin-bottom:3px;}
input{width:100%;padding:8px 10px;background:#0c1219;border:1px solid #2d3b4d;border-radius:4px;color:#fff;font-size:0.85rem;}
input:focus{outline:none;border-color:#00bcd4;}
.btn{width:100%;padding:8px;border:none;border-radius:4px;font-weight:bold;cursor:pointer;font-size:0.85rem;}
.btn-primary{background:#00bcd4;color:#000;}
.btn-danger{background:#c62828;color:#fff;}
.btn-preset{background:#0c1219;border:1px solid #2d3b4d;color:#00bcd4;padding:4px 8px;border-radius:3px;cursor:pointer;font-size:0.8rem;}
.fav-item{display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid #2d3b4d;}
.fav-item:last-child{border-bottom:none;}
.alert{padding:8px;border-radius:4px;margin-bottom:10px;display:none;font-size:0.8rem;}
.alert-ok{background:#2e7d32;color:#fff;display:block;}
.alert-err{background:#c62828;color:#fff;display:block;}
</style>
</head>
<body>
<header>
<h1>MICROLINK</h1>
<p>ESP32-C6 EchoLink Node</p>
</header>
<nav>
<a id="tabnav_status" href="/" onclick="switchTab('status');return false;" class="active">Status</a>
<a id="tabnav_connect" href="/connect.html" onclick="switchTab('connect');return false;">Connect</a>
<a id="tabnav_settings" href="/settings.html" onclick="switchTab('settings');return false;">Settings</a>
</nav>
<div id="alertBox" class="alert"></div>

<!-- TAB 1: STATUS -->
<div id="tab_status" class="tab-content active">
<div class="card">
<h2 class="card-title">Live System Status</h2>
<div class="grid">
<div><div class="label">WiFi Network</div><div id="st_wifi" class="val">Connecting...</div></div>
<div><div class="label">EchoLink Status</div><div id="st_echolink" class="val badge badge-warn">CHECKING</div></div>
<div><div class="label">Link LED</div><div id="st_link_led" class="val badge badge-info">Idle</div></div>
<div><div class="label">Connected Station</div><div id="st_station" class="val">IDLE</div></div>
<div><div class="label">Transmitter (TX)</div><div id="st_tx" class="val badge badge-info">IDLE</div></div>
<div><div class="label">Receiver (RX)</div><div id="st_rx" class="val badge badge-info">IDLE</div></div>
<div><div class="label">Operating Mode</div><div id="st_mode" class="val badge badge-info">PTT</div></div>
<div><div class="label">VOX Sensitivity</div><div id="st_vox_sens" class="val">10 / 20</div></div>
<div><div class="label">TOT (60 s limit)</div><div id="st_tot" class="val badge badge-info">OK</div></div>
<div><div class="label">Free Heap RAM</div><div id="st_heap" class="val">---</div></div>
</div>
<div style="margin-top:10px;">
<div class="label">Microphone Input Level (KY-038)</div>
<div class="meter-bg"><div id="mic_fill" class="meter-fill"></div></div>
<p id="mic_val" style="font-size:0.75rem;color:#8899a6;margin-top:3px;">Raw: 0 (0%)</p>
</div>
<div style="margin-top:10px;">
<div class="label">Receiver Audio Level (RX)</div>
<div class="meter-bg"><div id="rx_fill" class="meter-fill" style="background:#28a745;"></div></div>
<p id="rx_val" style="font-size:0.75rem;color:#8899a6;margin-top:3px;">Peak: 0 (0%)</p>
</div>
<div style="margin-top:10px;font-size:0.75rem;color:#8899a6;text-align:right;">
Uptime: <span id="st_uptime">0h 0m 0s</span>
</div>
</div>

<div class="card">
<h2 class="card-title">Audio Pipeline & Jitter Buffer (8 kHz)</h2>
<div class="grid">
<div><div class="label">Jitter Depth</div><div id="jb_depth" class="val">0 frames</div></div>
<div><div class="label">Underflows / Drops</div><div id="jb_errors" class="val">0 / 0</div></div>
</div>
<div style="margin-top:10px;">
<button id="btn_loopback" class="btn btn-primary" onclick="toggleLoopback()">Start Real-time Loopback Test</button>
</div>
</div>
</div>

<!-- TAB 2: CONNECT -->
<div id="tab_connect" class="tab-content">
<div class="card">
<h2 class="card-title">Connect EchoLink Station</h2>
<div class="form-group">
<label>Node Number</label>
<input type="number" id="target_node" placeholder="e.g. 9999 or 123456">
</div>
<div class="form-group">
<label>Callsign (Optional)</label>
<input type="text" id="target_call" placeholder="e.g. *ECHOTEST*">
</div>
<div style="display:flex;gap:8px;margin-top:10px;">
<button class="btn btn-primary" onclick="doConnect()">Connect</button>
<button class="btn btn-danger" onclick="doDisconnect()">Disconnect</button>
</div>
</div>

<div class="card">
<h2 class="card-title">Quick Connect & Favorites</h2>
<div style="display:flex;gap:6px;flex-wrap:wrap;margin-bottom:10px;">
<button class="btn-preset" onclick="quickConnect(9999,'*ECHOTEST*')">*ECHOTEST* #9999</button>
<button class="btn-preset" onclick="quickConnect(123456,'*THAILAND*')">*THAILAND* #123456</button>
<button class="btn-preset" onclick="quickConnect(211880,'*HS1AB-L*')">*HS1AB-L* #211880</button>
</div>
<div id="fav_list" style="font-size:0.85rem;">Loading...</div>
<form onsubmit="addFav(event)" style="margin-top:10px;border-top:1px solid #2d3b4d;padding-top:10px;">
<div style="font-size:0.8rem;color:#8899a6;margin-bottom:6px;">Add Favorite</div>
<div style="display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-bottom:6px;">
<input type="number" id="fav_node" placeholder="Node #" required>
<input type="text" id="fav_call" placeholder="Callsign" required>
</div>
<input type="text" id="fav_desc" placeholder="Description" style="margin-bottom:6px;">
<button type="submit" class="btn btn-primary" style="padding:6px;">+ Save</button>
</form>
</div>
</div>

<!-- TAB 3: SETTINGS -->
<div id="tab_settings" class="tab-content">
<form onsubmit="saveSettings(event)">
<div class="card">
<h2 class="card-title">WiFi Configuration</h2>
<div class="form-group">
<label>WiFi SSID</label>
<input type="text" id="cfg_wifi_ssid" required>
</div>
<div class="form-group">
<label>WiFi Password</label>
<input type="password" id="cfg_wifi_pass" placeholder="Leave blank to keep existing">
</div>
</div>

<div class="card">
<h2 class="card-title">EchoLink Station Identity</h2>
<div class="form-group">
<label>Station Callsign (e.g. HS1ABC-L)</label>
<input type="text" id="cfg_callsign" required>
</div>
<div class="form-group">
<label>EchoLink Password</label>
<input type="password" id="cfg_el_password" placeholder="EchoLink account password">
</div>
<div class="form-group">
<label>Sysop Name</label>
<input type="text" id="cfg_station_name">
</div>
<div class="form-group">
<label>Location / QTH / Freq</label>
<input type="text" id="cfg_location">
</div>
<div class="card">
<h2 class="card-title">EchoLink Proxy (Public / NAT Bypass)</h2>
<div class="form-group">
<label style="display:flex;align-items:center;gap:8px;cursor:pointer;">
<input type="checkbox" id="cfg_proxy_enabled" onchange="toggleProxyFields()" style="width:auto;margin:0;">
<span>Enable EchoLink Proxy</span>
</label>
<p style="font-size:0.75rem;color:#8899a6;margin-top:3px;">
Bypasses firewalls and CGNAT (4G/5G mobile hotspot, hotel/dorm Wi-Fi) without port forwarding.
</p>
</div>
<div id="proxy_fields" style="display:none;">
<div class="form-group">
<label>Quick Select Public Proxy</label>
<select id="cfg_public_proxy_sel" onchange="onSelectPublicProxy(this.value)">
<option value="">-- Custom or Select Public Proxy --</option>
<option value="154.64.217.174:8100">WP4CL4 (154.64.217.174:8100) - Ready</option>
<option value="44.76.15.47:8100">WX5FWD-47 (44.76.15.47:8100) - Ready</option>
<option value="154.64.217.180:8100">WP4CL10 (154.64.217.180:8100)</option>
<option value="172.105.105.151:8100">E25ZDO Asia/Thailand (172.105.105.151:8100)</option>
<option value="139.162.58.118:8100">JA1ZLO Japan (139.162.58.118:8100)</option>
<option value="45.79.147.214:8100">US East (45.79.147.214:8100)</option>
</select>
</div>
<div class="form-group">
<label>Proxy Server Host / IP</label>
<input type="text" id="cfg_proxy_host" placeholder="e.g. 154.64.217.174">
</div>
<div class="form-group">
<label>Proxy Port</label>
<input type="number" id="cfg_proxy_port" placeholder="8100" value="8100">
</div>
<div class="form-group">
<label>Proxy Password</label>
<input type="password" id="cfg_proxy_password" placeholder="Leave blank for default PUBLIC">
</div>
</div>
</div>

<div class="card">
<h2 class="card-title">Web Security</h2>
<div class="form-group">
<label>Admin Username</label>
<input type="text" id="cfg_web_user" required>
</div>
<div class="form-group">
<label>Admin Password</label>
<input type="password" id="cfg_web_pass" placeholder="Leave blank to keep existing">
</div>
</div>

<button type="submit" class="btn btn-primary" style="margin-bottom:16px;">Save Configuration</button>
</form>
</div>

<script>
var isFetching=false;
var statusTimer=null;

function showAlert(msg,isErr){
var b=document.getElementById('alertBox');
if(!b)return;
b.textContent=msg;
b.className='alert '+(isErr?'alert-err':'alert-ok');
setTimeout(function(){b.className='alert';b.textContent='';},3500);
}

function switchTab(name){
document.querySelectorAll('.tab-content').forEach(function(e){e.classList.remove('active');});
document.querySelectorAll('nav a').forEach(function(e){e.classList.remove('active');});
var c=document.getElementById('tab_'+name);
var b=document.getElementById('tabnav_'+name);
if(c)c.classList.add('active');
if(b)b.classList.add('active');
if(window.history&&window.history.pushState){
var p=(name==='status')?'/':'/'+name+'.html';
window.history.pushState(null,'',p);
}
}

function updateStatus(){
if(isFetching||document.hidden)return;
isFetching=true;
fetch('/api/status')
.then(function(r){return r.json();})
.then(function(d){
var w=document.getElementById('st_wifi');
if(w)w.textContent=d.wifi.state+' ('+d.wifi.ssid+' '+d.wifi.rssi+'dBm)';
var el=document.getElementById('st_echolink');
if(el){
el.textContent=d.echolink.state;
el.className='val badge '+(d.echolink.state==='LOGGED_IN'?'badge-ok':(d.echolink.state==='CONNECTING_DIR'?'badge-warn':'badge-err'));
}
var led=document.getElementById('st_link_led');
if(led)led.textContent=d.link_led||'Idle';
var st=document.getElementById('st_station');
if(st){
st.textContent=(d.station.callsign&&d.station.callsign.length>0)?(d.station.callsign+(d.station.node>0?' #'+d.station.node:'')+' - '+d.station.state):d.station.state;
}
var tx=document.getElementById('st_tx');
if(tx){
var txSrc=['','PTT','VOX','ANN','WEB'];
var txLabel=d.tx_active?(txSrc[d.tx_source]||'TX')+' ON':'IDLE';
tx.textContent=txLabel;
tx.className='val badge '+(d.tx_active?'badge-err':'badge-info');
}
var rx=document.getElementById('st_rx');
if(rx){
rx.textContent=d.rx_active?'AUDIO RX':'IDLE';
rx.className='val badge '+(d.rx_active?'badge-ok':'badge-info');
}
var sm=document.getElementById('st_mode');
if(sm){
var modeLabel=d.op_mode===1?'VOX':'PTT';
sm.textContent=modeLabel;
sm.className='val badge '+(d.op_mode===1?'badge-warn':'badge-info');
}
var ss=document.getElementById('st_vox_sens');
if(ss)ss.textContent=(d.op_mode===1)?('Level '+d.vox_sensitivity+' / 20'):'-';
var tot=document.getElementById('st_tot');
if(tot){
tot.textContent=d.tot_triggered?'FIRED':'OK';
tot.className='val badge '+(d.tot_triggered?'badge-err':'badge-info');
}
var hp=document.getElementById('st_heap');
if(hp)hp.textContent=Math.round(d.free_heap/1024)+' KB';
var ut=document.getElementById('st_uptime');
if(ut){
var u=d.uptime_sec;
ut.textContent=Math.floor(u/3600)+'h '+Math.floor((u%3600)/60)+'m '+(u%60)+'s';
}
var mf=document.getElementById('mic_fill');
var mv=document.getElementById('mic_val');
if(mf&&mv){
var pct=Math.min(Math.max(d.mic_level.pct,0),100);
mf.style.width=pct+'%';
mv.textContent='Raw: '+d.mic_level.raw+' ('+pct+'%)';
}
var rf=document.getElementById('rx_fill');
var rv=document.getElementById('rx_val');
if(rf&&rv&&d.rx_level){
var r_pct=Math.min(Math.max(d.rx_level.pct,0),100);
rf.style.width=r_pct+'%';
rv.textContent='Peak: '+d.rx_level.raw+' ('+r_pct+'%)';
}
var jd=document.getElementById('jb_depth');
if(jd)jd.textContent=d.jitter.depth+' frames ('+(d.jitter.depth*20)+' ms)';
var je=document.getElementById('jb_errors');
if(je)je.textContent=d.jitter.underflows+' / '+d.jitter.overflows;
var btn=document.getElementById('btn_loopback');
if(btn){
btn.textContent=d.loopback_active?'Stop Loopback':'Start Audio Loopback';
btn.className='btn '+(d.loopback_active?'btn-danger':'btn-primary');
}
})
.catch(function(e){})
.finally(function(){
isFetching=false;
});
}

function doConnect(){
var n=parseInt(document.getElementById('target_node').value)||0;
var c=document.getElementById('target_call').value.trim();
if(!n&&!c){showAlert('Enter Node number or Callsign',true);return;}
fetch('/api/connect',{
method:'POST',
headers:{'Content-Type':'application/json'},
body:JSON.stringify({node:n,callsign:c})
})
.then(function(r){return r.json();})
.then(function(d){
showAlert(d.message,!d.success);
if(d.success)switchTab('status');
})
.catch(function(e){showAlert('Connect failed',true);});
}

function doDisconnect(){
fetch('/api/disconnect',{method:'POST'})
.then(function(r){return r.json();})
.then(function(d){showAlert(d.message,!d.success);})
.catch(function(e){showAlert('Disconnect failed',true);});
}

function quickConnect(node,call){
document.getElementById('target_node').value=node;
document.getElementById('target_call').value=call;
doConnect();
}

function toggleLoopback(){
fetch('/api/loopback',{method:'POST'})
.then(function(r){return r.json();})
.then(function(d){updateStatus();})
.catch(function(e){});
}

function toggleProxyFields(){
var en=document.getElementById('cfg_proxy_enabled').checked;
var pf=document.getElementById('proxy_fields');
if(pf)pf.style.display=en?'block':'none';
}

function onSelectPublicProxy(v){
if(!v)return;
var p=v.split(':');
document.getElementById('cfg_proxy_host').value=p[0];
document.getElementById('cfg_proxy_port').value=p[1]||8100;
document.getElementById('cfg_proxy_password').value='PUBLIC';
}

function loadSettings(){
fetch('/api/config')
.then(function(r){return r.json();})
.then(function(d){
document.getElementById('cfg_wifi_ssid').value=d.wifi_ssid||'';
document.getElementById('cfg_callsign').value=d.callsign||'';
document.getElementById('cfg_station_name').value=d.station_name||'';
document.getElementById('cfg_location').value=d.location||'';
document.getElementById('cfg_web_user').value=d.web_user||'admin';
document.getElementById('cfg_proxy_enabled').checked=!!d.proxy_enabled;
document.getElementById('cfg_proxy_host').value=d.proxy_host||'';
document.getElementById('cfg_proxy_port').value=d.proxy_port||8100;
document.getElementById('cfg_proxy_password').value=d.proxy_password||'';
toggleProxyFields();
})
.catch(function(e){});
}

function saveSettings(e){
e.preventDefault();
var payload={
wifi_ssid:document.getElementById('cfg_wifi_ssid').value.trim(),
wifi_pass:document.getElementById('cfg_wifi_pass').value,
callsign:document.getElementById('cfg_callsign').value.trim().toUpperCase(),
el_password:document.getElementById('cfg_el_password').value,
station_name:document.getElementById('cfg_station_name').value.trim(),
location:document.getElementById('cfg_location').value.trim(),
web_user:document.getElementById('cfg_web_user').value.trim(),
web_pass:document.getElementById('cfg_web_pass').value,
proxy_enabled:document.getElementById('cfg_proxy_enabled').checked,
proxy_host:document.getElementById('cfg_proxy_host').value.trim(),
proxy_port:parseInt(document.getElementById('cfg_proxy_port').value)||8100,
proxy_password:document.getElementById('cfg_proxy_password').value.trim()
};
fetch('/api/config',{
method:'POST',
headers:{'Content-Type':'application/json'},
body:JSON.stringify(payload)
})
.then(function(r){return r.json();})
.then(function(d){
showAlert('Settings saved! Registering with EchoLink...',!d.success);
if(d.success)setTimeout(function(){location.reload();},1500);
})
.catch(function(e){showAlert('Save failed',true);});
}

var g_favs=[];
function loadFavorites(){
fetch('/api/favorites')
.then(function(r){return r.json();})
.then(function(d){
g_favs=d||[];
var el=document.getElementById('fav_list');
if(!el)return;
if(g_favs.length===0){
el.innerHTML='<p style="font-size:0.8rem;color:#8899a6;">No favorites saved.</p>';
return;
}
var h='';
g_favs.forEach(function(item,i){
h+='<div class="fav-item"><div><strong>'+item.call+'</strong> <span style="color:#8899a6;font-size:0.8rem;">#'+item.node+'</span>'+(item.desc?'<div style="font-size:0.75rem;color:#8899a6;">'+item.desc+'</div>':'')+'</div><div style="display:flex;gap:4px;"><button class="btn-preset" onclick="quickConnect('+item.node+',\''+item.call+'\')">Connect</button><button class="btn-preset" style="color:#c62828;" onclick="delFav('+i+')">&times;</button></div></div>';
});
el.innerHTML=h;
})
.catch(function(e){});
}

function addFav(e){
e.preventDefault();
var n=parseInt(document.getElementById('fav_node').value)||0;
var c=document.getElementById('fav_call').value.trim().toUpperCase();
var d=document.getElementById('fav_desc').value.trim();
if(!n||!c)return;
g_favs.push({node:n,call:c,desc:d});
fetch('/api/favorites',{
method:'POST',
headers:{'Content-Type':'application/json'},
body:JSON.stringify(g_favs)
})
.then(function(r){return r.json();})
.then(function(){
document.getElementById('fav_node').value='';
document.getElementById('fav_call').value='';
document.getElementById('fav_desc').value='';
loadFavorites();
showAlert('Favorite added!',false);
});
}

function delFav(idx){
g_favs.splice(idx,1);
fetch('/api/favorites',{
method:'POST',
headers:{'Content-Type':'application/json'},
body:JSON.stringify(g_favs)
})
.then(function(){loadFavorites();});
}

window.addEventListener('DOMContentLoaded',function(){
var p=window.location.pathname.toLowerCase();
if(p.indexOf('connect')!==-1)switchTab('connect');
else if(p.indexOf('settings')!==-1)switchTab('settings');
else switchTab('status');
updateStatus();
loadSettings();
loadFavorites();
statusTimer=setInterval(updateStatus,3000);
});

document.addEventListener('visibilitychange',function(){
if(document.hidden){
if(statusTimer){clearInterval(statusTimer);statusTimer=null;}
}else{
updateStatus();
if(statusTimer)clearInterval(statusTimer);
statusTimer=setInterval(updateStatus,3000);
}
});
</script>
</body>
</html>
)rawliteral";

static bool check_auth()
{
    if (wifi_manager_is_ap())
    {
        return true; // Open access during AP configuration
    }

    ConfigData cfg = config_manager_get();
    if (!s_server.authenticate(cfg.web_user, cfg.web_pass))
    {
        s_server.requestAuthentication(BASIC_AUTH, "MicroLink", "Authentication Required");
        return false;
    }
    return true;
}

static void send_json_response(int code, const String &json)
{
    s_server.sendHeader("Connection", "close");
    s_server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    s_server.send(code, "application/json", json);
}

// -------------------------------------------------------------
// API: GET /api/status
// -------------------------------------------------------------
static void handle_api_status()
{
    if (!check_auth())
        return;

    SystemState st = system_state_get();
    JsonDocument doc;

    doc["wifi"]["state"] = wifi_state_str(st.wifi_state);
    doc["wifi"]["ssid"] = wifi_manager_get_ssid();
    doc["wifi"]["ip"] = wifi_manager_get_ip().toString();
    doc["wifi"]["rssi"] = wifi_manager_get_rssi();

    doc["echolink"]["state"] = echolink_state_str(st.echolink_state);

    LinkInputs link_in;
    link_in.apMode = (st.wifi_state == WifiState::AP_MODE);
    link_in.wifiUp = (st.wifi_state == WifiState::CONNECTED);
    link_in.regFailed = st.reg_failed;
    link_in.registered = (st.echolink_state == EchoLinkState::LOGGED_IN);
    link_in.linked = (st.station_state == StationState::CONNECTED);
    doc["link_led"] = linkLedName(pickLinkLed(link_in));

    doc["station"]["state"] = station_state_str(st.station_state);
    doc["station"]["callsign"] = st.connected_callsign;
    doc["station"]["node"] = st.connected_node;

    doc["tx_active"] = st.tx_active;
    doc["rx_active"] = st.rx_active;
    doc["tx_source"] = (int)echolink_client_get_tx_source();
    doc["announcing"] = announcer_busy();

    doc["mic_level"]["raw"] = st.mic_raw_level;
    doc["mic_level"]["pct"] = st.mic_level_pct;

    doc["rx_level"]["raw"] = st.rx_raw_level;
    doc["rx_level"]["pct"] = st.rx_level_pct;

    doc["jitter"]["depth"]      = st.jitter_depth;
    doc["jitter"]["underflows"] = st.jitter_underflows;
    doc["jitter"]["overflows"]  = st.jitter_overflows;
    doc["loopback_active"]      = st.loopback_active;

    doc["op_mode"]         = st.op_mode;          // 0=PTT, 1=VOX
    doc["vox_sensitivity"] = st.vox_sensitivity;  // 1-9
    doc["tot_triggered"]   = st.tot_triggered;    // true when 60 s TOT fired

    doc["free_heap"] = st.free_heap;
    doc["uptime"] = st.uptime_sec;

    String resp;
    serializeJson(doc, resp);
    send_json_response(200, resp);
}

// -------------------------------------------------------------
// API: POST /api/connect (Real EchoLink Backend)
// -------------------------------------------------------------
static void handle_api_connect()
{
    if (!check_auth())
        return;

    if (!s_server.hasArg("plain"))
    {
        send_json_response(400, "{\"success\":false,\"message\":\"Missing body\"}");
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, s_server.arg("plain"));
    if (err)
    {
        send_json_response(400, "{\"success\":false,\"message\":\"Invalid JSON\"}");
        return;
    }

    uint32_t node = doc["node"] | 0;
    const char *call = doc["callsign"] | "";

    char target[32] = {0};
    if (node > 0)
    {
        snprintf(target, sizeof(target), "%lu", (unsigned long)node);
    }
    else if (strlen(call) > 0)
    {
        strncpy(target, call, sizeof(target) - 1);
    }
    else
    {
        send_json_response(400, "{\"success\":false,\"message\":\"No callsign or node specified\"}");
        return;
    }

    bool started = echolink_client_connect(target);
    if (started)
    {
        send_json_response(200, "{\"success\":true,\"message\":\"Connecting to station...\"}");
    }
    else
    {
        send_json_response(500, "{\"success\":false,\"message\":\"Failed to start connection\"}");
    }
}

// -------------------------------------------------------------
// API: POST /api/disconnect (Real EchoLink Backend)
// -------------------------------------------------------------
static void handle_api_disconnect()
{
    if (!check_auth())
        return;

    echolink_client_disconnect();
    send_json_response(200, "{\"success\":true,\"message\":\"Disconnect initiated\"}");
}

// -------------------------------------------------------------
// API: GET /api/config
// -------------------------------------------------------------
static void handle_api_config_get()
{
    if (!check_auth())
        return;

    ConfigData cfg = config_manager_get();
    JsonDocument doc;

    doc["wifi_ssid"] = cfg.wifi_ssid;
    doc["wifi_pass"] = mask_secret(cfg.wifi_pass);
    doc["callsign"] = cfg.callsign;
    doc["el_password"] = mask_secret(cfg.el_password);
    doc["station_name"] = cfg.station_name;
    doc["location"] = cfg.location;
    doc["web_user"] = cfg.web_user;
    doc["web_pass"] = mask_secret(cfg.web_pass);
    doc["proxy_enabled"] = cfg.proxy_enabled;
    doc["proxy_host"] = cfg.proxy_host;
    doc["proxy_port"] = cfg.proxy_port;
    doc["proxy_password"] = mask_secret(cfg.proxy_password);

    String resp;
    serializeJson(doc, resp);
    send_json_response(200, resp);
}

// -------------------------------------------------------------
// API: POST /api/config
// -------------------------------------------------------------
static void handle_api_config_post()
{
    if (!check_auth())
        return;

    if (!s_server.hasArg("plain"))
    {
        send_json_response(400, "{\"success\":false,\"message\":\"Missing body\"}");
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, s_server.arg("plain"));
    if (err)
    {
        send_json_response(400, "{\"success\":false,\"message\":\"Invalid JSON\"}");
        return;
    }

    ConfigData new_cfg = config_manager_get();

    if (doc["wifi_ssid"].is<const char *>())
        strncpy(new_cfg.wifi_ssid, doc["wifi_ssid"], sizeof(new_cfg.wifi_ssid) - 1);
    if (doc["wifi_pass"].is<const char *>())
        strncpy(new_cfg.wifi_pass, doc["wifi_pass"], sizeof(new_cfg.wifi_pass) - 1);
    if (doc["callsign"].is<const char *>())
        strncpy(new_cfg.callsign, doc["callsign"], sizeof(new_cfg.callsign) - 1);
    if (doc["el_password"].is<const char *>())
        strncpy(new_cfg.el_password, doc["el_password"], sizeof(new_cfg.el_password) - 1);
    if (doc["station_name"].is<const char *>())
        strncpy(new_cfg.station_name, doc["station_name"], sizeof(new_cfg.station_name) - 1);
    if (doc["location"].is<const char *>())
        strncpy(new_cfg.location, doc["location"], sizeof(new_cfg.location) - 1);
    if (doc["web_user"].is<const char *>())
        strncpy(new_cfg.web_user, doc["web_user"], sizeof(new_cfg.web_user) - 1);
    if (doc["web_pass"].is<const char *>())
        strncpy(new_cfg.web_pass, doc["web_pass"], sizeof(new_cfg.web_pass) - 1);
    if (doc["proxy_enabled"].is<bool>())
        new_cfg.proxy_enabled = doc["proxy_enabled"].as<bool>();
    if (doc["proxy_host"].is<const char *>())
        strncpy(new_cfg.proxy_host, doc["proxy_host"], sizeof(new_cfg.proxy_host) - 1);
    if (doc["proxy_port"].is<uint16_t>())
        new_cfg.proxy_port = doc["proxy_port"].as<uint16_t>();
    if (doc["proxy_password"].is<const char *>())
        strncpy(new_cfg.proxy_password, doc["proxy_password"], sizeof(new_cfg.proxy_password) - 1);

    config_manager_save(new_cfg);
    echolink_client_trigger_registration();

    send_json_response(200, "{\"success\":true,\"message\":\"Configuration saved\"}");
}

// -------------------------------------------------------------
// API: GET /api/favorites & POST /api/favorites
// -------------------------------------------------------------
static void handle_api_favorites_get()
{
    if (!check_auth())
        return;
    String favs = config_manager_get_favorites();
    send_json_response(200, favs);
}

static void handle_api_favorites_post()
{
    if (!check_auth())
        return;
    if (!s_server.hasArg("plain"))
    {
        send_json_response(400, "{\"success\":false,\"message\":\"Missing body\"}");
        return;
    }

    String favs = s_server.arg("plain");
    config_manager_save_favorites(favs);
    send_json_response(200, "{\"success\":true}");
}

static void handle_api_loopback()
{
    if (!check_auth())
        return;
    bool cur = system_state_get_loopback();
    system_state_set_loopback(!cur);
    send_json_response(200, String("{\"success\":true,\"loopback_active\":") + (!cur ? "true" : "false") + "}");
}

// -------------------------------------------------------------
// API: POST /api/dtmf  {"command":"*19999#"}
// -------------------------------------------------------------
static void handle_api_dtmf()
{
    if (!check_auth())
        return;

    if (!s_server.hasArg("plain"))
    {
        send_json_response(400, "{\"success\":false,\"message\":\"Missing body\"}");
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, s_server.arg("plain"));
    if (err)
    {
        send_json_response(400, "{\"success\":false,\"message\":\"Invalid JSON\"}");
        return;
    }

    const char *cmd = doc["command"] | "";
    if (strlen(cmd) == 0)
    {
        send_json_response(400, "{\"success\":false,\"message\":\"Missing 'command' field\"}");
        return;
    }

    bool ok = dtmf_controller_handle_command(cmd);
    if (ok)
    {
        String resp = String("{\"success\":true,\"command\":\"") + cmd + "\",\"last_action\":\"" + dtmf_controller_get_last_command() + "\"}";
        send_json_response(200, resp);
    }
    else
    {
        send_json_response(400, "{\"success\":false,\"message\":\"Unknown DTMF command\"}");
    }
}

// -------------------------------------------------------------
// API: POST /api/announce  {"text":"connected 12345","route":3}
// route: 1=LOCAL, 2=TX, 3=LOCAL|TX (default)
// -------------------------------------------------------------
static void handle_api_announce()
{
    if (!check_auth())
        return;

    if (!s_server.hasArg("plain"))
    {
        send_json_response(400, "{\"success\":false,\"message\":\"Missing body\"}");
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, s_server.arg("plain"));
    if (err)
    {
        send_json_response(400, "{\"success\":false,\"message\":\"Invalid JSON\"}");
        return;
    }

    const char *text = doc["text"] | "";
    if (strlen(text) == 0)
    {
        send_json_response(400, "{\"success\":false,\"message\":\"Missing 'text' field\"}");
        return;
    }

    uint8_t route = doc["route"] | (uint8_t)(ROUTE_LOCAL | ROUTE_TX);
    Announcer::instance().say(text, route);

    send_json_response(200, "{\"success\":true}");
}

// -------------------------------------------------------------
// Static File Serving & Fallback Handlers
// -------------------------------------------------------------
static bool handle_static_file(const String &path)
{
    if (!s_fs_mounted)
        return false;

    String content_type = "text/plain";
    if (path.endsWith(".html"))
        content_type = "text/html";
    else if (path.endsWith(".css"))
        content_type = "text/css";
    else if (path.endsWith(".js"))
        content_type = "application/javascript";
    else if (path.endsWith(".json"))
        content_type = "application/json";

    if (LittleFS.exists(path))
    {
        File file = LittleFS.open(path, "r");
        s_server.sendHeader("Connection", "close");
        s_server.streamFile(file, content_type);
        file.close();
        return true;
    }
    return false;
}

static void handle_root()
{
    if (!check_auth())
        return;

    if (handle_static_file("/index.html"))
        return;

    // Embedded full multi-tab interface if LittleFS is not uploaded
    s_server.sendHeader("Connection", "close");
    s_server.send_P(200, "text/html", EMBEDDED_UI_HTML);
}

static void handle_not_found()
{
    // Captive portal redirects
    if (wifi_manager_is_ap())
    {
        s_server.sendHeader("Location", String("http://") + wifi_manager_get_ip().toString() + "/", true);
        s_server.sendHeader("Connection", "close");
        s_server.send(302, "text/plain", "");
        return;
    }

    String uri = s_server.uri();
    if (handle_static_file(uri))
        return;

    s_server.sendHeader("Connection", "close");
    s_server.send(404, "text/plain", "404 Not Found");
}

static void handle_save_fallback()
{
    ConfigData cfg = config_manager_get();
    if (s_server.hasArg("wifi_ssid"))
        strncpy(cfg.wifi_ssid, s_server.arg("wifi_ssid").c_str(), sizeof(cfg.wifi_ssid) - 1);
    if (s_server.hasArg("wifi_pass") && s_server.arg("wifi_pass").length() > 0)
        strncpy(cfg.wifi_pass, s_server.arg("wifi_pass").c_str(), sizeof(cfg.wifi_pass) - 1);
    if (s_server.hasArg("callsign"))
        strncpy(cfg.callsign, s_server.arg("callsign").c_str(), sizeof(cfg.callsign) - 1);
    if (s_server.hasArg("el_password") && s_server.arg("el_password").length() > 0)
        strncpy(cfg.el_password, s_server.arg("el_password").c_str(), sizeof(cfg.el_password) - 1);
    if (s_server.hasArg("station_name"))
        strncpy(cfg.station_name, s_server.arg("station_name").c_str(), sizeof(cfg.station_name) - 1);
    if (s_server.hasArg("location"))
        strncpy(cfg.location, s_server.arg("location").c_str(), sizeof(cfg.location) - 1);
    config_manager_save(cfg);
    echolink_client_trigger_registration();

    s_server.sendHeader("Connection", "close");
    s_server.send(200, "text/html", "<h3>Saved! Settings updated.</h3><script>setTimeout(()=>{location.href='/';},2000);</script>");
}

void web_ui_init()
{
    s_fs_mounted = LittleFS.begin(true);
    if (s_fs_mounted)
    {
        Serial.println(F("[WEB UI] LittleFS mounted successfully."));
    }
    else
    {
        Serial.println(F("[WEB UI] LittleFS mount failed; using embedded fallback HTML."));
    }

    // Static page routes
    s_server.on("/", HTTP_GET, handle_root);
    s_server.on("/index.html", HTTP_GET, handle_root);
    s_server.on("/connect.html", HTTP_GET, []()
                {
        if (!check_auth()) return;
        if (!handle_static_file("/connect.html")) {
            s_server.sendHeader("Connection", "close");
            s_server.send_P(200, "text/html", EMBEDDED_UI_HTML);
        } });
    s_server.on("/settings.html", HTTP_GET, []()
                {
        if (!check_auth()) return;
        if (!handle_static_file("/settings.html")) {
            s_server.sendHeader("Connection", "close");
            s_server.send_P(200, "text/html", EMBEDDED_UI_HTML);
        } });
    s_server.on("/style.css", HTTP_GET, []()
                {
        if (!handle_static_file("/style.css")) {
            s_server.sendHeader("Connection", "close");
            s_server.send(200, "text/css", "/* Embedded in HTML */");
        } });
    s_server.on("/app.js", HTTP_GET, []()
                {
        if (!handle_static_file("/app.js")) {
            s_server.sendHeader("Connection", "close");
            s_server.send(200, "application/javascript", "// Embedded in HTML");
        } });

    // REST API routes
    s_server.on("/api/status", HTTP_GET, handle_api_status);
    s_server.on("/api/connect", HTTP_POST, handle_api_connect);
    s_server.on("/api/disconnect", HTTP_POST, handle_api_disconnect);
    s_server.on("/api/config", HTTP_GET, handle_api_config_get);
    s_server.on("/api/config", HTTP_POST, handle_api_config_post);
    s_server.on("/api/favorites", HTTP_GET, handle_api_favorites_get);
    s_server.on("/api/favorites", HTTP_POST, handle_api_favorites_post);
    s_server.on("/api/loopback", HTTP_POST, handle_api_loopback);
    s_server.on("/api/dtmf", HTTP_POST, handle_api_dtmf);
    s_server.on("/api/announce", HTTP_POST, handle_api_announce);

    // Fallback form handler
    s_server.on("/save_fallback", HTTP_POST, handle_save_fallback);

    // Captive portal probes
    s_server.on("/generate_204", HTTP_GET, handle_root);
    s_server.on("/hotspot-detect.html", HTTP_GET, handle_root);

    s_server.onNotFound(handle_not_found);

    s_server.begin();
    Serial.println(F("[WEB UI] HTTP Server listening on port 80."));
}

void web_ui_process()
{
    s_server.handleClient();
}
