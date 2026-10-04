// MicroLink Web Interface Client JS - Lightweight & Resilient
let isFetching = false;
let statusTimer = null;

function showAlert(msg, isSuccess = true) {
    const alertBox = document.getElementById('alertBox');
    if (!alertBox) return;
    alertBox.textContent = msg;
    alertBox.className = 'alert ' + (isSuccess ? 'alert-success' : 'alert-error');
    alertBox.style.display = 'block';
    setTimeout(() => { alertBox.style.display = 'none'; }, 3500);
}

// -------------------------------------------------------------
// Status Polling (with isFetching guard & 3s interval)
// -------------------------------------------------------------
async function updateStatus() {
    if (isFetching || document.hidden) return;
    isFetching = true;
    try {
        const res = await fetch('/api/status');
        if (!res.ok) return;
        const data = await res.json();

        // WiFi
        const wifiEl = document.getElementById('st_wifi');
        if (wifiEl) {
            wifiEl.textContent = `${data.wifi.state} (${data.wifi.ssid} ${data.wifi.rssi}dBm)`;
        }

        // EchoLink
        const elStateEl = document.getElementById('st_echolink');
        if (elStateEl) {
            elStateEl.textContent = data.echolink.state;
            elStateEl.className = 'status-value badge ' + (data.echolink.state === 'LOGGED_IN' ? 'badge-success' : (data.echolink.state === 'CONNECTING_DIR' ? 'badge-warning' : 'badge-danger'));
        }

        // Link LED
        const ledEl = document.getElementById('st_link_led');
        if (ledEl) {
            ledEl.textContent = data.link_led || 'Idle';
        }

        // Connected Station
        const stationEl = document.getElementById('st_station');
        if (stationEl) {
            if (data.station.callsign && data.station.callsign.length > 0) {
                stationEl.textContent = `${data.station.callsign}${data.station.node > 0 ? ' (#' + data.station.node + ')' : ''} - ${data.station.state}`;
            } else {
                stationEl.textContent = data.station.state;
            }
        }

        // TX / RX Badges
        const txEl = document.getElementById('st_tx');
        if (txEl) {
            txEl.textContent = data.tx_active ? 'TRANSMITTING' : 'IDLE';
            txEl.className = 'status-value badge ' + (data.tx_active ? 'badge-danger' : 'badge-info');
        }

        const rxEl = document.getElementById('st_rx');
        if (rxEl) {
            rxEl.textContent = data.rx_active ? 'RECEIVING AUDIO' : 'IDLE';
            rxEl.className = 'status-value badge ' + (data.rx_active ? 'badge-success' : 'badge-info');
        }

        // Mic Level Meter
        const micBar = document.getElementById('mic_fill');
        const micVal = document.getElementById('mic_val');
        if (micBar && micVal && data.mic_level) {
            const pct = Math.min(100, Math.max(0, data.mic_level.pct));
            micBar.style.width = pct + '%';
            micVal.textContent = `Raw: ${data.mic_level.raw} (${pct}%)`;
        }

        // RX Level Meter
        const rxBar = document.getElementById('rx_fill');
        const rxVal = document.getElementById('rx_val');
        if (rxBar && rxVal && data.rx_level) {
            const r_pct = Math.min(100, Math.max(0, data.rx_level.pct));
            rxBar.style.width = r_pct + '%';
            rxVal.textContent = `Peak: ${data.rx_level.raw} (${r_pct}%)`;
        }

        // Uptime & Memory
        const heapEl = document.getElementById('st_heap');
        if (heapEl) heapEl.textContent = `${Math.round(data.free_heap / 1024)} KB`;
        const uptimeEl = document.getElementById('st_uptime');
        if (uptimeEl) {
            const s = data.uptime;
            const h = Math.floor(s / 3600);
            const m = Math.floor((s % 3600) / 60);
            const sec = s % 60;
            uptimeEl.textContent = `${h}h ${m}m ${sec}s`;
        }

        // Jitter Buffer & Loopback status
        const jbDepthEl = document.getElementById('jb_depth');
        if (jbDepthEl && data.jitter) {
            jbDepthEl.textContent = `${data.jitter.depth} frames (${data.jitter.depth * 20} ms)`;
        }
        const jbErrEl = document.getElementById('jb_errors');
        if (jbErrEl && data.jitter) {
            jbErrEl.textContent = `${data.jitter.underflows} / ${data.jitter.overflows}`;
        }
        const btnLoop = document.getElementById('btn_loopback');
        if (btnLoop) {
            if (data.loopback_active) {
                btnLoop.textContent = "Stop Real-time Loopback";
                btnLoop.className = "btn btn-danger";
            } else {
                btnLoop.textContent = "Start Real-time Loopback Test";
                btnLoop.className = "btn btn-primary";
            }
        }
    } catch (e) {
        // silent catch
    } finally {
        isFetching = false;
    }
}

// -------------------------------------------------------------
// Connect / Disconnect Handlers
// -------------------------------------------------------------
async function connectStation(nodeVal, callVal) {
    const node = parseInt(nodeVal, 10) || 0;
    const callsign = (callVal || '').trim();

    if (!node && !callsign) {
        showAlert("Please enter a Node number or Callsign", false);
        return;
    }

    try {
        const res = await fetch('/api/connect', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ node: node, callsign: callsign })
        });
        const resp = await res.json();
        showAlert(resp.message, resp.success);
        if (resp.success) updateStatus();
    } catch (e) {
        showAlert("Failed to connect", false);
    }
}

async function disconnectStation() {
    try {
        const res = await fetch('/api/disconnect', { method: 'POST' });
        const resp = await res.json();
        showAlert(resp.message, resp.success);
        if (resp.success) updateStatus();
    } catch (e) {
        showAlert("Failed to disconnect", false);
    }
}

async function toggleLoopback() {
    try {
        const res = await fetch('/api/loopback', { method: 'POST' });
        const resp = await res.json();
        if (resp.success) {
            showAlert(resp.loopback_active ? "Loopback Started!" : "Loopback Stopped.");
            updateStatus();
        }
    } catch (e) {
        showAlert("Failed to toggle loopback", false);
    }
}

// -------------------------------------------------------------
// Settings Management
// -------------------------------------------------------------
function toggleProxyFields() {
    const el = document.getElementById('proxy_enabled');
    const pf = document.getElementById('proxy_fields');
    if (el && pf) {
        pf.style.display = el.checked ? 'block' : 'none';
    }
}

function onSelectPublicProxy(val) {
    if (!val) return;
    const parts = val.split(':');
    if (document.getElementById('proxy_host')) document.getElementById('proxy_host').value = parts[0];
    if (document.getElementById('proxy_port')) document.getElementById('proxy_port').value = parts[1] || 8100;
    if (document.getElementById('proxy_password')) document.getElementById('proxy_password').value = 'PUBLIC';
}

async function loadSettings() {
    try {
        const res = await fetch('/api/config');
        if (!res.ok) return;
        const cfg = await res.json();

        if (document.getElementById('wifi_ssid')) document.getElementById('wifi_ssid').value = cfg.wifi_ssid || '';
        if (document.getElementById('callsign')) document.getElementById('callsign').value = cfg.callsign || '';
        if (document.getElementById('station_name')) document.getElementById('station_name').value = cfg.station_name || '';
        if (document.getElementById('location')) document.getElementById('location').value = cfg.location || '';
        if (document.getElementById('web_user')) document.getElementById('web_user').value = cfg.web_user || 'admin';
        if (document.getElementById('proxy_enabled')) document.getElementById('proxy_enabled').checked = !!cfg.proxy_enabled;
        if (document.getElementById('proxy_host')) document.getElementById('proxy_host').value = cfg.proxy_host || '';
        if (document.getElementById('proxy_port')) document.getElementById('proxy_port').value = cfg.proxy_port || 8100;
        if (document.getElementById('proxy_password')) document.getElementById('proxy_password').value = cfg.proxy_password || '';
        toggleProxyFields();
    } catch (e) {
        showAlert("Failed to load settings", false);
    }
}

async function saveSettings(event) {
    event.preventDefault();
    const payload = {
        wifi_ssid: document.getElementById('wifi_ssid').value.trim(),
        wifi_pass: document.getElementById('wifi_pass').value,
        callsign: document.getElementById('callsign').value.trim().toUpperCase(),
        el_password: document.getElementById('el_password').value,
        station_name: document.getElementById('station_name').value.trim(),
        location: document.getElementById('location').value.trim(),
        web_user: document.getElementById('web_user').value.trim(),
        web_pass: document.getElementById('web_pass').value,
        proxy_enabled: document.getElementById('proxy_enabled') ? document.getElementById('proxy_enabled').checked : false,
        proxy_host: document.getElementById('proxy_host') ? document.getElementById('proxy_host').value.trim() : '',
        proxy_port: document.getElementById('proxy_port') ? parseInt(document.getElementById('proxy_port').value) || 8100 : 8100,
        proxy_password: document.getElementById('proxy_password') ? document.getElementById('proxy_password').value.trim() : ''
    };

    try {
        const res = await fetch('/api/config', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
        });
        const resp = await res.json();
        showAlert(resp.message || "Settings saved!", resp.success);
        if (resp.success) {
            setTimeout(() => { location.reload(); }, 1500);
        }
    } catch (e) {
        showAlert("Failed to save settings", false);
    }
}

// -------------------------------------------------------------
// Favorites Management
// -------------------------------------------------------------
async function loadFavorites() {
    const listEl = document.getElementById('favorites_list');
    if (!listEl) return;

    try {
        const res = await fetch('/api/favorites');
        const favs = await res.json();

        listEl.innerHTML = '';
        if (!Array.isArray(favs) || favs.length === 0) {
            listEl.innerHTML = '<p style="color:var(--text-muted);font-size:0.85rem;">No saved favorites.</p>';
            return;
        }

        favs.forEach((fav, index) => {
            const item = document.createElement('div');
            item.className = 'fav-item';
            item.innerHTML = `
                <div>
                    <strong>${fav.call}</strong> <span style="color:#8899a6;font-size:0.8rem;">#${fav.node}</span>
                    ${fav.desc ? '<div style="font-size:0.75rem;color:#8899a6;">' + fav.desc + '</div>' : ''}
                </div>
                <div style="display:flex;gap:4px;">
                    <button class="btn-preset" onclick="connectStation(${fav.node}, '${fav.call}')">Connect</button>
                    <button class="btn-preset" style="color:#c62828;" onclick="deleteFavorite(${index})">&times;</button>
                </div>
            `;
            listEl.appendChild(item);
        });
    } catch (e) {
        listEl.innerHTML = '<p style="color:#c62828;font-size:0.85rem;">Error loading favorites.</p>';
    }
}

async function addFavorite(event) {
    event.preventDefault();
    const node = document.getElementById('fav_node').value;
    const call = document.getElementById('fav_call').value;
    const desc = document.getElementById('fav_desc').value;

    try {
        const getRes = await fetch('/api/favorites');
        let favs = await getRes.json();
        if (!Array.isArray(favs)) favs = [];

        favs.push({ node: parseInt(node, 10), call: call.toUpperCase(), desc: desc });

        const postRes = await fetch('/api/favorites', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(favs)
        });

        if (postRes.ok) {
            showAlert("Favorite added!");
            document.getElementById('fav_node').value = '';
            document.getElementById('fav_call').value = '';
            document.getElementById('fav_desc').value = '';
            loadFavorites();
        }
    } catch (e) {
        showAlert("Failed to save favorite", false);
    }
}

async function deleteFavorite(index) {
    try {
        const getRes = await fetch('/api/favorites');
        let favs = await getRes.json();
        favs.splice(index, 1);

        const postRes = await fetch('/api/favorites', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(favs)
        });

        if (postRes.ok) {
            loadFavorites();
        }
    } catch (e) {
        showAlert("Failed to delete favorite", false);
    }
}

// Visibility change listener: pause polling when tab is hidden
document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
        if (statusTimer) { clearInterval(statusTimer); statusTimer = null; }
    } else {
        updateStatus();
        if (statusTimer) clearInterval(statusTimer);
        statusTimer = setInterval(updateStatus, 3000);
    }
});
