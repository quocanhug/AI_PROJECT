// ============================================================
//  RoboControl — AI Robot Pathfinder Dashboard
//  script.js — Tabs, Map, Algorithms, WebSocket, Telemetry
// ============================================================

let currentMode = "manual";
let timeChartInstance = null;
let statsInterval = null;

// ==================== Clock ====================
function updateClock() {
  const now = new Date();
  const ts = now.toLocaleTimeString('en-US', { hour:'2-digit', minute:'2-digit', second:'2-digit', hour12:true });
  const el = document.getElementById('real-time-clock');
  if (el) el.textContent = ts;
  const ft = document.getElementById('footer-update-time');
  if (ft) ft.textContent = ts + ' (GMT+7)';
}
setInterval(updateClock, 1000);
updateClock();

// ==================== Toast ====================
let toastContainer;
function ensureToastContainer() {
  if (!toastContainer) {
    toastContainer = document.createElement('div');
    toastContainer.className = 'toast-container';
    document.body.appendChild(toastContainer);
  }
}
function showToast(msg, type = "info") {
  ensureToastContainer();
  const t = document.createElement("div");
  t.className = "toast " + type;
  t.textContent = msg;
  toastContainer.appendChild(t);
  setTimeout(() => { t.style.opacity="0"; t.style.transform="translateX(20px)"; setTimeout(() => t.remove(), 300); }, 3000);
}

// ==================== WebSocket ====================
let ws = null, wsReconnectTimer = null, pingTimer = null, lastPingTime = 0;

function connectWebSocket() {
  const host = window.location.hostname || "192.168.4.1";
  const port = window.location.port || "80";
  try { ws = new WebSocket(`ws://${host}:${port}/ws`); } catch(e) { scheduleReconnect(); return; }

  ws.onopen = () => {
    setWsStatus(true);
    showToast("Kết nối ESP32 thành công", "success");
    if (wsReconnectTimer) { clearTimeout(wsReconnectTimer); wsReconnectTimer = null; }
    pingTimer = setInterval(() => { lastPingTime = Date.now(); try { ws.send(JSON.stringify({type:"PING"})); } catch(e){} }, 5000);
  };
  ws.onmessage = (evt) => { try { handleWsMessage(JSON.parse(evt.data)); } catch(e){} };
  ws.onclose = () => { setWsStatus(false); if (pingTimer) { clearInterval(pingTimer); pingTimer=null; } scheduleReconnect(); };
  ws.onerror = () => setWsStatus(false);
}

function scheduleReconnect() { if (wsReconnectTimer) return; wsReconnectTimer = setTimeout(() => { wsReconnectTimer=null; connectWebSocket(); }, 3000); }

function setWsStatus(connected) {
  document.getElementById('wsStatusText').textContent = connected ? 'Trực tuyến' : 'Mất kết nối';
  document.getElementById('wsStatusDot').className = 'w-2 h-2 rounded-full ' + (connected ? 'bg-green-400 animate-pulse' : 'bg-yellow-400');
  document.getElementById('connBadge').textContent = connected ? 'Kết nối ổn định' : 'Đang kết nối...';
  document.getElementById('connBadge').className = 'px-3 py-1 text-[10px] font-bold rounded-full uppercase tracking-widest border ' +
    (connected ? 'bg-green-100 text-green-700 border-green-200' : 'bg-yellow-100 text-yellow-700 border-yellow-200');
}

function wsSend(obj) { if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj)); }

// ==================== WS Message Handler ====================
const speedHistory = new Array(20).fill(0);

function handleWsMessage(data) {
  if (data.type === "TELEMETRY") updateTelemetry(data);
  else if (data.type === "ROUTE_ACK") showToast(`Route nạp: ${data.commands} lệnh`, "success");
  else if (data.type === "OBSTACLE_DETECTED") {
    showToast(`⚠️ Vật cản tại (${data.position.row},${data.position.col})!`, "error");
    updateRobotState("OBSTACLE");
  }
  else if (data.type === "COMPLETED") { showToast(`Hoàn thành! ${data.intersections} giao lộ, ${data.distance_cm}cm`, "success"); updateRobotState("DONE"); }
  else if (data.type === "PONG" || data.type === "PING") { /* ping handled */ }
}

function updateTelemetry(d) {
  const avg = (Math.abs(d.speedL||0) + Math.abs(d.speedR||0)) / 2;
  const avgCms = avg * 100;

  // Manual tab
  document.getElementById('teleSpeedVal').textContent = avg.toFixed(2);
  document.getElementById('speedBar').style.width = Math.min(avg / 1.5 * 100, 100) + '%';
  document.getElementById('teleDistVal').textContent = (d.distance||0).toFixed(1);
  document.getElementById('teleStateVal').textContent = d.state || 'IDLE';
  document.getElementById('teleStepVal').textContent = `${d.step||0} / ${d.total||0}`;

  const obs = d.obstacle || 999;
  document.getElementById('sonarStatus').textContent = obs > 200 ? '-- cm' : obs.toFixed(1) + ' cm';
  document.getElementById('sonarStatus').className = 'text-xs font-bold ' + (obs < 20 ? 'text-error' : 'text-green-600');

  // Auto tab
  document.getElementById('autoSpeedVal').textContent = avg.toFixed(1);
  document.getElementById('autoObstacleVal').textContent = obs > 200 ? '--' : obs.toFixed(0);

  // Sensors
  if (d.sensors) {
    for (let i = 0; i < 5; i++) {
      const el = document.getElementById('sm' + i);
      if (el) el.className = 'w-3 h-3 rounded-full ' + (d.sensors[i] ? 'bg-primary shadow-[0_0_6px_rgba(0,90,180,0.5)]' : 'bg-surface-container');
    }
  }
  if (d.state) updateRobotState(d.state);
}

function updateRobotState(state) {
  const map = { "IDLE":"IDLE", "FOLLOWING_LINE":"DÒ LINE", "AT_INTERSECTION":"GIAO LỘ", "OBSTACLE":"VẬT CẢN", "REROUTING":"TÁI ĐỊNH TUYẾN", "DONE":"HOÀN THÀNH" };
  document.getElementById('teleStateVal').textContent = map[state] || state;
}

// ==================== Tab Switching ====================
async function switchTab(tab) {
  document.querySelectorAll('.panel-view').forEach(el => el.classList.remove('active'));
  document.querySelectorAll('.nav-link').forEach(el => el.classList.remove('active-nav'));
  document.getElementById('panel-' + tab).classList.add('active');
  document.getElementById('nav-' + tab).classList.add('active-nav');
  currentMode = tab;

  const titles = { manual: 'Hệ thống: Vận hành Thủ công', auto: 'Hệ thống: Chế độ Tự động', stats: 'Mission Control Center — Thống kê' };
  document.getElementById('headerTitle').textContent = titles[tab];

  if (statsInterval) { clearInterval(statsInterval); statsInterval = null; }
  if (tab !== "stats") {
    try { await fetch("/setMode?m=" + (tab === "manual" ? "manual" : "line")); } catch(e) {}
  } else {
    fetchRealStats();
    statsInterval = setInterval(fetchRealStats, 3000);
  }
}

// ==================== Stats ====================
async function fetchRealStats() {
  try {
    const r = await fetch("/api/stats");
    if (r.ok) {
      const d = await r.json();
      document.getElementById('kpiDelivered').textContent = d.delivered;
      document.getElementById('kpiAvgTime').textContent = d.avgTime;
      document.getElementById('kpiSuccess').textContent = d.efficiency;
      document.getElementById('kpiDistance').textContent = d.totalDistance || '--';
      if (d.chart) updateChart(d.chart.labels, d.chart.expected, d.chart.actual);
    }
  } catch(e) {}
}

function updateChart(labels, expected, actual) {
  const ctx = document.getElementById('timeComparisonChart').getContext('2d');
  if (timeChartInstance) { timeChartInstance.data.labels=labels; timeChartInstance.data.datasets[0].data=expected; timeChartInstance.data.datasets[1].data=actual; timeChartInstance.update(); return; }
  timeChartInstance = new Chart(ctx, {
    type:'bar', data:{ labels, datasets:[
      { label:'Dự kiến (s)', data:expected, backgroundColor:'#aac7ff', borderRadius:6 },
      { label:'Thực tế (s)', data:actual, backgroundColor:'#005ab4', borderRadius:6 }
    ]},
    options:{ responsive:true, maintainAspectRatio:false, animation:{duration:500},
      scales:{ y:{beginAtZero:true, grid:{color:'#e1e3e4'}, ticks:{color:'#414753',font:{size:10}}}, x:{grid:{display:false}, ticks:{color:'#414753',font:{size:10}}} },
      plugins:{ legend:{position:'top', labels:{color:'#191c1d',font:{size:11,family:'Inter'},boxWidth:12}} }
    }
  });
}

// ==================== Map & Graph ====================
const startNodeEl = document.getElementById("startNode");
for (let i = 0; i < 20; i++) {
  let opt = document.createElement("option");
  opt.value = i; opt.innerText = "Node " + i;
  if (i === 15) opt.selected = true;
  startNodeEl.appendChild(opt);
}

const graph = {
  0:[1,5], 1:[0,6], 2:[3,7], 3:[2,4,8], 4:[3],
  5:[0,6,10], 6:[1,5,7], 7:[2,6,8,12], 8:[3,7,9,13], 9:[8,14],
  10:[5,11,15], 11:[10,16], 12:[7,13,17], 13:[8,12,14,18], 14:[9,13],
  15:[10,16], 16:[11,15], 17:[12,18], 18:[13,17,19], 19:[18]
};
const edgeWeights = {};
function getEdgeWeight(a,b) { return edgeWeights[Math.min(a,b)+'-'+Math.max(a,b)] || 1; }

const coords = {
  0:[30,30],1:[100,30],2:[170,30],3:[240,30],4:[310,30],
  5:[30,100],6:[100,100],7:[170,100],8:[240,100],9:[310,100],
  10:[30,170],11:[100,170],12:[170,170],13:[240,170],14:[310,170],
  15:[30,240],16:[100,240],17:[170,240],18:[240,240],19:[310,240]
};
const TARGET_COLORS = ["#ba1a1a","#964400","#465f89","#0873df","#bd5700","#2e4770"];
let targets = new Set([4]), obstacles = new Set(), finalCalculatedPath = [];
let currentMapTool = 'target';

function setMapTool(tool) {
  currentMapTool = tool;
  document.getElementById('toolTarget').classList.toggle('active', tool === 'target');
  document.getElementById('toolObstacle').classList.toggle('active', tool === 'obstacle');
}

function initNodes() {
  const layer = document.getElementById("nodeLayer");
  layer.innerHTML = "";
  for (let i = 0; i < 20; i++) {
    const [x,y] = coords[i];
    const g = document.createElementNS("http://www.w3.org/2000/svg","g");
    g.setAttribute("class","node-group"); g.setAttribute("id","svg-node-"+i);
    g.onclick = () => handleNodeClick(i);
    const c = document.createElementNS("http://www.w3.org/2000/svg","circle");
    c.setAttribute("cx",x); c.setAttribute("cy",y); c.setAttribute("r","12"); c.setAttribute("class","node-circle");
    const t = document.createElementNS("http://www.w3.org/2000/svg","text");
    t.setAttribute("x",x); t.setAttribute("y",y); t.setAttribute("class","node-text"); t.textContent = i;
    g.appendChild(c); g.appendChild(t); layer.appendChild(g);
  }
}

function handleNodeClick(nid) {
  const sn = parseInt(document.getElementById("startNode").value);
  const mode = document.getElementById("orderModeSel").value;
  if (nid === sn) return;

  if (currentMapTool === "target") {
    obstacles.delete(nid);
    if (mode === "single") { targets.clear(); targets.add(nid); }
    else { if (targets.has(nid)) targets.delete(nid); else { if (targets.size >= 6) { showToast("Tối đa 6 đơn!","error"); return; } targets.add(nid); } }
  } else {
    targets.delete(nid);
    if (obstacles.has(nid)) obstacles.delete(nid); else obstacles.add(nid);
  }
  calculateOverallPath();
}

// ==================== AI Algorithms ====================
function heuristic(a,b) { return Math.abs(coords[a][0]-coords[b][0]) + Math.abs(coords[a][1]-coords[b][1]); }

function findPath(start, end, algo) {
  if (obstacles.has(start) || obstacles.has(end)) return null;
  algo = algo || document.getElementById("algoSel").value;

  if (algo === "bfs") {
    let q = [[start]], vis = new Set([start]);
    while (q.length) { let p=q.shift(), n=p[p.length-1]; if (n===end) return {path:p,visitedCount:vis.size}; for (let nx of graph[n]) if (!vis.has(nx)&&!obstacles.has(nx)) { vis.add(nx); q.push([...p,nx]); } }
  } else if (algo === "dfs") {
    let s = [[start]], vis = new Set([start]);
    while (s.length) { let p=s.pop(), n=p[p.length-1]; if (n===end) return {path:p,visitedCount:vis.size}; for (let nx of graph[n]) if (!vis.has(nx)&&!obstacles.has(nx)) { vis.add(nx); s.push([...p,nx]); } }
  } else if (algo === "ucs") {
    let pq = [{path:[start],cost:0}], vis = new Map(); vis.set(start,0);
    while (pq.length) { pq.sort((a,b)=>a.cost-b.cost); let c=pq.shift(), n=c.path[c.path.length-1]; if (n===end) return {path:c.path,visitedCount:vis.size,cost:c.cost}; for (let nx of graph[n]) { if (obstacles.has(nx)) continue; let nc=c.cost+getEdgeWeight(n,nx); if (!vis.has(nx)||nc<vis.get(nx)) { vis.set(nx,nc); pq.push({path:[...c.path,nx],cost:nc}); } } }
  } else if (algo === "greedy") {
    let pq = [{path:[start],h:heuristic(start,end)}], vis = new Set([start]);
    while (pq.length) { pq.sort((a,b)=>a.h-b.h); let c=pq.shift(), n=c.path[c.path.length-1]; if (n===end) return {path:c.path,visitedCount:vis.size}; for (let nx of graph[n]) if (!vis.has(nx)&&!obstacles.has(nx)) { vis.add(nx); pq.push({path:[...c.path,nx],h:heuristic(nx,end)}); } }
  } else { // astar
    let pq = [{path:[start],g:0,f:heuristic(start,end)}], vis = new Map(); vis.set(start,0);
    while (pq.length) { pq.sort((a,b)=>a.f-b.f); let c=pq.shift(), n=c.path[c.path.length-1]; if (n===end) return {path:c.path,visitedCount:vis.size,cost:c.g}; for (let nx of graph[n]) { if (obstacles.has(nx)) continue; let ng=c.g+getEdgeWeight(n,nx); if (!vis.has(nx)||ng<vis.get(nx)) { vis.set(nx,ng); pq.push({path:[...c.path,nx],g:ng,f:ng+heuristic(nx,end)}); } } }
  }
  return null;
}

// ==================== Path Calculation ====================
function calculateOverallPath() {
  const sn = parseInt(document.getElementById("startNode").value);
  for (let i = 0; i < 20; i++) {
    let cls = "node-group";
    if (i === sn) cls += " node-start";
    else if (obstacles.has(i)) cls += " node-obstacle";
    document.getElementById("svg-node-"+i).setAttribute("class",cls);
  }
  const pathLayer = document.getElementById("pathLayer");
  pathLayer.innerHTML = "";
  const resEl = document.getElementById("pathResult");
  const legendBox = document.getElementById("deliveryLegend");

  if (!targets.size) {
    resEl.innerText = "Chọn điểm giao hàng trên bản đồ"; resEl.className = "p-4 text-sm text-on-surface-variant text-center";
    legendBox.classList.add("hidden"); finalCalculatedPath = [];
    document.getElementById("autoStepsVal").innerHTML = '0 <span class="text-xs font-medium text-on-surface-variant italic">bước</span>';
    document.getElementById("autoVisitedVal").textContent = '0';
    return;
  }

  let fullPath = [sn], curS = sn, rem = new Set(targets), totalSteps=0, totalVis=0;
  let ordTar = [], segs = [];

  while (rem.size) {
    let best=null, bestT=null, shortLen=Infinity;
    for (let t of rem) { let r=findPath(curS,t); if (r && r.path.length < shortLen) { shortLen=r.path.length; best=r; bestT=t; } }
    if (!best) break;
    ordTar.push(bestT); segs.push(best.path);
    fullPath = fullPath.length > 1 ? fullPath.concat(best.path.slice(1)) : best.path;
    totalSteps += best.path.length-1; totalVis += best.visitedCount;
    curS = bestT; rem.delete(bestT);
  }

  if (ordTar.length) {
    finalCalculatedPath = fullPath;
    let msg = "LỘ TRÌNH: " + fullPath.join(" ➔ ");
    resEl.className = "p-4 text-sm text-center has-path";

    if (rem.size) {
      msg += "\n⚠️ BỎ QUA: " + Array.from(rem).join(", ") + " (vướng vật cản)";
      resEl.className = "p-4 text-sm text-center warn-msg";
      Array.from(rem).forEach(t => document.getElementById("svg-node-"+t).classList.add("node-skipped"));
    }
    resEl.innerText = msg;

    document.getElementById("autoStepsVal").innerHTML = totalSteps + ' <span class="text-xs font-medium text-on-surface-variant italic">bước</span>';
    document.getElementById("autoVisitedVal").textContent = totalVis;
    document.getElementById("autoOrderNum").textContent = '#01/' + (ordTar.length < 10 ? '0'+ordTar.length : ordTar.length);

    let legHTML = '<div class="w-full text-center font-bold text-xs text-on-surface-variant mb-1">THỨ TỰ GIAO HÀNG</div>';
    ordTar.forEach((node, idx) => {
      let ci = Math.min(idx,5), col = TARGET_COLORS[ci];
      document.getElementById("svg-node-"+node).classList.add("target-"+(ci+1));
      let pts = segs[idx].map(n => coords[n].join(",")).join(" ");
      let poly = document.createElementNS("http://www.w3.org/2000/svg","polyline");
      poly.setAttribute("class","segment-path"); poly.setAttribute("points",pts);
      poly.setAttribute("stroke",col); poly.setAttribute("style",`filter:drop-shadow(0 0 4px ${col}40)`);
      pathLayer.appendChild(poly);
      legHTML += `<span class="flex items-center gap-1 px-2 py-1 bg-surface-container rounded-lg"><span class="w-2 h-2 rounded-full" style="background:${col}"></span>Đơn ${idx+1} (N${node})</span>`;
      if (idx < ordTar.length-1) legHTML += '<span class="text-outline-variant">➔</span>';
    });
    legendBox.innerHTML = legHTML; legendBox.classList.remove("hidden");
    for (let i=1; i<fullPath.length-1; i++) { if (!targets.has(fullPath[i])) document.getElementById("svg-node-"+fullPath[i]).classList.add("node-path"); }
  } else {
    resEl.innerText = "LỖI: Không tìm được đường! Vật cản chặn toàn bộ."; resEl.className = "p-4 text-sm text-center err-msg";
    legendBox.classList.add("hidden"); finalCalculatedPath = [];
    targets.forEach(t => document.getElementById("svg-node-"+t).classList.add("target-1"));
  }
}

// ==================== Path to Commands ====================
function pathToCommands(path, initialDir) {
  let dir = parseInt(initialDir || 1);
  const cmds = [];
  const dd = [{dx:0,dy:-1},{dx:1,dy:0},{dx:0,dy:1},{dx:-1,dy:0}];
  for (let i=0; i<path.length-1; i++) {
    const dx = Math.sign(coords[path[i+1]][0]-coords[path[i]][0]);
    const dy = Math.sign(coords[path[i+1]][1]-coords[path[i]][1]);
    let td = -1;
    for (let d=0; d<4; d++) if (dd[d].dx===dx && dd[d].dy===dy) { td=d; break; }
    if (td===-1) { cmds.push("F"); continue; }
    const diff = (td-dir+4)%4;
    if (diff===1) cmds.push("R"); else if (diff===3) cmds.push("L"); else if (diff===2) { cmds.push("R"); cmds.push("R"); } else cmds.push("F");
    dir = td;
  }
  return cmds;
}

// ==================== Deliver Button ====================
document.getElementById("deliverBtn").addEventListener("click", async () => {
  if (finalCalculatedPath.length < 2) return showToast("Lộ trình không hợp lệ!","error");
  const dir = document.getElementById("startDir").value;
  const cmds = pathToCommands(finalCalculatedPath, dir);
  const algo = document.getElementById("algoSel").value;
  const btn = document.getElementById("deliverBtn");
  btn.innerHTML = '<span class="material-symbols-outlined text-sm animate-spin">progress_activity</span> ĐANG TRUYỀN...';

  if (ws && ws.readyState === WebSocket.OPEN) {
    wsSend({ type:"ROUTE", commands:cmds, total_steps:cmds.length, algorithm:algo, path:finalCalculatedPath });
  } else {
    try { await fetch(`/deliver?dir=${dir}&path=${finalCalculatedPath.join(",")}`); } catch(e) {}
  }

  setTimeout(() => {
    btn.innerHTML = '<span class="material-symbols-outlined text-sm">check_circle</span> ĐÃ NẠP!';
    showToast("Lộ trình đã được gửi đến robot","success");
    setTimeout(() => { btn.innerHTML = '<span class="material-symbols-outlined text-sm">play_circle</span> NẠP LỘ TRÌNH & BẮT ĐẦU'; }, 2000);
  }, 1000);
});

// ==================== Algorithm Comparison ====================
document.getElementById("compareAllBtn").addEventListener("click", () => {
  const sn = parseInt(document.getElementById("startNode").value);
  const tarArr = Array.from(targets);
  if (!tarArr.length) { showToast("Chọn điểm giao!","error"); return; }
  const algos = ["bfs","dfs","ucs","astar","greedy"];
  const names = {bfs:"BFS",dfs:"DFS",ucs:"UCS",astar:"A*",greedy:"Greedy"};
  const results = [];

  for (let algo of algos) {
    const t0 = performance.now();
    let tv=0,tc=0,pl=0,ok=true,cs=sn;
    for (let t of tarArr) { const r=findPath(cs,t,algo); if (r) { tv+=r.visitedCount; tc+=(r.cost||r.path.length-1); pl+=r.path.length-1; cs=t; } else { ok=false; break; } }
    results.push({ algo, name:names[algo], visited:tv, pathLen:pl, cost:tc, time:(performance.now()-t0).toFixed(2), success:ok });
  }

  let html = `<table class="w-full text-left text-xs border-separate border-spacing-y-1 mt-2">
    <thead><tr class="text-[10px] uppercase font-bold text-on-surface-variant tracking-widest">
      <th class="px-2 py-1">Thuật toán</th><th class="px-2 py-1 text-center">Node</th><th class="px-2 py-1 text-center">Bước</th><th class="px-2 py-1 text-center">ms</th>
    </tr></thead><tbody>`;
  for (let r of results) {
    html += `<tr class="bg-surface-container-low/40 rounded"><td class="px-2 py-2 font-bold text-primary">${r.name}</td>
      <td class="text-center px-2 py-2">${r.success?r.visited:'❌'}</td><td class="text-center px-2 py-2">${r.success?r.pathLen:'-'}</td>
      <td class="text-center px-2 py-2">${r.time}</td></tr>`;
  }
  html += '</tbody></table>';
  document.getElementById("algoCompareResult").innerHTML = html;
  showToast("So sánh hoàn tất!","success");
});

// ==================== Event Listeners ====================
document.querySelectorAll("#startNode, #algoSel, #orderModeSel").forEach(el =>
  el.addEventListener("change", () => {
    if (el.id === "orderModeSel" && el.value === "single" && targets.size > 1) { const f = Array.from(targets)[0]; targets.clear(); targets.add(f); }
    obstacles.delete(parseInt(document.getElementById("startNode").value));
    calculateOverallPath();
  })
);

// ==================== Manual D-Pad ====================
async function sendCommand(cmd) { try { await fetch(cmd); showToast("Lệnh: "+cmd,"info"); } catch(e){} }
async function send(path) { try { await fetch(path); } catch(e){} }

let activeHold = { btn:null, pid:null };
function guard(fn) { return (e) => { if (currentMode !== "manual") { e.preventDefault(); return; } return fn(e); }; }

document.querySelectorAll(".hold").forEach(btn => {
  btn.addEventListener("pointerdown", guard(e => {
    e.preventDefault(); activeHold = {btn, pid:e.pointerId};
    btn.classList.add("active-hold"); btn.setPointerCapture(e.pointerId);
    send(btn.dataset.path);
  }), {passive:false});
  const release = guard(e => {
    e.preventDefault();
    if (activeHold.btn===btn && activeHold.pid===e.pointerId) {
      btn.classList.remove("active-hold"); send("/stop"); activeHold = {btn:null,pid:null};
    }
    try { btn.releasePointerCapture(e.pointerId); } catch(_) {}
  });
  btn.addEventListener("pointerup", release, {passive:false});
  btn.addEventListener("pointercancel", release, {passive:false});
});

document.getElementById("stopBtn").addEventListener("pointerdown", guard(e => { e.preventDefault(); send("/stop"); }), {passive:false});

// ==================== Init ====================
initNodes();
calculateOverallPath();
setTimeout(() => connectWebSocket(), 1000);
