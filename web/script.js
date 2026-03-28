// ============================================================
//  RoboControl — AI Robot Pathfinder Dashboard
//  script.js — Algorithms, Animation, Edge Editor, WebSocket
// ============================================================

let currentMode = "manual";
let algoChartInstance = null;
let statsInterval = null;
let deliveryStartTime = null;
let deliveryExpectedTime = null;
let deliveryInProgress = false;

// ==================== Biến Khóa An Toàn ====================
let isRouteValidated = false;

function updateValidationUI(isValid) {
  isRouteValidated = isValid;
  const btn1 = document.getElementById("deliverBtn");
  const btn2 = document.getElementById("deliverBtn2");
  if (!btn1 || !btn2) return;

  if (isValid) {
    btn1.innerHTML = "▶ NẠP LỘ TRÌNH & BẮT ĐẦU";
    btn1.className =
      "text-xs font-bold text-primary bg-primary/10 px-4 py-2 rounded-lg flex items-center gap-2 hover:bg-primary/20 transition-colors";
    btn2.innerHTML = "🚀 NẠP LỘ TRÌNH & CHO XE CHẠY";
    btn2.className =
      "w-full flex items-center justify-center gap-2 bg-green-600 text-white px-4 py-3 rounded-xl text-sm font-bold shadow-lg shadow-green-600/20 hover:bg-green-700 transition-all active:scale-[0.98]";
  } else {
    btn1.innerHTML = "⚠️ CHƯA MÔ PHỎNG";
    btn1.className =
      "text-xs font-bold text-gray-500 bg-gray-200 px-4 py-2 rounded-lg flex items-center gap-2 cursor-not-allowed";
    btn2.innerHTML = "⚠️ YÊU CẦU CHẠY MÔ PHỎNG TRƯỚC";
    btn2.className =
      "w-full flex items-center justify-center gap-2 bg-gray-400 text-white px-4 py-3 rounded-xl text-sm font-bold shadow-lg cursor-not-allowed";
  }
}

// ==================== Graph Data ====================
let graph = {
  0: [],
  1: [],
  2: [],
  3: [],
  4: [],
  5: [],
  6: [],
  7: [],
  8: [],
  9: [],
  10: [],
  11: [],
  12: [],
  13: [],
  14: [],
  15: [],
  16: [],
  17: [],
  18: [],
  19: [],
};
const edgeWeights = {};
const coords = {
  0: [30, 30],
  1: [100, 30],
  2: [170, 30],
  3: [240, 30],
  4: [310, 30],
  5: [30, 100],
  6: [100, 100],
  7: [170, 100],
  8: [240, 100],
  9: [310, 100],
  10: [30, 170],
  11: [100, 170],
  12: [170, 170],
  13: [240, 170],
  14: [310, 170],
  15: [30, 240],
  16: [100, 240],
  17: [170, 240],
  18: [240, 240],
  19: [310, 240],
};
const TARGET_COLORS = [
  "#ba1a1a",
  "#964400",
  "#465f89",
  "#0873df",
  "#bd5700",
  "#2e4770",
];
let targets = new Set([4]),
  obstacles = new Set(),
  finalCalculatedPath = [];
let currentMapTool = "target";
let edgeSelFirst = null;

let anim = {
  steps: [],
  path: [],
  currentStep: 0,
  playing: false,
  speed: 300,
  timer: null,
  active: false,
};

// ==================== Robot Indicator ====================
let robotIndicator = null;
let robotCurrentNode = -1;
let robotCurrentDir = 2;
let robotInitialDir = 2;
const DIR_ROTATION = [0, 90, 180, 270];
const JS_TO_CPP_DIR = [2, 1, 0, 3];
const CPP_TO_JS_DIR = [2, 1, 0, 3];
const DIR_LABELS = ["⬆ Lên", "➡ Phải", "⬇ Xuống", "⬅ Trái"];

function createRobotIndicator() {
  const layer = document.getElementById("robotLayer");
  if (!layer) return;
  layer.innerHTML = "";
  const g = document.createElementNS("http://www.w3.org/2000/svg", "g");
  g.id = "robotArrow";
  g.setAttribute("class", "robot-indicator");
  g.style.display = "none";
  const glow = document.createElementNS("http://www.w3.org/2000/svg", "circle");
  glow.setAttribute("cx", "0");
  glow.setAttribute("cy", "0");
  glow.setAttribute("r", "18");
  glow.setAttribute("class", "robot-glow");
  const arrow = document.createElementNS(
    "http://www.w3.org/2000/svg",
    "polygon",
  );
  arrow.setAttribute("points", "0,-18 -10,10 10,10");
  arrow.setAttribute("class", "robot-arrow");
  const dot = document.createElementNS("http://www.w3.org/2000/svg", "circle");
  dot.setAttribute("cx", "0");
  dot.setAttribute("cy", "0");
  dot.setAttribute("r", "4");
  dot.setAttribute("class", "robot-center");
  g.appendChild(glow);
  g.appendChild(arrow);
  g.appendChild(dot);
  layer.appendChild(g);
  g.addEventListener("click", (e) => {
    e.stopPropagation();
    if (anim.active) return;
    if (currentMapTool === "edge" || currentMapTool === "weight") {
      handleNodeClick(robotCurrentNode);
      return;
    }
    robotInitialDir = (robotInitialDir + 1) % 4;
    robotCurrentDir = robotInitialDir;
    updateRobotIndicator(robotCurrentNode, robotCurrentDir);
    showToast(`Hướng đầu xe: ${DIR_LABELS[robotInitialDir]}`, "info");
  });
  robotIndicator = g;
}
function updateRobotIndicator(nodeId, dir) {
  if (!robotIndicator || nodeId < 0 || nodeId >= 20) return;
  const [x, y] = coords[nodeId];
  const rot = DIR_ROTATION[dir];
  robotIndicator.setAttribute(
    "transform",
    `translate(${x},${y}) rotate(${rot})`,
  );
  robotIndicator.style.display = "";
  robotCurrentNode = nodeId;
  robotCurrentDir = dir;
}
function hideRobotIndicator() {
  if (robotIndicator) robotIndicator.style.display = "none";
  robotCurrentNode = -1;
}
function getDirBetweenNodes(a, b) {
  const dx = Math.sign(coords[b][0] - coords[a][0]);
  const dy = Math.sign(coords[b][1] - coords[a][1]);
  if (dy < 0) return 0;
  if (dx > 0) return 1;
  if (dy > 0) return 2;
  if (dx < 0) return 3;
  return 2;
}

function heuristic(a, b) {
  return (
    Math.abs(coords[a][0] - coords[b][0]) +
    Math.abs(coords[a][1] - coords[b][1])
  );
}
function getEdgeWeight(a, b) {
  return edgeWeights[Math.min(a, b) + "-" + Math.max(a, b)] || 1;
}
function setEdgeWeight(a, b, w) {
  edgeWeights[Math.min(a, b) + "-" + Math.max(a, b)] = w;
}
function getAlgoName(a) {
  return (
    { bfs: "BFS", dfs: "DFS", ucs: "UCS", astar: "A*", greedy: "Greedy" }[a] ||
    a
  );
}

function updateClock() {
  const ts = new Date().toLocaleTimeString("en-US", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: true,
  });
  const el = document.getElementById("real-time-clock");
  if (el) el.textContent = ts;
  const ft = document.getElementById("footer-update-time");
  if (ft) ft.textContent = ts + " (GMT+7)";
}
setInterval(updateClock, 1000);
updateClock();

let toastContainer;
function ensureToastContainer() {
  if (!toastContainer) {
    toastContainer = document.createElement("div");
    toastContainer.className = "toast-container";
    document.body.appendChild(toastContainer);
  }
}
function showToast(msg, type = "info") {
  ensureToastContainer();
  const t = document.createElement("div");
  t.className = "toast " + type;
  t.textContent = msg;
  toastContainer.appendChild(t);
  setTimeout(() => {
    t.style.opacity = "0";
    t.style.transform = "translateX(20px)";
    setTimeout(() => t.remove(), 300);
  }, 3000);
}

// ==================== WebSocket ====================
let ws = null,
  wsReconnectTimer = null,
  pingTimer = null;
function connectWebSocket() {
  const host = window.location.hostname || "192.168.4.1";
  const port = window.location.port || "80";
  try {
    ws = new WebSocket(`ws://${host}:${port}/ws`);
  } catch (e) {
    scheduleReconnect();
    return;
  }
  ws.onopen = () => {
    setWsStatus(true);
    showToast("Kết nối ESP32 thành công", "success");
    if (wsReconnectTimer) {
      clearTimeout(wsReconnectTimer);
      wsReconnectTimer = null;
    }
    pingTimer = setInterval(() => {
      try {
        ws.send(JSON.stringify({ type: "PING" }));
      } catch (e) {}
    }, 5000);
  };
  ws.onmessage = (evt) => {
    try {
      handleWsMessage(JSON.parse(evt.data));
    } catch (e) {}
  };
  ws.onclose = () => {
    setWsStatus(false);
    if (pingTimer) {
      clearInterval(pingTimer);
      pingTimer = null;
    }
    scheduleReconnect();
  };
  ws.onerror = () => setWsStatus(false);
}
function scheduleReconnect() {
  if (wsReconnectTimer) return;
  wsReconnectTimer = setTimeout(() => {
    wsReconnectTimer = null;
    connectWebSocket();
  }, 3000);
}
function setWsStatus(c) {
  document.getElementById("wsStatusText").textContent = c
    ? "Trực tuyến"
    : "Mất kết nối";
  document.getElementById("wsStatusDot").className =
    "w-2 h-2 rounded-full " +
    (c ? "bg-green-400 animate-pulse" : "bg-yellow-400");
  document.getElementById("connBadge").textContent = c
    ? "Kết nối ổn định"
    : "Đang kết nối...";
  document.getElementById("connBadge").className =
    "px-3 py-1 text-[10px] font-bold rounded-full uppercase tracking-widest border " +
    (c
      ? "bg-green-100 text-green-700 border-green-200"
      : "bg-yellow-100 text-yellow-700 border-yellow-200");
}
function wsSend(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

function handleWsMessage(data) {
  if (data.type === "TELEMETRY") updateTelemetry(data);
  else if (data.type === "ROUTE_ACK")
    showToast(`Route nạp: ${data.commands} lệnh`, "success");
  else if (data.type === "OBSTACLE_DETECTED") {
    showToast(
      `⚠️ Vật cản tại (${data.position.row},${data.position.col})!`,
      "error",
    );
    updateRobotState("OBSTACLE");
    handleDynamicReroute(data);
  } else if (data.type === "COMPLETED") {
    showToast(
      `Hoàn thành! ${data.intersections} giao lộ, ${data.distance_cm}cm`,
      "success",
    );
    updateRobotState("DONE");
    handleDeliveryCompleted();
  }
}

function handleDynamicReroute(data) {
  const obsNode = data.position.row * 5 + data.position.col;
  if (obsNode >= 0 && obsNode < 20) {
    obstacles.add(obsNode);
  }
  const robotNode = data.robot_position.row * 5 + data.robot_position.col;
  const tarArr = Array.from(targets);
  if (tarArr.length > 0) {
    const algo = document.getElementById("algoSel").value;
    const result = findPathAnimated(robotNode, tarArr[0], algo);
    if (result) {
      const cmds = pathToCommands(result.path, robotCurrentDir);
      wsSend({
        type: "ROUTE",
        commands: cmds,
        total_steps: cmds.length,
        rerouted: true,
      });
      animRunSingle(result, getAlgoName(algo), "🔄 Tái định tuyến");
      showToast("🔄 Đang tính lại đường đi mới...", "info");
    } else {
      showToast("❌ Không tìm được đường đi mới!", "error");
    }
  }
  updateNodeVisuals();
  calculateOverallPath();
}

function handleDeliveryCompleted() {
  if (!deliveryInProgress) return;
  deliveryInProgress = false;

  if (robotCurrentNode >= 0) {
    document.getElementById("startNode").value = robotCurrentNode;
  }

  const actualTime = ((Date.now() - deliveryStartTime) / 1000).toFixed(1);
  const banner = document.getElementById("deliveryStatusBanner");
  const icon = document.getElementById("deliveryStatusIcon");
  const text = document.getElementById("deliveryStatusText");
  banner.classList.remove("hidden");
  if (deliveryExpectedTime && parseFloat(actualTime) > deliveryExpectedTime) {
    const lateBy = (parseFloat(actualTime) - deliveryExpectedTime).toFixed(1);
    banner.className =
      "flex items-center gap-2 rounded-xl p-3 bg-red-50 border border-red-200";
    icon.textContent = "warning";
    icon.className = "material-symbols-outlined text-lg text-red-600";
    text.textContent = `✅ Đã giao xong — ⚠️ TRỄ ${lateBy}s (${actualTime}s / ${deliveryExpectedTime}s)`;
    text.className = "text-sm font-bold text-red-700";
  } else {
    banner.className =
      "flex items-center gap-2 rounded-xl p-3 bg-green-50 border border-green-200";
    icon.textContent = "check_circle";
    icon.className = "material-symbols-outlined text-lg text-green-600";
    text.textContent = `✅ Đã giao xong — Thời gian: ${actualTime}s`;
    text.className = "text-sm font-bold text-green-700";
  }
}

function updateTelemetry(d) {
  const avg = (Math.abs(d.speedL || 0) + Math.abs(d.speedR || 0)) / 2;
  document.getElementById("teleSpeedVal").textContent = avg.toFixed(2);
  document.getElementById("speedBar").style.width =
    Math.min((avg / 1.5) * 100, 100) + "%";
  document.getElementById("teleDistVal").textContent = (
    d.distance || 0
  ).toFixed(1);
  document.getElementById("teleStateVal").textContent = d.state || "IDLE";
  document.getElementById("teleStepVal").textContent =
    `${d.step || 0} / ${d.total || 0}`;
  const obs = d.obstacle || 999;
  document.getElementById("sonarStatus").textContent =
    obs > 200 ? "-- cm" : obs.toFixed(1) + " cm";
  document.getElementById("sonarStatus").className =
    "text-xs font-bold " + (obs < 20 ? "text-error" : "text-green-600");
  document.getElementById("autoSpeedVal").textContent = avg.toFixed(1);
  document.getElementById("autoObstacleVal").textContent =
    obs > 200 ? "--" : obs.toFixed(0);
  if (d.sensors) {
    for (let i = 0; i < 5; i++) {
      const el = document.getElementById("sm" + i);
      if (el)
        el.className =
          "w-3 h-3 rounded-full " +
          (d.sensors[i]
            ? "bg-primary shadow-[0_0_6px_rgba(0,90,180,0.5)]"
            : "bg-surface-container");
    }
  }
  if (d.state) updateRobotState(d.state);
  if (d.robotNode !== undefined && d.robotNode >= 0) {
    const jsDir = CPP_TO_JS_DIR[d.robotDir || 0];
    updateRobotIndicator(d.robotNode, jsDir);
  }
}
function updateRobotState(s) {
  const map = {
    IDLE: "IDLE",
    FOLLOWING_LINE: "DÒ LINE",
    AT_INTERSECTION: "GIAO LỘ",
    OBSTACLE: "VẬT CẢN",
    REROUTING: "TÁI ĐỊNH TUYẾN",
    DONE: "HOÀN THÀNH",
  };
  document.getElementById("teleStateVal").textContent = map[s] || s;
}

async function switchTab(tab) {
  document
    .querySelectorAll(".panel-view")
    .forEach((el) => el.classList.remove("active"));
  document
    .querySelectorAll(".nav-link")
    .forEach((el) => el.classList.remove("active-nav"));
  document.getElementById("panel-" + tab).classList.add("active");
  document.getElementById("nav-" + tab).classList.add("active-nav");
  currentMode = tab;
  const titles = {
    manual: "Hệ thống: Vận hành Thủ công",
    auto: "Hệ thống: Chế độ Tự động",
    stats: "Mission Control Center — Thống kê",
  };
  document.getElementById("headerTitle").textContent = titles[tab];
  if (statsInterval) {
    clearInterval(statsInterval);
    statsInterval = null;
  }
  if (tab !== "stats") {
    try {
      await fetch("/setMode?m=" + (tab === "manual" ? "manual" : "line"));
    } catch (e) {}
  } else {
    fetchRealStats();
    statsInterval = setInterval(fetchRealStats, 3000);
  }
}
async function fetchRealStats() {
  try {
    const r = await fetch("/api/stats");
    if (r.ok) {
      const d = await r.json();
      document.getElementById("kpiDelivered").textContent = d.delivered;
      document.getElementById("kpiAvgTime").textContent = d.avgTime;
      document.getElementById("kpiSuccess").textContent = d.efficiency;
      document.getElementById("kpiDistance").textContent =
        d.totalDistance || "--";
    }
  } catch (e) {}
}
function switchAutoMode(mode) {
  const btnDeliv = document.getElementById("btn-sub-delivery");
  const btnLine = document.getElementById("btn-sub-line");
  if (mode === "delivery") {
    btnDeliv.className =
      "flex-1 py-2 text-xs font-bold rounded-lg bg-white shadow-sm text-primary";
    btnLine.className =
      "flex-1 py-2 text-xs font-bold rounded-lg text-gray-500 hover:text-primary";
    document.getElementById("delivery-section").classList.remove("hidden");
    document.getElementById("line-only-section").classList.add("hidden");
    document.getElementById("line-only-section").classList.remove("flex");
  } else {
    btnLine.className =
      "flex-1 py-2 text-xs font-bold rounded-lg bg-white shadow-sm text-primary";
    btnDeliv.className =
      "flex-1 py-2 text-xs font-bold rounded-lg text-gray-500 hover:text-primary";
    document.getElementById("delivery-section").classList.add("hidden");
    document.getElementById("line-only-section").classList.remove("hidden");
    document.getElementById("line-only-section").classList.add("flex");
  }
}
async function startLineOnly() {
  try {
    await fetch("/setMode?m=line_only");
    showToast("Xe bắt đầu dò line cơ bản!", "success");
  } catch (e) {}
}

function renderEdges() {
  const layer = document.querySelector(".track-lines");
  layer.innerHTML = "";
  const drawn = new Set();
  for (const [nid, neighbors] of Object.entries(graph)) {
    for (const nb of neighbors) {
      const a = parseInt(nid),
        b = nb;
      const key = Math.min(a, b) + "-" + Math.max(a, b);
      if (drawn.has(key)) continue;
      drawn.add(key);
      const [x1, y1] = coords[a],
        [x2, y2] = coords[b];
      const line = document.createElementNS(
        "http://www.w3.org/2000/svg",
        "line",
      );
      line.setAttribute("x1", x1);
      line.setAttribute("y1", y1);
      line.setAttribute("x2", x2);
      line.setAttribute("y2", y2);
      layer.appendChild(line);
      const w = getEdgeWeight(a, b);
      if (w !== 1) {
        const txt = document.createElementNS(
          "http://www.w3.org/2000/svg",
          "text",
        );
        txt.setAttribute("x", (x1 + x2) / 2);
        txt.setAttribute("y", (y1 + y2) / 2 - 8);
        txt.setAttribute("class", "edge-weight-label");
        txt.textContent = w;
        layer.appendChild(txt);
      }
    }
  }
}

function toggleEdge(a, b) {
  if (a === b) return;
  const has = graph[a].includes(b);
  if (has) {
    graph[a] = graph[a].filter((n) => n !== b);
    graph[b] = graph[b].filter((n) => n !== a);
    delete edgeWeights[Math.min(a, b) + "-" + Math.max(a, b)];
  } else {
    graph[a].push(b);
    graph[b].push(a);
  }
  renderEdges();
  updateNodeVisuals();
  calculateOverallPath();
  return !has;
}

function initNodes() {
  const startNodeEl = document.getElementById("startNode");
  for (let i = 0; i < 20; i++) {
    let o = document.createElement("option");
    o.value = i;
    o.innerText = "Node " + i;
    if (i === 15) o.selected = true;
    startNodeEl.appendChild(o);
  }
  const layer = document.getElementById("nodeLayer");
  for (let i = 0; i < 20; i++) {
    const [x, y] = coords[i],
      g = document.createElementNS("http://www.w3.org/2000/svg", "g");
    g.setAttribute("class", "node-group");
    g.setAttribute("id", "svg-node-" + i);
    g.onclick = () => handleNodeClick(i);
    const c = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    c.setAttribute("cx", x);
    c.setAttribute("cy", y);
    c.setAttribute("r", "12");
    c.setAttribute("class", "node-circle");
    const t = document.createElementNS("http://www.w3.org/2000/svg", "text");
    t.setAttribute("x", x);
    t.setAttribute("y", y);
    t.setAttribute("class", "node-text");
    t.textContent = i;
    g.appendChild(c);
    g.appendChild(t);
    layer.appendChild(g);
  }
}

function setMapTool(tool) {
  currentMapTool = tool;
  edgeSelFirst = null;
  ["Start", "Target", "Obstacle", "Edge", "Weight"].forEach((t) => {
    document.getElementById("tool" + t).className =
      tool === t.toLowerCase() ? "map-tool-btn active" : "map-tool-btn";
  });
  updateEdgeSelUI();
}
function updateEdgeSelUI() {
  for (let i = 0; i < 20; i++)
    document.getElementById("svg-node-" + i).classList.remove("node-edge-sel");
  if (edgeSelFirst !== null)
    document
      .getElementById("svg-node-" + edgeSelFirst)
      .classList.add("node-edge-sel");
}

function handleNodeClick(nid) {
  if (anim.active) {
    animReset();
  }
  const sn = parseInt(document.getElementById("startNode").value);
  updateValidationUI(false);

  if (currentMapTool === "start") {
    targets.delete(nid);
    obstacles.delete(nid);
    document.getElementById("startNode").value = nid;
    updateRobotIndicator(nid, robotCurrentDir);
    updateNodeVisuals();
    calculateOverallPath();
    return;
  }

  if (currentMapTool === "edge") {
    if (edgeSelFirst === null) {
      edgeSelFirst = nid;
      updateEdgeSelUI();
      showToast(`Đã chọn Node ${nid} — Click node kề để nối dây`, "info");
    } else {
      if (edgeSelFirst !== nid) {
        const added = toggleEdge(edgeSelFirst, nid);
        showToast(
          `Edge ${edgeSelFirst}↔${nid} ${added ? "đã nối" : "đã xóa"}`,
          "success",
        );
      }
      edgeSelFirst = null;
      updateEdgeSelUI();
    }
    return;
  }

  if (currentMapTool === "weight") {
    if (edgeSelFirst === null) {
      edgeSelFirst = nid;
      updateEdgeSelUI();
      showToast(`Đã chọn Node ${nid} — Click node kề để đổi trọng số`, "info");
    } else {
      if (edgeSelFirst !== nid && graph[edgeSelFirst].includes(nid)) {
        const cur = getEdgeWeight(edgeSelFirst, nid);
        const nw = prompt(
          `Trọng số edge ${edgeSelFirst}↔${nid} (hiện: ${cur}):`,
          cur,
        );
        if (nw !== null && !isNaN(parseInt(nw)) && parseInt(nw) > 0) {
          setEdgeWeight(edgeSelFirst, nid, parseInt(nw));
          renderEdges();
          updateNodeVisuals();
          calculateOverallPath();
          showToast(`Trọng số ${edgeSelFirst}↔${nid} = ${nw}`, "success");
        }
      } else if (edgeSelFirst !== nid) {
        showToast("Không có kết nối giữa 2 node này!", "error");
      }
      edgeSelFirst = null;
      updateEdgeSelUI();
    }
    return;
  }

  if (nid === sn) return;

  if (currentMapTool === "target") {
    obstacles.delete(nid);
    const mode = document.getElementById("orderModeSel").value;
    if (mode === "single") {
      targets.clear();
      targets.add(nid);
    } else {
      if (targets.has(nid)) targets.delete(nid);
      else {
        if (targets.size >= 6) {
          showToast("Tối đa 6 đơn!", "error");
          return;
        }
        targets.add(nid);
      }
    }
  } else if (currentMapTool === "obstacle") {
    targets.delete(nid);
    if (obstacles.has(nid)) obstacles.delete(nid);
    else obstacles.add(nid);
  }
  updateNodeVisuals();
  calculateOverallPath();
}

function updateNodeVisuals() {
  const sn = parseInt(document.getElementById("startNode").value);
  for (let i = 0; i < 20; i++) {
    let cls = "node-group";
    if (i === sn) cls += " node-start";
    else if (obstacles.has(i)) cls += " node-obstacle";
    else if (targets.has(i)) cls += " target-1"; // Mặc định cho mọi target được đánh dấu
    document.getElementById("svg-node-" + i).setAttribute("class", cls);
  }
  document.getElementById("pathLayer").innerHTML = "";
  if (targets.size === 0) {
    document.getElementById("pathResult").innerText =
      "Chọn điểm xuất phát và điểm đến trên bản đồ";
  }
}

// ==================== Algorithms ====================
function shuffle(arr) {
  let a = [...arr];
  for (let i = a.length - 1; i > 0; i--) {
    let j = Math.floor(Math.random() * (i + 1));
    [a[i], a[j]] = [a[j], a[i]];
  }
  return a;
}

function findPathAnimated(start, end, algo) {
  if (obstacles.has(start) || obstacles.has(end)) return null;
  algo = algo || document.getElementById("algoSel").value;
  const steps = [];

  if (algo === "bfs") {
    let queue = [[start]],
      explored = new Set(),
      frontier = new Set([start]);
    while (queue.length) {
      let path = queue.shift(),
        node = path[path.length - 1];
      frontier.delete(node);
      explored.add(node);
      steps.push({
        current: node,
        explored: new Set(explored),
        frontier: new Set(frontier),
        from: path.length > 1 ? path[path.length - 2] : -1,
      });
      if (node === end)
        return {
          path,
          steps,
          visitedCount: explored.size,
          cost: path.length - 1,
        };
      for (let nx of shuffle(graph[node] || [])) {
        if (!explored.has(nx) && !frontier.has(nx) && !obstacles.has(nx)) {
          frontier.add(nx);
          queue.push([...path, nx]);
        }
      }
    }
  } else if (algo === "dfs") {
    let stack = [[start]],
      explored = new Set(),
      frontier = new Set([start]);
    while (stack.length) {
      let path = stack.pop(),
        node = path[path.length - 1];
      frontier.delete(node);
      if (explored.has(node)) continue;
      explored.add(node);
      steps.push({
        current: node,
        explored: new Set(explored),
        frontier: new Set(frontier),
        from: path.length > 1 ? path[path.length - 2] : -1,
      });
      if (node === end)
        return {
          path,
          steps,
          visitedCount: explored.size,
          cost: path.length - 1,
        };
      for (let nx of shuffle(graph[node] || [])) {
        if (!explored.has(nx) && !obstacles.has(nx)) {
          frontier.add(nx);
          stack.push([...path, nx]);
        }
      }
    }
  } else if (algo === "ucs") {
    let pq = [{ path: [start], cost: 0 }],
      best = new Map(),
      frontier = new Set([start]);
    best.set(start, 0);
    while (pq.length) {
      pq.sort((a, b) => a.cost - b.cost);
      let cur = pq.shift(),
        node = cur.path[cur.path.length - 1];
      frontier.delete(node);
      steps.push({
        current: node,
        explored: new Set(best.keys()),
        frontier: new Set(frontier),
        from: cur.path.length > 1 ? cur.path[cur.path.length - 2] : -1,
      });
      if (node === end)
        return {
          path: cur.path,
          steps,
          visitedCount: best.size,
          cost: cur.cost,
        };
      for (let nx of shuffle(graph[node] || [])) {
        if (obstacles.has(nx)) continue;
        let nc = cur.cost + getEdgeWeight(node, nx);
        if (!best.has(nx) || nc < best.get(nx)) {
          best.set(nx, nc);
          frontier.add(nx);
          pq.push({ path: [...cur.path, nx], cost: nc });
        }
      }
    }
  } else if (algo === "greedy") {
    let pq = [{ path: [start], h: heuristic(start, end) }],
      explored = new Set(),
      frontier = new Set([start]);
    while (pq.length) {
      pq.sort((a, b) => a.h - b.h);
      let cur = pq.shift(),
        node = cur.path[cur.path.length - 1];
      frontier.delete(node);
      if (explored.has(node)) continue;
      explored.add(node);
      steps.push({
        current: node,
        explored: new Set(explored),
        frontier: new Set(frontier),
        from: cur.path.length > 1 ? cur.path[cur.path.length - 2] : -1,
      });
      if (node === end)
        return {
          path: cur.path,
          steps,
          visitedCount: explored.size,
          cost: cur.path.length - 1,
        };
      for (let nx of shuffle(graph[node] || [])) {
        if (!explored.has(nx) && !obstacles.has(nx)) {
          frontier.add(nx);
          pq.push({ path: [...cur.path, nx], h: heuristic(nx, end) });
        }
      }
    }
  } else {
    // astar
    let pq = [{ path: [start], g: 0, f: heuristic(start, end) }],
      gS = new Map(),
      explored = new Set(),
      frontier = new Set([start]);
    gS.set(start, 0);
    while (pq.length) {
      pq.sort((a, b) => a.f - b.f);
      let cur = pq.shift(),
        node = cur.path[cur.path.length - 1];
      frontier.delete(node);
      if (explored.has(node)) continue;
      explored.add(node);
      steps.push({
        current: node,
        explored: new Set(explored),
        frontier: new Set(frontier),
        from: cur.path.length > 1 ? cur.path[cur.path.length - 2] : -1,
      });
      if (node === end)
        return {
          path: cur.path,
          steps,
          visitedCount: explored.size,
          cost: cur.g,
        };
      for (let nx of shuffle(graph[node] || [])) {
        if (obstacles.has(nx) || explored.has(nx)) continue;
        let ng = cur.g + getEdgeWeight(node, nx);
        if (!gS.has(nx) || ng < gS.get(nx)) {
          gS.set(nx, ng);
          frontier.add(nx);
          pq.push({
            path: [...cur.path, nx],
            g: ng,
            f: ng + heuristic(nx, end),
          });
        }
      }
    }
  }
  return null;
}
function findPath(start, end, algo) {
  const r = findPathAnimated(start, end, algo);
  if (!r) return null;
  return { path: r.path, visitedCount: r.visitedCount, cost: r.cost };
}

function findPathToNearest(start, targetSet, algo) {
  if (obstacles.has(start)) return null;
  algo = algo || document.getElementById("algoSel").value;
  let bestR = null;
  let minLen = Infinity;
  let bestT = null;
  for (let t of targetSet) {
    let r = findPathAnimated(start, t, algo);
    if (r && r.path.length < minLen) {
      minLen = r.path.length;
      bestR = r;
      bestT = t;
    }
  }
  if (bestR) bestR.foundTarget = bestT;
  return bestR;
}

function buildSegments(sn, tarArr, algo) {
  let ordered = [],
    currentS = sn,
    remaining = new Set(tarArr);
  const dm = document.getElementById("deliveryModeSel").value;
  if (dm === "express" && tarArr.length > 1) {
    ordered.push(tarArr[tarArr.length - 1]);
    remaining.delete(tarArr[tarArr.length - 1]);
  }

  let segs = [],
    fullPath = [sn];
  currentS = sn;
  let totalSteps = 0,
    totalExplored = 0,
    totalCost = 0;

  while (remaining.size) {
    const r = findPathToNearest(currentS, remaining, algo);
    if (!r) break;
    ordered.push(r.foundTarget);
    remaining.delete(r.foundTarget);
    segs.push({
      steps: r.steps,
      path: r.path,
      from: currentS,
      to: r.foundTarget,
      visitedCount: r.visitedCount,
      cost: r.cost,
    });
    totalSteps += r.path.length - 1;
    totalExplored += r.visitedCount;
    totalCost += r.cost;
    fullPath = fullPath.concat(r.path.slice(1));
    currentS = r.foundTarget;
  }
  return {
    segs,
    fullPath,
    ordered,
    totalSteps,
    totalExplored,
    totalCost,
    remaining,
  };
}

function flattenSegments(segs) {
  let all = [];
  for (let si = 0; si < segs.length; si++) {
    const seg = segs[si];
    for (let j = 0; j < seg.steps.length; j++) {
      all.push({
        ...seg.steps[j],
        segIdx: si,
        isLast: j === seg.steps.length - 1,
        segPath: seg.path,
      });
    }
  }
  return all;
}

const SEG_COLORS = [
  "#16a34a",
  "#0873df",
  "#964400",
  "#ba1a1a",
  "#465f89",
  "#bd5700",
];

function animStart() {
  const sn = parseInt(document.getElementById("startNode").value);
  const tarArr = Array.from(targets);
  if (!tarArr.length) {
    showToast("Chọn điểm đích trước!", "error");
    return;
  }

  let hasEdges = false;
  for (let i = 0; i < 20; i++) if (graph[i].length > 0) hasEdges = true;
  if (!hasEdges) {
    showToast(
      "Vui lòng dùng công cụ Edge nối các điểm lại với nhau trước!",
      "error",
    );
    return;
  }

  const algo = document.getElementById("algoSel").value;
  const {
    segs,
    fullPath,
    ordered,
    totalSteps,
    totalExplored,
    totalCost,
    remaining,
  } = buildSegments(sn, tarArr, algo);

  if (!segs.length) {
    showToast("Không thể tìm đường (Bị cô lập hoặc chưa nối Edge)", "error");
    updateValidationUI(false);
    return;
  }

  animStop();
  animClearVis();
  anim.steps = flattenSegments(segs);
  anim.segPaths = segs.map((s) => s.path);
  anim.fullPath = fullPath;
  anim.path = fullPath;
  anim.currentStep = 0;
  anim.active = true;
  anim.algoName = getAlgoName(algo);
  anim.resultSteps = totalSteps;
  anim.resultExplored = totalExplored;
  anim.resultCost = totalCost;
  anim.segCount = segs.length;
  anim.prevSegIdx = -1;
  anim.skippedTargets = remaining;

  updateNodeVisuals();
  document.getElementById("animationControls").classList.remove("hidden");
  document.getElementById("pathLayer").innerHTML = "";
  updateAnimUI();
  anim.playing = true;
  updateAnimUI();
  animTick();
}

function animRunSingle(result, algoName, prefix) {
  animStop();
  animClearVis();
  anim.steps = result.steps.map((s, j) => ({
    ...s,
    segIdx: 0,
    isLast: j === result.steps.length - 1,
    segPath: result.path,
  }));
  anim.segPaths = [result.path];
  anim.fullPath = result.path;
  anim.path = result.path;
  anim.currentStep = 0;
  anim.active = true;
  anim.algoName = algoName;
  anim.resultSteps = result.path.length - 1;
  anim.resultExplored = result.visitedCount;
  anim.resultCost = result.cost;
  anim.segCount = 1;
  anim.prevSegIdx = -1;
  anim.skippedTargets = new Set();

  updateNodeVisuals();
  document.getElementById("animationControls").classList.remove("hidden");
  document.getElementById("pathLayer").innerHTML = "";
  updateAnimUI();
  anim.playing = true;
  updateAnimUI();
  animTick();
}

function animPlay() {
  if (!anim.active) return;
  if (anim.currentStep >= anim.steps.length) {
    animShowFinalPath();
    return;
  }
  anim.playing = true;
  updateAnimUI();
  animTick();
}
function animTick() {
  if (!anim.playing || anim.currentStep >= anim.steps.length) {
    if (anim.currentStep >= anim.steps.length) animShowFinalPath();
    anim.playing = false;
    updateAnimUI();
    return;
  }
  animRenderStep(anim.steps[anim.currentStep]);
  anim.currentStep++;
  updateAnimUI();
  anim.timer = setTimeout(animTick, anim.speed);
}
function animPause() {
  anim.playing = false;
  if (anim.timer) {
    clearTimeout(anim.timer);
    anim.timer = null;
  }
  updateAnimUI();
}
function animStepFwd() {
  if (!anim.active) return;
  if (anim.currentStep >= anim.steps.length) {
    animShowFinalPath();
    return;
  }
  animPause();
  animRenderStep(anim.steps[anim.currentStep]);
  anim.currentStep++;
  updateAnimUI();
}
function animReset() {
  animStop();
  animClearVis();
  anim.currentStep = 0;
  anim.active = false;
  document.getElementById("animationControls").classList.add("hidden");
  document.getElementById("pathLayer").innerHTML = "";
  updateNodeVisuals();
  calculateOverallPath();
}
function animStop() {
  anim.playing = false;
  if (anim.timer) {
    clearTimeout(anim.timer);
    anim.timer = null;
  }
}

function animClearVis() {
  for (let i = 0; i < 20; i++)
    document
      .getElementById("svg-node-" + i)
      .classList.remove(
        "node-explored",
        "node-frontier",
        "node-current",
        "node-final-path",
        "node-skipped",
      );
  const fl = document.getElementById("flowLayer");
  if (fl) fl.innerHTML = "";
}

function drawSegPath(path, idx) {
  const col = SEG_COLORS[idx % SEG_COLORS.length];
  const pts = path.map((n) => coords[n].join(",")).join(" ");
  const poly = document.createElementNS(
    "http://www.w3.org/2000/svg",
    "polyline",
  );
  poly.setAttribute("class", "segment-path");
  poly.setAttribute("points", pts);
  poly.setAttribute("stroke", col);
  poly.setAttribute("style", `filter:drop-shadow(0 0 5px ${col}40)`);
  document.getElementById("pathLayer").appendChild(poly);
}

function getFlowLayer() {
  let fl = document.getElementById("flowLayer");
  if (!fl) {
    fl = document.createElementNS("http://www.w3.org/2000/svg", "g");
    fl.id = "flowLayer";
    const nodeLayer = document.getElementById("nodeLayer");
    nodeLayer.parentNode.insertBefore(fl, nodeLayer);
  }
  return fl;
}
function drawFlowEdge(layer, fromN, toN, cls) {
  const [x1, y1] = coords[fromN],
    [x2, y2] = coords[toN];
  const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
  line.setAttribute("x1", x1);
  line.setAttribute("y1", y1);
  line.setAttribute("x2", x2);
  line.setAttribute("y2", y2);
  line.setAttribute("class", "flow-edge " + cls);
  const len = Math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2);
  line.style.strokeDasharray = len;
  line.style.strokeDashoffset = len;
  layer.appendChild(line);
  line.animate(
    [{ strokeDashoffset: len + "px" }, { strokeDashoffset: "0px" }],
    { duration: 220, fill: "forwards", easing: "ease-out" },
  );
}

function animRenderStep(step) {
  const sn = parseInt(document.getElementById("startNode").value);
  const flowLayer = getFlowLayer();
  if (step.segIdx !== anim.prevSegIdx) {
    if (anim.prevSegIdx >= 0) {
      drawSegPath(anim.segPaths[anim.prevSegIdx], anim.prevSegIdx);
      for (let n of anim.segPaths[anim.prevSegIdx]) {
        if (n === sn || targets.has(n) || obstacles.has(n)) continue;
        const el = document.getElementById("svg-node-" + n);
        el.classList.remove("node-explored", "node-frontier", "node-current");
        el.classList.add("node-final-path");
      }
    }
    for (let i = 0; i < 20; i++)
      document
        .getElementById("svg-node-" + i)
        .classList.remove("node-explored", "node-frontier", "node-current");
    flowLayer.innerHTML = "";
    anim.prevSegIdx = step.segIdx;
  }

  flowLayer.querySelectorAll(".edge-current").forEach((el) => {
    el.classList.remove("edge-current");
    el.classList.add("edge-explored");
    el.style.strokeDashoffset = "0px";
  });
  if (step.from !== undefined && step.from >= 0)
    drawFlowEdge(flowLayer, step.from, step.current, "edge-current");

  for (let n of step.explored) {
    if (n === sn || targets.has(n) || obstacles.has(n)) continue;
    const el = document.getElementById("svg-node-" + n);
    if (!el.classList.contains("node-final-path"))
      el.classList.add("node-explored");
  }
  for (let n of step.frontier) {
    if (n === sn || targets.has(n) || obstacles.has(n)) continue;
    const el = document.getElementById("svg-node-" + n);
    if (!el.classList.contains("node-final-path"))
      el.classList.add("node-frontier");
  }
  if (
    step.current !== sn &&
    !targets.has(step.current) &&
    !obstacles.has(step.current)
  ) {
    const el = document.getElementById("svg-node-" + step.current);
    el.classList.remove("node-explored", "node-frontier");
    el.classList.add("node-current");
  }
  document.getElementById("svg-node-" + sn).classList.add("node-start");
  let ti = 0;
  for (let t of targets) {
    document
      .getElementById("svg-node-" + t)
      .setAttribute("class", "node-group target-" + (Math.min(ti, 5) + 1));
    ti++;
  }
  for (let o of obstacles)
    document.getElementById("svg-node-" + o).classList.add("node-obstacle");
}

function animShowFinalPath() {
  anim.playing = false;
  updateAnimUI();
  if (!anim.fullPath || !anim.fullPath.length) return;
  const sn = parseInt(document.getElementById("startNode").value);
  const fl = document.getElementById("flowLayer");
  if (fl) fl.innerHTML = "";
  if (anim.prevSegIdx >= 0)
    drawSegPath(anim.segPaths[anim.prevSegIdx], anim.prevSegIdx);

  for (let n of anim.fullPath) {
    if (n === sn || targets.has(n)) continue;
    const el = document.getElementById("svg-node-" + n);
    el.classList.remove("node-explored", "node-frontier", "node-current");
    el.classList.add("node-final-path");
  }

  document.getElementById("autoStepsVal").innerHTML =
    anim.resultSteps +
    ' <span class="text-xs font-medium text-on-surface-variant italic">bước</span>';
  document.getElementById("autoVisitedVal").textContent = anim.resultExplored;
  const resEl = document.getElementById("pathResult");

  let msg = `✅ Đã chốt lộ trình! (${anim.resultSteps} bước, ${anim.resultExplored} node)`;
  if (anim.skippedTargets && anim.skippedTargets.size > 0) {
    let skippedArr = Array.from(anim.skippedTargets);
    msg += `\n⚠️ BỎ QUA ĐƠN: Node ${skippedArr.join(", ")} (Vướng vật cản)`;
    resEl.className = "p-4 text-sm text-center warn-msg font-bold";
    // Tích hợp: Tự động gạch chéo đỏ ngay cả sau khi animation chạy xong
    skippedArr.forEach((t) =>
      document
        .getElementById("svg-node-" + t)
        .setAttribute("class", "node-group node-skipped"),
    );
    showToast(
      `✅ Đã chốt đường! Bỏ qua ${skippedArr.length} đơn bị kẹt`,
      "warning",
    );
  } else {
    resEl.className = "p-4 text-sm text-center has-path font-bold";
    showToast(`✅ Mô phỏng hoàn tất! Sẵn sàng nạp xuống xe.`, "success");
  }
  resEl.innerText = msg;
  finalCalculatedPath = anim.fullPath;

  updateValidationUI(true);
}

function updateAnimUI() {
  document.getElementById("animStepNum").textContent = anim.currentStep;
  document.getElementById("animTotalSteps").textContent = anim.steps.length;
  if (anim.currentStep > 0 && anim.currentStep <= anim.steps.length) {
    const s = anim.steps[anim.currentStep - 1];
    document.getElementById("animExploredCount").textContent = s.explored.size;
    document.getElementById("animFrontierCount").textContent = s.frontier.size;
  } else {
    document.getElementById("animExploredCount").textContent = "0";
    document.getElementById("animFrontierCount").textContent = "0";
  }
}

// ==================== TÍNH TOÁN NỀN VÀ HIỂN THỊ TÌNH TRẠNG KẸT ====================
function calculateOverallPath() {
  const sn = parseInt(document.getElementById("startNode").value);
  const deliveryMode = document.getElementById("deliveryModeSel").value;
  const algo = document.getElementById("algoSel").value;

  for (let i = 0; i < 20; i++) {
    let cls = "node-group";
    if (i === sn) cls += " node-start";
    else if (obstacles.has(i)) cls += " node-obstacle";
    else if (targets.has(i)) cls += " target-1"; // <--- LUÔN ĐỂ TARGET HIỆN LÊN ĐỂ BIẾT CHỖ CẦN GIAO
    document.getElementById("svg-node-" + i).setAttribute("class", cls);
  }

  const pathLayer = document.getElementById("pathLayer");
  pathLayer.innerHTML = "";
  const legendBox = document.getElementById("deliveryLegend");
  const orderInfoEl = document.getElementById("deliveryOrderInfo");
  const routeInfoEl = document.getElementById("deliveryRouteInfo");

  if (!targets.size) {
    finalCalculatedPath = [];
    document.getElementById("autoStepsVal").innerHTML =
      '0 <span class="text-xs font-medium text-on-surface-variant italic">bước</span>';
    document.getElementById("autoVisitedVal").textContent = "0";
    orderInfoEl.textContent = "Chưa có đơn hàng";
    routeInfoEl.textContent = "—";
    updateValidationUI(false);
    return;
  }

  const tarArr = Array.from(targets);
  const { segs, fullPath, ordered, totalSteps, totalExplored, remaining } =
    buildSegments(sn, tarArr, algo); // Lấy ra những Node bị bỏ lại

  if (ordered.length) {
    orderInfoEl.textContent = ordered
      .map(
        (n, i) =>
          `Đơn ${i + 1} → Node ${n}` +
          (deliveryMode === "express" && i === 0 ? " 🚀" : ""),
      )
      .join("  |  ");
    routeInfoEl.textContent = fullPath.join(" → ");
    document.getElementById("autoStepsVal").innerHTML =
      totalSteps +
      ' <span class="text-xs font-medium text-on-surface-variant italic">bước</span>';
    document.getElementById("autoVisitedVal").textContent = totalExplored;
    document.getElementById("autoOrderNum").textContent =
      "#01/" + (ordered.length < 10 ? "0" + ordered.length : ordered.length);

    let legHTML =
      '<div class="w-full text-center font-bold text-xs text-on-surface-variant mb-1">DỰ KIẾN GIAO HÀNG</div>';
    ordered.forEach((node, idx) => {
      let ci = Math.min(idx, 5),
        col = TARGET_COLORS[ci];
      // Nếu có đường tới, override màu của nó theo thứ tự ưu tiên giao hàng
      document
        .getElementById("svg-node-" + node)
        .setAttribute("class", "node-group target-" + (ci + 1));
      legHTML += `<span class="flex items-center gap-1 px-2 py-1 bg-surface-container rounded-lg"><span class="w-2 h-2 rounded-full" style="background:${col}"></span>Đơn ${idx + 1} (N${node})</span>`;
    });
    legendBox.innerHTML = legHTML;
    legendBox.classList.remove("hidden");
  } else {
    orderInfoEl.textContent = "Cần Mô phỏng Thuật toán trước!";
    routeInfoEl.textContent = "—";
    legendBox.classList.add("hidden");
  }

  // <--- NẾU CÓ NHỮNG NODE KẸT ĐƯỜNG, ĐÁNH DẤU CHÉO ĐỎ (node-skipped) LÊN NGAY LẬP TỨC
  if (remaining && remaining.size > 0) {
    Array.from(remaining).forEach((t) => {
      document
        .getElementById("svg-node-" + t)
        .setAttribute("class", "node-group node-skipped");
    });
  }
}

// ==================== Chuyển Lệnh ====================
function pathToCommands(path, initialDir) {
  let dir = parseInt(initialDir || 1);
  const cmds = [];
  const dd = [
    { dx: 0, dy: -1 },
    { dx: 1, dy: 0 },
    { dx: 0, dy: 1 },
    { dx: -1, dy: 0 },
  ];
  for (let i = 0; i < path.length - 1; i++) {
    const dx = Math.sign(coords[path[i + 1]][0] - coords[path[i]][0]);
    const dy = Math.sign(coords[path[i + 1]][1] - coords[path[i]][1]);
    let td = -1;
    for (let d = 0; d < 4; d++)
      if (dd[d].dx === dx && dd[d].dy === dy) {
        td = d;
        break;
      }
    if (td === -1) {
      cmds.push("F");
      continue;
    }
    const diff = (td - dir + 4) % 4;
    if (diff === 1) cmds.push("R");
    else if (diff === 3) cmds.push("L");
    else if (diff === 2) {
      cmds.push("R");
      cmds.push("R");
    } else cmds.push("F");
    dir = td;
  }
  return cmds;
}

// ==================== NÚT NẠP LỘ TRÌNH ====================
document.getElementById("deliverBtn").addEventListener("click", async () => {
  if (!isRouteValidated) {
    showToast("Vui lòng ấn MÔ PHỎNG THUẬT TOÁN trước khi nạp!", "error");
    return;
  }
  if (!finalCalculatedPath || finalCalculatedPath.length < 2) {
    showToast("Lộ trình không hợp lệ!", "error");
    return;
  }

  const cmds = pathToCommands(finalCalculatedPath, robotInitialDir);
  const algo = document.getElementById("algoSel").value;
  const btn1 = document.getElementById("deliverBtn");
  const btn2 = document.getElementById("deliverBtn2");

  btn1.innerHTML = "⏳ ĐANG TRUYỀN...";
  btn2.innerHTML = "⏳ ĐANG TRUYỀN...";
  btn1.classList.add("opacity-70");
  btn2.classList.add("opacity-70");

  deliveryStartTime = Date.now();
  deliveryExpectedTime = finalCalculatedPath.length * 2;
  deliveryInProgress = true;

  const banner = document.getElementById("deliveryStatusBanner");
  const icon = document.getElementById("deliveryStatusIcon");
  const text = document.getElementById("deliveryStatusText");
  banner.classList.remove("hidden");
  banner.className =
    "flex items-center gap-2 rounded-xl p-3 bg-amber-50 border border-amber-200";
  icon.textContent = "🚚";
  icon.className = "text-lg text-amber-600 animate-pulse";
  text.textContent = `🚚 Đang giao hàng... (Dự kiến: ~${deliveryExpectedTime}s)`;
  text.className = "text-sm font-bold text-amber-700";

  if (ws && ws.readyState === WebSocket.OPEN) {
    wsSend({
      type: "ROUTE",
      commands: cmds,
      total_steps: cmds.length,
      algorithm: algo,
      path: finalCalculatedPath,
      startNode: parseInt(document.getElementById("startNode").value),
      initialDir: JS_TO_CPP_DIR[robotInitialDir],
    });
  }
  setTimeout(() => {
    btn1.innerHTML = "✅ ĐÃ NẠP THÀNH CÔNG!";
    btn2.innerHTML = "✅ ĐÃ NẠP THÀNH CÔNG!";
    showToast("Lộ trình đã được gửi đến robot", "success");
    setTimeout(() => {
      updateValidationUI(true);
    }, 2000);
  }, 1000);
});

// ==================== LỆNH VỀ KHO TỰ ĐỘNG (ĐÃ TỐI ƯU) ====================
function returnHome() {
  const warehouseNode = 0;

  // 1. Lấy vị trí THỰC TẾ của xe hiện tại (ưu tiên Telemetry)
  let startN =
    robotCurrentNode >= 0
      ? robotCurrentNode
      : parseInt(document.getElementById("startNode").value);

  // Nếu đã ở kho rồi thì thôi
  if (startN === warehouseNode) {
    showToast("Xe hiện đang ở kho (Node 0) rồi!", "info");
    return;
  }

  // 2. Dọn dẹp bản đồ, chốt duy nhất 1 điểm đến là Node 0
  targets.clear();
  targets.add(warehouseNode);

  // Cập nhật lại UI ô "Xuất phát" cho khớp với thực tế
  document.getElementById("startNode").value = startN;
  updateNodeVisuals();

  // 3. Tính toán đường đi ngắn nhất về kho dựa trên thuật toán đang chọn
  const algo = document.getElementById("algoSel").value;
  const { fullPath } = buildSegments(startN, [warehouseNode], algo);

  if (fullPath && fullPath.length >= 2) {
    // Có đường về -> Chốt lộ trình và vẽ lên bản đồ
    finalCalculatedPath = fullPath;
    calculateOverallPath(); // Vẽ đường hiển thị lên UI cho người dùng thấy

    // Ép mở khóa nút Nạp và tự động click
    updateValidationUI(true);
    document.getElementById("deliverBtn").click();
    showToast("Đã tìm thấy đường về kho, đang tự động nạp...", "success");
  } else {
    // Không có đường về (Bị vật cản chặn kín)
    calculateOverallPath(); // Cập nhật UI để hiện gạch chéo đỏ báo kẹt
    showToast(
      "Cảnh báo: Xe bị cô lập bởi vật cản! Không thể tự về kho.",
      "error",
    );
  }
}

document
  .querySelectorAll("#startNode,#algoSel,#orderModeSel,#deliveryModeSel")
  .forEach((el) =>
    el.addEventListener("change", () => {
      if (
        el.id === "orderModeSel" &&
        el.value === "single" &&
        targets.size > 1
      ) {
        const f = Array.from(targets)[0];
        targets.clear();
        targets.add(f);
      }
      if (el.id === "startNode")
        updateRobotIndicator(parseInt(el.value), robotCurrentDir);
      updateValidationUI(false);
      obstacles.delete(parseInt(document.getElementById("startNode").value));
      updateNodeVisuals();
      calculateOverallPath();
    }),
  );
document.getElementById("animSpeedSlider").addEventListener("input", (e) => {
  const v = parseInt(e.target.value);
  const t = 1 - v / 100;
  anim.speed = Math.round(1500 * t * t + 30);
});

document.getElementById("robotSpeedSlider")?.addEventListener("input", (e) => {
  const v = (e.target.value / 100).toFixed(2);
  document.getElementById("robotSpeedVal").textContent = v;
  wsSend({ type: "SPEED", v_base: parseFloat(v) });
});

async function sendCommand(cmd) {
  try {
    await fetch(cmd);
    showToast("Lệnh: " + cmd, "info");
  } catch (e) {}
}
async function send(path) {
  try {
    await fetch(path);
  } catch (e) {}
}
let activeHold = { btn: null, pid: null };
function guard(fn) {
  return (e) => {
    if (currentMode !== "manual") {
      e.preventDefault();
      return;
    }
    return fn(e);
  };
}
document.querySelectorAll(".hold").forEach((btn) => {
  btn.addEventListener(
    "pointerdown",
    guard((e) => {
      e.preventDefault();
      activeHold = { btn, pid: e.pointerId };
      btn.classList.add("active-hold");
      btn.setPointerCapture(e.pointerId);
      send(btn.dataset.path);
    }),
    { passive: false },
  );
  const release = guard((e) => {
    e.preventDefault();
    if (activeHold.btn === btn && activeHold.pid === e.pointerId) {
      btn.classList.remove("active-hold");
      send("/stop");
      activeHold = { btn: null, pid: null };
    }
    try {
      btn.releasePointerCapture(e.pointerId);
    } catch (_) {}
  });
  btn.addEventListener("pointerup", release, { passive: false });
  btn.addEventListener("pointercancel", release, { passive: false });
});
document.getElementById("stopBtn").addEventListener(
  "pointerdown",
  guard((e) => {
    e.preventDefault();
    send("/stop");
  }),
  { passive: false },
);

updateValidationUI(false);
initNodes();
renderEdges();
createRobotIndicator();
updateRobotIndicator(
  parseInt(document.getElementById("startNode").value),
  robotInitialDir,
);
updateNodeVisuals();
calculateOverallPath();
setTimeout(() => connectWebSocket(), 1000);
