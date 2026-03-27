let currentMode = "manual";
let timeChartInstance = null;
let statsInterval = null;

// ================= UI TABS LOGIC =================
async function switchTab(tab) {
  document
    .querySelectorAll(".panel, .tab-btn")
    .forEach((el) => el.classList.remove("active"));
  document.getElementById("panel-" + tab).classList.add("active");
  document.getElementById("btn-tab-" + tab).classList.add("active");
  currentMode = tab;

  if (statsInterval) {
    clearInterval(statsInterval);
    statsInterval = null;
  }

  if (tab !== "stats" && tab !== "auto") {
    try {
      await fetch("/setMode?m=" + (tab === "manual" ? "manual" : "line"));
    } catch (e) {}
  } else if (tab === "stats") {
    fetchRealStats();
    statsInterval = setInterval(fetchRealStats, 3000);
  }
}

function switchAutoMode(mode) {
  document
    .querySelectorAll(".auto-tab-btn")
    .forEach((b) => b.classList.remove("active"));
  if (mode === "delivery") {
    document.getElementById("btn-sub-delivery").classList.add("active");
    document.getElementById("delivery-section").style.display = "block";
    document.getElementById("line-only-section").style.display = "none";
  } else {
    document.getElementById("btn-sub-line").classList.add("active");
    document.getElementById("delivery-section").style.display = "none";
    document.getElementById("line-only-section").style.display = "block";
  }
}

async function startLineOnly() {
  try {
    await fetch("/setMode?m=line_only");
    alert("Xe bắt đầu dò line cơ bản!");
  } catch (e) {
    console.error("Lỗi gửi lệnh", e);
  }
}

// ================= STATS API & CHART =================
async function fetchRealStats() {
  try {
    const response = await fetch("/api/stats");
    if (response.ok) {
      const data = await response.json();
      document.getElementById("kpiDelivered").innerText = data.delivered;
      document.getElementById("kpiAvgTime").innerText = data.avgTime + "s";
      document.getElementById("kpiEfficiency").innerText =
        data.efficiency + "%";
      updateChart(data.chart.labels, data.chart.expected, data.chart.actual);
    }
  } catch (error) {
    console.log("Đang chờ kết nối dữ liệu thống kê...");
  }
}

function updateChart(labels, expectedData, actualData) {
  const ctx = document.getElementById("timeComparisonChart").getContext("2d");
  if (timeChartInstance) {
    timeChartInstance.data.labels = labels;
    timeChartInstance.data.datasets[0].data = expectedData;
    timeChartInstance.data.datasets[1].data = actualData;
    timeChartInstance.update();
    return;
  }
  timeChartInstance = new Chart(ctx, {
    type: "bar",
    data: {
      labels: labels,
      datasets: [
        {
          label: "Dự kiến (giây)",
          data: expectedData,
          backgroundColor: "#3b82f6",
          borderRadius: 4,
        },
        {
          label: "Thực tế (giây)",
          data: actualData,
          backgroundColor: "#10b981",
          borderRadius: 4,
        },
      ],
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: { duration: 500 },
      scales: {
        y: {
          beginAtZero: true,
          grid: { color: "#232b3e" },
          ticks: { color: "#94a3b8", font: { size: 10 } },
        },
        x: {
          grid: { display: false },
          ticks: { color: "#94a3b8", font: { size: 10 } },
        },
      },
      plugins: {
        legend: {
          position: "top",
          labels: {
            color: "#f8fafc",
            font: { size: 11, family: "Inter" },
            boxWidth: 12,
          },
        },
      },
    },
  });
  document.getElementById("timeComparisonChart").style.height = "200px";
}

// ================= GRAPH & MAP LOGIC =================
const startNodeEl = document.getElementById("startNode");
for (let i = 0; i < 20; i++) {
  let opt = document.createElement("option");
  opt.value = i;
  opt.innerText = "Node " + i;
  if (i === 15) opt.selected = true;
  startNodeEl.appendChild(opt);
}

const graph = {
  0: [1, 5],
  1: [0, 6],
  2: [3, 7],
  3: [2, 4, 8],
  4: [3],
  5: [0, 6, 10],
  6: [1, 5, 7],
  7: [2, 6, 8, 12],
  8: [3, 7, 9, 13],
  9: [8, 14],
  10: [5, 11, 15],
  11: [10, 16],
  12: [7, 13, 17],
  13: [8, 12, 14, 18],
  14: [9, 13],
  15: [10, 16],
  16: [11, 15],
  17: [12, 18],
  18: [13, 17, 19],
  19: [18],
};

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
  "#ef4444",
  "#f97316",
  "#8b5cf6",
  "#ec4899",
  "#06b6d4",
  "#14b8a6",
];
let targets = new Set([4]);
let obstacles = new Set();
let finalCalculatedPath = [];

function initNodes() {
  const layer = document.getElementById("nodeLayer");
  layer.innerHTML = "";
  for (let i = 0; i < 20; i++) {
    const [x, y] = coords[i];
    const g = document.createElementNS("http://www.w3.org/2000/svg", "g");
    g.setAttribute("class", "node-group");
    g.setAttribute("id", "svg-node-" + i);
    g.onclick = () => handleNodeClick(i);
    const circle = document.createElementNS(
      "http://www.w3.org/2000/svg",
      "circle",
    );
    circle.setAttribute("cx", x);
    circle.setAttribute("cy", y);
    circle.setAttribute("r", "12");
    circle.setAttribute("class", "node-circle");
    const text = document.createElementNS("http://www.w3.org/2000/svg", "text");
    text.setAttribute("x", x);
    text.setAttribute("y", y);
    text.setAttribute("class", "node-text");
    text.textContent = i;
    g.appendChild(circle);
    g.appendChild(text);
    layer.appendChild(g);
  }
}

function handleNodeClick(nodeId) {
  const startNode = parseInt(document.getElementById("startNode").value);
  const tool = document.getElementById("mapTool").value;
  const mode = document.getElementById("orderModeSel").value;
  if (nodeId === startNode) return;

  if (tool === "target") {
    obstacles.delete(nodeId);
    if (mode === "single") {
      targets.clear();
      targets.add(nodeId);
    } else {
      if (targets.has(nodeId)) {
        targets.delete(nodeId);
      } else {
        if (targets.size >= 6) {
          alert("Chỉ được giao tối đa 6 đơn hàng cùng lúc.");
          return;
        }
        targets.add(nodeId);
      }
    }
  } else if (tool === "obstacle") {
    targets.delete(nodeId);
    if (obstacles.has(nodeId)) obstacles.delete(nodeId);
    else obstacles.add(nodeId);
  }
  calculateOverallPath();
}

function heuristic(a, b) {
  return (
    Math.abs(coords[a][0] - coords[b][0]) +
    Math.abs(coords[a][1] - coords[b][1])
  );
}

function findPath(start, end) {
  if (obstacles.has(start) || obstacles.has(end)) return null;
  const algo = document.getElementById("algoSel").value;

  if (algo === "bfs") {
    let q = [[start]];
    let visited = new Set([start]);
    while (q.length > 0) {
      let p = q.shift();
      let n = p[p.length - 1];
      if (n === end) return { path: p, visitedCount: visited.size };
      for (let nxt of graph[n]) {
        if (!visited.has(nxt) && !obstacles.has(nxt)) {
          visited.add(nxt);
          q.push([...p, nxt]);
        }
      }
    }
  } else if (algo === "dfs") {
    let stack = [[start]];
    let visited = new Set([start]);
    while (stack.length > 0) {
      let p = stack.pop();
      let n = p[p.length - 1];
      if (n === end) return { path: p, visitedCount: visited.size };
      for (let nxt of graph[n]) {
        if (!visited.has(nxt) && !obstacles.has(nxt)) {
          visited.add(nxt);
          stack.push([...p, nxt]);
        }
      }
    }
  } else if (algo === "astar") {
    let pq = [{ path: [start], g: 0, f: heuristic(start, end) }];
    let visited = new Map();
    visited.set(start, 0);
    while (pq.length > 0) {
      pq.sort((a, b) => a.f - b.f);
      let curr = pq.shift();
      let n = curr.path[curr.path.length - 1];
      if (n === end) return { path: curr.path, visitedCount: visited.size };
      for (let nxt of graph[n]) {
        if (obstacles.has(nxt)) continue;
        let newG = curr.g + 1;
        if (!visited.has(nxt) || newG < visited.get(nxt)) {
          visited.set(nxt, newG);
          pq.push({
            path: [...curr.path, nxt],
            g: newG,
            f: newG + heuristic(nxt, end),
          });
        }
      }
    }
  }
  return null;
}

// ==== ĐÃ SỬA LỖI TẠI HÀM NÀY ====
function calculateOverallPath() {
  const startNode = parseInt(document.getElementById("startNode").value);
  for (let i = 0; i < 20; i++) {
    let cls = "node-group";
    if (i === startNode) cls += " node-start";
    else if (obstacles.has(i)) cls += " node-obstacle";
    document.getElementById("svg-node-" + i).setAttribute("class", cls);
  }

  const pathLayer = document.getElementById("pathLayer");
  pathLayer.innerHTML = "";
  const resEl = document.getElementById("pathResult");
  const statsBar = document.getElementById("searchStats");
  const legendBox = document.getElementById("deliveryLegend");

  if (targets.size === 0) {
    resEl.innerText = "Vui lòng chọn điểm giao hàng";
    resEl.className = "err-msg";
    statsBar.style.display = "none";
    legendBox.style.display = "none";
    finalCalculatedPath = [];
    return;
  }

  let fullPath = [startNode];
  let currentS = startNode;
  let remainingTargets = new Set(targets);
  let totalSteps = 0;
  let totalVisitedNodes = 0;
  let orderedTargets = [];
  let pathSegments = [];

  while (remainingTargets.size > 0) {
    let bestPathObj = null,
      bestTarget = null,
      shortestLen = Infinity;
    for (let t of remainingTargets) {
      let res = findPath(currentS, t);
      if (res && res.path.length < shortestLen) {
        shortestLen = res.path.length;
        bestPathObj = res;
        bestTarget = t;
      }
    }

    // Nếu không còn đường tới TẤT CẢ các điểm còn lại thì dừng vòng lặp tính toán
    if (!bestPathObj) {
      break;
    }

    orderedTargets.push(bestTarget);
    pathSegments.push(bestPathObj.path);
    fullPath =
      fullPath.length > 1
        ? fullPath.concat(bestPathObj.path.slice(1))
        : bestPathObj.path;
    totalSteps += bestPathObj.path.length - 1;
    totalVisitedNodes += bestPathObj.visitedCount;
    currentS = bestTarget;
    remainingTargets.delete(bestTarget);
  }

  // CHỈ CẦN CÓ ÍT NHẤT 1 ĐIỂM ĐẾN ĐƯỢC LÀ XE SẼ CHẠY
  if (orderedTargets.length > 0) {
    finalCalculatedPath = fullPath;
    let msgText = "LỘ TRÌNH: " + fullPath.join(" ➔ ");

    // Xử lý báo lỗi gạch chéo cho các điểm không thể đến
    if (remainingTargets.size > 0) {
      let skippedArr = Array.from(remainingTargets);
      msgText +=
        "\n⚠️ BỎ QUA ĐƠN: " + skippedArr.join(", ") + " (Vướng vật cản)";
      resEl.className = "warn-msg";
      skippedArr.forEach((t) =>
        document.getElementById("svg-node-" + t).classList.add("node-skipped"),
      );
    } else {
      resEl.className = "";
    }

    resEl.innerText = msgText;
    document.getElementById("statSteps").innerText = totalSteps;
    document.getElementById("statVisited").innerText = totalVisitedNodes;
    statsBar.style.display = "flex";

    let legendHTML = `<div style="width:100%; text-align:center; margin-bottom:4px;">THỨ TỰ GIAO HÀNG</div>`;
    orderedTargets.forEach((node, idx) => {
      let colorIndex = Math.min(idx, 5);
      let colorHex = TARGET_COLORS[colorIndex];
      document
        .getElementById("svg-node-" + node)
        .classList.add("target-" + (colorIndex + 1));
      let segPts = pathSegments[idx].map((n) => coords[n].join(",")).join(" ");
      let poly = document.createElementNS(
        "http://www.w3.org/2000/svg",
        "polyline",
      );
      poly.setAttribute("class", "segment-path");
      poly.setAttribute("points", segPts);
      poly.setAttribute("stroke", colorHex);
      poly.setAttribute("style", `filter: drop-shadow(0 0 6px ${colorHex})`);
      pathLayer.appendChild(poly);
      legendHTML += `<div class="legend-item"><span class="legend-color" style="background:${colorHex}"></span> Đơn ${idx + 1} (Node ${node})</div>`;
      if (idx < orderedTargets.length - 1)
        legendHTML += `<div style="color:#475569;font-size:10px;">➔</div>`;
    });
    legendBox.innerHTML = legendHTML;
    legendBox.style.display = "flex";

    for (let i = 1; i < fullPath.length - 1; i++) {
      if (!targets.has(fullPath[i]))
        document
          .getElementById("svg-node-" + fullPath[i])
          .classList.add("node-path");
    }
  } else {
    // KHÔNG THỂ ĐẾN BẤT CỨ ĐIỂM NÀO
    resEl.innerText = "LỖI: Khu vực bị cô lập, không thể tìm đường!";
    resEl.className = "err-msg";
    statsBar.style.display = "none";
    legendBox.style.display = "none";
    finalCalculatedPath = [];
    targets.forEach((t) =>
      document.getElementById("svg-node-" + t).classList.add("target-1"),
    );
  }
}
// ===========================================

document.querySelectorAll("#startNode, #algoSel, #orderModeSel").forEach((el) =>
  el.addEventListener("change", () => {
    if (el.id === "orderModeSel" && el.value === "single" && targets.size > 1) {
      const first = Array.from(targets)[0];
      targets.clear();
      targets.add(first);
    }
    obstacles.delete(parseInt(document.getElementById("startNode").value));
    calculateOverallPath();
  }),
);
initNodes();
calculateOverallPath();

// ================= API GỬI LỆNH XUỐNG XE =================
document.getElementById("deliverBtn").addEventListener("click", async () => {
  if (finalCalculatedPath.length < 2) return alert("Lộ trình không hợp lệ!");
  const dir = document.getElementById("startDir").value;
  const btn = document.getElementById("deliverBtn");
  const pathString = finalCalculatedPath.join(",");

  btn.innerText = "ĐANG TRUYỀN DỮ LIỆU...";
  btn.style.opacity = "0.7";
  try {
    await fetch(`/deliver?dir=${dir}&path=${pathString}`);
  } catch (e) {}
  setTimeout(() => {
    btn.innerText = "🚀 NẠP LỘ TRÌNH & BẮT ĐẦU CHẠY";
    btn.style.opacity = "1";
  }, 1500);
});

async function sendCommand(cmd) {
  try {
    await fetch(cmd);
    console.log("Đã gửi lệnh sự cố: " + cmd);
  } catch (e) {
    console.error("Lỗi gửi lệnh", e);
  }
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
document.querySelectorAll(".grip").forEach((b) => {
  b.addEventListener(
    "pointerdown",
    guard((e) => {
      e.preventDefault();
      send(b.dataset.path);
    }),
    { passive: false },
  );
});
