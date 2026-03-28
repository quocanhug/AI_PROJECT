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

// ==================== Graph Data (Editable) ====================
let graph = {
  0:[1,5], 1:[0,6], 2:[3,7], 3:[2,4,8], 4:[3],
  5:[0,6,10], 6:[1,5,7], 7:[2,6,8,12], 8:[3,7,9,13], 9:[8,14],
  10:[5,11,15], 11:[10,16], 12:[7,13,17], 13:[8,12,14,18], 14:[9,13],
  15:[10,16], 16:[11,15], 17:[12,18], 18:[13,17,19], 19:[18]
};
const edgeWeights = {};
const coords = {
  0:[30,30],1:[100,30],2:[170,30],3:[240,30],4:[310,30],
  5:[30,100],6:[100,100],7:[170,100],8:[240,100],9:[310,100],
  10:[30,170],11:[100,170],12:[170,170],13:[240,170],14:[310,170],
  15:[30,240],16:[100,240],17:[170,240],18:[240,240],19:[310,240]
};
const TARGET_COLORS = ["#ba1a1a","#964400","#465f89","#0873df","#bd5700","#2e4770"];
let targets = new Set([4]), obstacles = new Set(), finalCalculatedPath = [];
let currentMapTool = 'target';
let edgeSelFirst = null;

// Animation state
let anim = { steps:[], path:[], currentStep:0, playing:false, speed:300, timer:null, active:false };

// ==================== Robot Direction Indicator ====================
let robotIndicator = null;
let robotCurrentNode = -1;
let robotCurrentDir = 2;   // JS dirs: 0=up, 1=right, 2=down, 3=left
let robotInitialDir = 2;   // Default: facing down (click arrow to cycle)
const DIR_ROTATION = [0, 90, 180, 270];
const JS_TO_CPP_DIR = [2, 1, 0, 3];  // JS→C++ direction mapping
const CPP_TO_JS_DIR = [2, 1, 0, 3];  // C++→JS direction mapping
const DIR_LABELS = ['⬆ Lên','➡ Phải','⬇ Xuống','⬅ Trái'];

function createRobotIndicator(){
  const layer=document.getElementById('robotLayer'); if(!layer) return;
  layer.innerHTML='';
  const g=document.createElementNS("http://www.w3.org/2000/svg","g");
  g.id="robotArrow"; g.setAttribute("class","robot-indicator"); g.style.display="none";
  const glow=document.createElementNS("http://www.w3.org/2000/svg","circle");
  glow.setAttribute("cx","0"); glow.setAttribute("cy","0"); glow.setAttribute("r","18"); glow.setAttribute("class","robot-glow");
  const arrow=document.createElementNS("http://www.w3.org/2000/svg","polygon");
  arrow.setAttribute("points","0,-18 -10,10 10,10"); arrow.setAttribute("class","robot-arrow");
  const dot=document.createElementNS("http://www.w3.org/2000/svg","circle");
  dot.setAttribute("cx","0"); dot.setAttribute("cy","0"); dot.setAttribute("r","4"); dot.setAttribute("class","robot-center");
  g.appendChild(glow); g.appendChild(arrow); g.appendChild(dot);
  layer.appendChild(g);
  g.addEventListener("click",(e)=>{
    e.stopPropagation();
    if(anim.active) return;
    robotInitialDir=(robotInitialDir+1)%4;
    robotCurrentDir=robotInitialDir;
    updateRobotIndicator(robotCurrentNode, robotCurrentDir);
    showToast(`Hướng đầu xe: ${DIR_LABELS[robotInitialDir]}`,"info");
  });
  robotIndicator=g;
}
function updateRobotIndicator(nodeId, dir){
  if(!robotIndicator||nodeId<0||nodeId>=20) return;
  const [x,y]=coords[nodeId]; const rot=DIR_ROTATION[dir];
  robotIndicator.setAttribute("transform",`translate(${x},${y}) rotate(${rot})`);
  robotIndicator.style.display="";
  robotCurrentNode=nodeId; robotCurrentDir=dir;
}
function hideRobotIndicator(){ if(robotIndicator) robotIndicator.style.display="none"; robotCurrentNode=-1; }
function getDirBetweenNodes(a,b){
  const dx=Math.sign(coords[b][0]-coords[a][0]);
  const dy=Math.sign(coords[b][1]-coords[a][1]);
  if(dy<0) return 0; if(dx>0) return 1; if(dy>0) return 2; if(dx<0) return 3; return 2;
}

function heuristic(a,b){ return Math.abs(coords[a][0]-coords[b][0])+Math.abs(coords[a][1]-coords[b][1]); }
function getEdgeWeight(a,b){ return edgeWeights[Math.min(a,b)+'-'+Math.max(a,b)]||1; }
function setEdgeWeight(a,b,w){ edgeWeights[Math.min(a,b)+'-'+Math.max(a,b)]=w; }
function getAlgoName(a){ return {bfs:"BFS",dfs:"DFS",ucs:"UCS",astar:"A*",greedy:"Greedy"}[a]||a; }

// ==================== Clock ====================
function updateClock(){
  const now=new Date();
  const ts=now.toLocaleTimeString('en-US',{hour:'2-digit',minute:'2-digit',second:'2-digit',hour12:true});
  const el=document.getElementById('real-time-clock'); if(el) el.textContent=ts;
  const ft=document.getElementById('footer-update-time'); if(ft) ft.textContent=ts+' (GMT+7)';
}
setInterval(updateClock,1000); updateClock();

// ==================== Toast ====================
let toastContainer;
function ensureToastContainer(){ if(!toastContainer){ toastContainer=document.createElement('div'); toastContainer.className='toast-container'; document.body.appendChild(toastContainer); } }
function showToast(msg,type="info"){
  ensureToastContainer();
  const t=document.createElement("div"); t.className="toast "+type; t.textContent=msg;
  toastContainer.appendChild(t);
  setTimeout(()=>{ t.style.opacity="0"; t.style.transform="translateX(20px)"; setTimeout(()=>t.remove(),300); },3000);
}

// ==================== WebSocket ====================
let ws=null, wsReconnectTimer=null, pingTimer=null;
function connectWebSocket(){
  const host=window.location.hostname||"192.168.4.1";
  const port=window.location.port||"80";
  try{ ws=new WebSocket(`ws://${host}:${port}/ws`); }catch(e){ scheduleReconnect(); return; }
  ws.onopen=()=>{
    setWsStatus(true); showToast("Kết nối ESP32 thành công","success");
    if(wsReconnectTimer){ clearTimeout(wsReconnectTimer); wsReconnectTimer=null; }
    pingTimer=setInterval(()=>{ try{ws.send(JSON.stringify({type:"PING"}));}catch(e){} },5000);
  };
  ws.onmessage=(evt)=>{ try{handleWsMessage(JSON.parse(evt.data));}catch(e){} };
  ws.onclose=()=>{ setWsStatus(false); if(pingTimer){clearInterval(pingTimer);pingTimer=null;} scheduleReconnect(); };
  ws.onerror=()=>setWsStatus(false);
}
function scheduleReconnect(){ if(wsReconnectTimer) return; wsReconnectTimer=setTimeout(()=>{ wsReconnectTimer=null; connectWebSocket(); },3000); }
function setWsStatus(c){
  document.getElementById('wsStatusText').textContent=c?'Trực tuyến':'Mất kết nối';
  document.getElementById('wsStatusDot').className='w-2 h-2 rounded-full '+(c?'bg-green-400 animate-pulse':'bg-yellow-400');
  document.getElementById('connBadge').textContent=c?'Kết nối ổn định':'Đang kết nối...';
  document.getElementById('connBadge').className='px-3 py-1 text-[10px] font-bold rounded-full uppercase tracking-widest border '+(c?'bg-green-100 text-green-700 border-green-200':'bg-yellow-100 text-yellow-700 border-yellow-200');
}
function wsSend(obj){ if(ws&&ws.readyState===WebSocket.OPEN) ws.send(JSON.stringify(obj)); }

// ==================== WS Message Handler ====================
function handleWsMessage(data){
  if(data.type==="TELEMETRY") updateTelemetry(data);
  else if(data.type==="ROUTE_ACK") showToast(`Route nạp: ${data.commands} lệnh`,"success");
  else if(data.type==="OBSTACLE_DETECTED"){
    showToast(`⚠️ Vật cản tại (${data.position.row},${data.position.col})!`,"error");
    updateRobotState("OBSTACLE");
    handleDynamicReroute(data);
  }
  else if(data.type==="COMPLETED"){
    showToast(`Hoàn thành! ${data.intersections} giao lộ, ${data.distance_cm}cm`,"success");
    updateRobotState("DONE"); handleDeliveryCompleted();
  }
  else if(data.type==="PONG"||data.type==="PING"){}
}

function handleDynamicReroute(data){
  const obsNode = data.position.row * 5 + data.position.col;
  if(obsNode>=0&&obsNode<20){ obstacles.add(obsNode); }
  const robotNode = data.robot_position.row * 5 + data.robot_position.col;
  const tarArr = Array.from(targets);
  if(tarArr.length>0){
    const algo = document.getElementById("algoSel").value;
    const result = findPathAnimated(robotNode, tarArr[0], algo);
    if(result){
      const cmds = pathToCommands(result.path, robotCurrentDir);
      wsSend({type:"ROUTE",commands:cmds,total_steps:cmds.length,rerouted:true});
      animRunSingle(result, getAlgoName(algo), '🔄 Tái định tuyến');
      showToast("🔄 Đang tính lại đường đi mới...","info");
    } else { showToast("❌ Không tìm được đường đi mới!","error"); }
  }
  updateNodeVisuals();
  calculateOverallPath(); // Cập nhật lại UI đường đi
}

function handleDeliveryCompleted(){
  if(!deliveryInProgress) return;
  deliveryInProgress=false;
  const actualTime=((Date.now()-deliveryStartTime)/1000).toFixed(1);
  const banner=document.getElementById('deliveryStatusBanner');
  const icon=document.getElementById('deliveryStatusIcon');
  const text=document.getElementById('deliveryStatusText');
  banner.classList.remove('hidden');
  if(deliveryExpectedTime&&parseFloat(actualTime)>deliveryExpectedTime){
    const lateBy=(parseFloat(actualTime)-deliveryExpectedTime).toFixed(1);
    banner.className='flex items-center gap-2 rounded-xl p-3 bg-red-50 border border-red-200';
    icon.textContent='warning'; icon.className='material-symbols-outlined text-lg text-red-600';
    text.textContent=`✅ Đã giao xong — ⚠️ TRỄ ${lateBy}s (${actualTime}s / ${deliveryExpectedTime}s)`;
    text.className='text-sm font-bold text-red-700';
  } else {
    banner.className='flex items-center gap-2 rounded-xl p-3 bg-green-50 border border-green-200';
    icon.textContent='check_circle'; icon.className='material-symbols-outlined text-lg text-green-600';
    text.textContent=`✅ Đã giao xong — Thời gian: ${actualTime}s`; text.className='text-sm font-bold text-green-700';
  }
}

function updateTelemetry(d){
  const avg=(Math.abs(d.speedL||0)+Math.abs(d.speedR||0))/2;
  document.getElementById('teleSpeedVal').textContent=avg.toFixed(2);
  document.getElementById('speedBar').style.width=Math.min(avg/1.5*100,100)+'%';
  document.getElementById('teleDistVal').textContent=(d.distance||0).toFixed(1);
  document.getElementById('teleStateVal').textContent=d.state||'IDLE';
  document.getElementById('teleStepVal').textContent=`${d.step||0} / ${d.total||0}`;
  const obs=d.obstacle||999;
  document.getElementById('sonarStatus').textContent=obs>200?'-- cm':obs.toFixed(1)+' cm';
  document.getElementById('sonarStatus').className='text-xs font-bold '+(obs<20?'text-error':'text-green-600');
  document.getElementById('autoSpeedVal').textContent=avg.toFixed(1);
  document.getElementById('autoObstacleVal').textContent=obs>200?'--':obs.toFixed(0);
  if(d.sensors){ for(let i=0;i<5;i++){ const el=document.getElementById('sm'+i); if(el) el.className='w-3 h-3 rounded-full '+(d.sensors[i]?'bg-primary shadow-[0_0_6px_rgba(0,90,180,0.5)]':'bg-surface-container'); } }
  if(d.state) updateRobotState(d.state);
  if(d.robotNode!==undefined && d.robotNode>=0){
    const jsDir=CPP_TO_JS_DIR[d.robotDir||0];
    updateRobotIndicator(d.robotNode, jsDir);
  }
}
function updateRobotState(s){
  const map={"IDLE":"IDLE","FOLLOWING_LINE":"DÒ LINE","AT_INTERSECTION":"GIAO LỘ","OBSTACLE":"VẬT CẢN","REROUTING":"TÁI ĐỊNH TUYẾN","DONE":"HOÀN THÀNH"};
  document.getElementById('teleStateVal').textContent=map[s]||s;
}

// ==================== Tab Switching ====================
async function switchTab(tab){
  document.querySelectorAll('.panel-view').forEach(el=>el.classList.remove('active'));
  document.querySelectorAll('.nav-link').forEach(el=>el.classList.remove('active-nav'));
  document.getElementById('panel-'+tab).classList.add('active');
  document.getElementById('nav-'+tab).classList.add('active-nav');
  currentMode=tab;
  const titles={manual:'Hệ thống: Vận hành Thủ công',auto:'Hệ thống: Chế độ Tự động',stats:'Mission Control Center — Thống kê'};
  document.getElementById('headerTitle').textContent=titles[tab];
  if(statsInterval){clearInterval(statsInterval);statsInterval=null;}
  if(tab!=="stats"){ try{await fetch("/setMode?m="+(tab==="manual"?"manual":"line"));}catch(e){} }
  else { fetchRealStats(); statsInterval=setInterval(fetchRealStats,3000); }
}
async function fetchRealStats(){
  try{ const r=await fetch("/api/stats"); if(r.ok){ const d=await r.json();
    document.getElementById('kpiDelivered').textContent=d.delivered;
    document.getElementById('kpiAvgTime').textContent=d.avgTime;
    document.getElementById('kpiSuccess').textContent=d.efficiency;
    document.getElementById('kpiDistance').textContent=d.totalDistance||'--';
  }}catch(e){}
}

// ==================== Edge Rendering (Dynamic) ====================
function renderEdges(){
  const layer=document.querySelector('.track-lines'); layer.innerHTML='';
  const drawn=new Set();
  for(const[nid,neighbors] of Object.entries(graph)){
    for(const nb of neighbors){
      const a=parseInt(nid),b=nb;
      const key=Math.min(a,b)+'-'+Math.max(a,b);
      if(drawn.has(key)) continue; drawn.add(key);
      const[x1,y1]=coords[a],[x2,y2]=coords[b];
      const line=document.createElementNS("http://www.w3.org/2000/svg","line");
      line.setAttribute("x1",x1);line.setAttribute("y1",y1);
      line.setAttribute("x2",x2);line.setAttribute("y2",y2);
      layer.appendChild(line);
      const w=getEdgeWeight(a,b);
      if(w!==1){
        const txt=document.createElementNS("http://www.w3.org/2000/svg","text");
        txt.setAttribute("x",(x1+x2)/2); txt.setAttribute("y",(y1+y2)/2-8);
        txt.setAttribute("class","edge-weight-label"); txt.textContent=w;
        layer.appendChild(txt);
      }
    }
  }
}

function toggleEdge(a,b){
  if(a===b) return;
  const has=graph[a].includes(b);
  if(has){ graph[a]=graph[a].filter(n=>n!==b); graph[b]=graph[b].filter(n=>n!==a); delete edgeWeights[Math.min(a,b)+'-'+Math.max(a,b)]; }
  else { graph[a].push(b); graph[b].push(a); }
  renderEdges(); updateNodeVisuals(); calculateOverallPath();
  return !has; // true if added
}

// ==================== Map Nodes ====================
const startNodeEl = document.getElementById("startNode");
for(let i=0;i<20;i++){ let o=document.createElement("option"); o.value=i; o.innerText="Node "+i; if(i===15) o.selected=true; startNodeEl.appendChild(o); }

function initNodes(){
  const layer=document.getElementById("nodeLayer"); layer.innerHTML="";
  for(let i=0;i<20;i++){
    const[x,y]=coords[i];
    const g=document.createElementNS("http://www.w3.org/2000/svg","g");
    g.setAttribute("class","node-group"); g.setAttribute("id","svg-node-"+i);
    g.onclick=()=>handleNodeClick(i);
    const c=document.createElementNS("http://www.w3.org/2000/svg","circle");
    c.setAttribute("cx",x);c.setAttribute("cy",y);c.setAttribute("r","12");c.setAttribute("class","node-circle");
    const t=document.createElementNS("http://www.w3.org/2000/svg","text");
    t.setAttribute("x",x);t.setAttribute("y",y);t.setAttribute("class","node-text");t.textContent=i;
    g.appendChild(c);g.appendChild(t);layer.appendChild(g);
  }
}

// ==================== Map Tools ====================
function setMapTool(tool){
  currentMapTool=tool; edgeSelFirst=null;
  document.getElementById('toolStart').classList.toggle('active',tool==='start');
  document.getElementById('toolTarget').classList.toggle('active',tool==='target');
  document.getElementById('toolObstacle').classList.toggle('active',tool==='obstacle');
  document.getElementById('toolEdge').classList.toggle('active',tool==='edge');
  document.getElementById('toolWeight').classList.toggle('active',tool==='weight');
  updateEdgeSelUI();
}
function updateEdgeSelUI(){
  for(let i=0;i<20;i++) document.getElementById("svg-node-"+i).classList.remove("node-edge-sel");
  if(edgeSelFirst!==null) document.getElementById("svg-node-"+edgeSelFirst).classList.add("node-edge-sel");
}

// ==================== Node Click Handler ====================
function handleNodeClick(nid){
  if(anim.active){ animReset(); } 
  const sn=parseInt(document.getElementById("startNode").value);

  // Tool: Start — set start node by clicking
  if(currentMapTool==="start"){
    targets.delete(nid); obstacles.delete(nid);
    document.getElementById("startNode").value=nid;
    showToast(`Điểm xuất phát: Node ${nid}`,"success");
    updateNodeVisuals(); calculateOverallPath(); return;
  }
  if(currentMapTool==="edge"){
    if(edgeSelFirst===null){ edgeSelFirst=nid; updateEdgeSelUI(); showToast(`Node ${nid} chọn — click node khác để tạo kết nối`,"info"); }
    else { if(edgeSelFirst!==nid){ const added=toggleEdge(edgeSelFirst,nid); showToast(`Edge ${edgeSelFirst}↔${nid} ${added?'đã thêm':'đã xóa'}`,"success"); } edgeSelFirst=null; updateEdgeSelUI(); }
    return;
  }
  if(currentMapTool==="weight"){
    if(edgeSelFirst===null){ edgeSelFirst=nid; updateEdgeSelUI(); showToast(`Node ${nid} chọn — click node kề để đặt trọng số`,"info"); }
    else {
      if(edgeSelFirst!==nid&&graph[edgeSelFirst].includes(nid)){
        const cur=getEdgeWeight(edgeSelFirst,nid);
        const nw=prompt(`Trọng số edge ${edgeSelFirst}↔${nid} (hiện: ${cur}):`,cur);
        if(nw!==null&&!isNaN(parseInt(nw))&&parseInt(nw)>0){ setEdgeWeight(edgeSelFirst,nid,parseInt(nw)); renderEdges(); updateNodeVisuals(); calculateOverallPath(); showToast(`Trọng số ${edgeSelFirst}↔${nid} = ${nw}`,"success"); }
      } else if(edgeSelFirst!==nid){ showToast("Không có kết nối giữa 2 node này!","error"); }
      edgeSelFirst=null; updateEdgeSelUI();
    }
    return;
  }
  // Tool: Start node selected — can't click on it again
  if(nid===sn && currentMapTool!=="start") return;
  if(currentMapTool==="target"){
    obstacles.delete(nid);
    const mode=document.getElementById("orderModeSel").value;
    if(mode==="single"){ targets.clear(); targets.add(nid); }
    else { if(targets.has(nid)) targets.delete(nid); else { if(targets.size>=6){showToast("Tối đa 6 đơn!","error");return;} targets.add(nid); } }
  } else if(currentMapTool==="obstacle"){
    targets.delete(nid);
    if(obstacles.has(nid)) obstacles.delete(nid); else obstacles.add(nid);
  }
  updateNodeVisuals();
  calculateOverallPath(); // Cập nhật lộ trình ngay sau khi click
}

// ==================== Update Node Visuals ====================
function updateNodeVisuals(){
  const sn=parseInt(document.getElementById("startNode").value);
  for(let i=0;i<20;i++){
    let cls="node-group";
    if(i===sn) cls+=" node-start";
    else if(obstacles.has(i)) cls+=" node-obstacle";
    else if(targets.has(i)) cls+=" target-1";
    document.getElementById("svg-node-"+i).setAttribute("class",cls);
  }
  document.getElementById("pathLayer").innerHTML="";
  const resEl=document.getElementById("pathResult");
  if(targets.size===0){
    resEl.innerText="Chọn điểm xuất phát và điểm đến trên bản đồ";
    resEl.className="p-4 text-sm text-on-surface-variant text-center";
    document.getElementById("autoStepsVal").innerHTML='— <span class="text-xs font-medium text-on-surface-variant italic">bước</span>';
    document.getElementById("autoVisitedVal").textContent='—';
  }
  updateRobotIndicator(sn, robotInitialDir);
}

// ==================== AI Algorithms ====================
function shuffle(arr){ let a=[...arr]; for(let i=a.length-1;i>0;i--){let j=Math.floor(Math.random()*(i+1));[a[i],a[j]]=[a[j],a[i]];} return a; }

function findPathAnimated(start,end,algo){
  if(obstacles.has(start)||obstacles.has(end)) return null;
  algo=algo||document.getElementById("algoSel").value;
  const steps=[];

  if(algo==="bfs"){
    let queue=[[start]], explored=new Set(), frontier=new Set([start]);
    while(queue.length){
      let path=queue.shift(), node=path[path.length-1];
      frontier.delete(node); explored.add(node);
      steps.push({current:node,explored:new Set(explored),frontier:new Set(frontier),from:path.length>1?path[path.length-2]:-1});
      if(node===end) return {path,steps,visitedCount:explored.size,cost:path.length-1};
      for(let nx of shuffle(graph[node]||[])){ if(!explored.has(nx)&&!frontier.has(nx)&&!obstacles.has(nx)){ frontier.add(nx); queue.push([...path,nx]); } }
    }
  } else if(algo==="dfs"){
    let stack=[[start]], explored=new Set(), frontier=new Set([start]);
    while(stack.length){
      let path=stack.pop(), node=path[path.length-1];
      frontier.delete(node);
      if(explored.has(node)) continue; explored.add(node);
      steps.push({current:node,explored:new Set(explored),frontier:new Set(frontier),from:path.length>1?path[path.length-2]:-1});
      if(node===end) return {path,steps,visitedCount:explored.size,cost:path.length-1};
      for(let nx of shuffle(graph[node]||[])){ if(!explored.has(nx)&&!obstacles.has(nx)){ frontier.add(nx); stack.push([...path,nx]); } }
    }
  } else if(algo==="ucs"){
    let pq=[{path:[start],cost:0}], best=new Map(), frontier=new Set([start]);
    best.set(start,0);
    while(pq.length){
      pq.sort((a,b)=>a.cost-b.cost); let cur=pq.shift(), node=cur.path[cur.path.length-1];
      frontier.delete(node);
      steps.push({current:node,explored:new Set(best.keys()),frontier:new Set(frontier),from:cur.path.length>1?cur.path[cur.path.length-2]:-1});
      if(node===end) return {path:cur.path,steps,visitedCount:best.size,cost:cur.cost};
      for(let nx of shuffle(graph[node]||[])){ if(obstacles.has(nx)) continue; let nc=cur.cost+getEdgeWeight(node,nx);
        if(!best.has(nx)||nc<best.get(nx)){ best.set(nx,nc); frontier.add(nx); pq.push({path:[...cur.path,nx],cost:nc}); } }
    }
  } else if(algo==="greedy"){
    let pq=[{path:[start],h:heuristic(start,end)}], explored=new Set(), frontier=new Set([start]);
    while(pq.length){
      pq.sort((a,b)=>a.h-b.h); let cur=pq.shift(), node=cur.path[cur.path.length-1];
      frontier.delete(node);
      if(explored.has(node)) continue; explored.add(node);
      steps.push({current:node,explored:new Set(explored),frontier:new Set(frontier),from:cur.path.length>1?cur.path[cur.path.length-2]:-1});
      if(node===end) return {path:cur.path,steps,visitedCount:explored.size,cost:cur.path.length-1};
      for(let nx of shuffle(graph[node]||[])){ if(!explored.has(nx)&&!obstacles.has(nx)){ frontier.add(nx); pq.push({path:[...cur.path,nx],h:heuristic(nx,end)}); } }
    }
  } else { // astar
    let pq=[{path:[start],g:0,f:heuristic(start,end)}], gS=new Map(), explored=new Set(), frontier=new Set([start]);
    gS.set(start,0);
    while(pq.length){
      pq.sort((a,b)=>a.f-b.f); let cur=pq.shift(), node=cur.path[cur.path.length-1];
      frontier.delete(node);
      if(explored.has(node)) continue; explored.add(node);
      steps.push({current:node,explored:new Set(explored),frontier:new Set(frontier),from:cur.path.length>1?cur.path[cur.path.length-2]:-1});
      if(node===end) return {path:cur.path,steps,visitedCount:explored.size,cost:cur.g};
      for(let nx of shuffle(graph[node]||[])){ if(obstacles.has(nx)||explored.has(nx)) continue; let ng=cur.g+getEdgeWeight(node,nx);
        if(!gS.has(nx)||ng<gS.get(nx)){ gS.set(nx,ng); frontier.add(nx); pq.push({path:[...cur.path,nx],g:ng,f:ng+heuristic(nx,end)}); } }
    }
  }
  return null;
}

function findPath(start,end,algo){
  const r=findPathAnimated(start,end,algo);
  if(!r) return null;
  return {path:r.path,visitedCount:r.visitedCount,cost:r.cost};
}

// ==================== Animation Engine ====================
const SEG_COLORS = ["#16a34a","#0873df","#964400","#ba1a1a","#465f89","#bd5700"];

function findPathToNearest(start, targetSet, algo){
  if(obstacles.has(start)) return null;
  algo=algo||document.getElementById("algoSel").value;
  const steps=[];

  if(algo==="bfs"){
    let queue=[[start]], explored=new Set(), frontier=new Set([start]);
    while(queue.length){
      let path=queue.shift(), node=path[path.length-1];
      frontier.delete(node); explored.add(node);
      steps.push({current:node,explored:new Set(explored),frontier:new Set(frontier),from:path.length>1?path[path.length-2]:-1});
      if(targetSet.has(node)) return {path,steps,visitedCount:explored.size,cost:path.length-1,foundTarget:node};
      for(let nx of shuffle(graph[node]||[])){ if(!explored.has(nx)&&!frontier.has(nx)&&!obstacles.has(nx)){ frontier.add(nx); queue.push([...path,nx]); } }
    }
  } else if(algo==="dfs"){
    let stack=[[start]], explored=new Set(), frontier=new Set([start]);
    while(stack.length){
      let path=stack.pop(), node=path[path.length-1];
      frontier.delete(node);
      if(explored.has(node)) continue; explored.add(node);
      steps.push({current:node,explored:new Set(explored),frontier:new Set(frontier),from:path.length>1?path[path.length-2]:-1});
      if(targetSet.has(node)) return {path,steps,visitedCount:explored.size,cost:path.length-1,foundTarget:node};
      for(let nx of shuffle(graph[node]||[])){ if(!explored.has(nx)&&!obstacles.has(nx)){ frontier.add(nx); stack.push([...path,nx]); } }
    }
  } else if(algo==="ucs"){
    let pq=[{path:[start],cost:0}], best=new Map(), frontier=new Set([start]);
    best.set(start,0);
    while(pq.length){
      pq.sort((a,b)=>a.cost-b.cost); let cur=pq.shift(), node=cur.path[cur.path.length-1];
      frontier.delete(node);
      steps.push({current:node,explored:new Set(best.keys()),frontier:new Set(frontier),from:cur.path.length>1?cur.path[cur.path.length-2]:-1});
      if(targetSet.has(node)) return {path:cur.path,steps,visitedCount:best.size,cost:cur.cost,foundTarget:node};
      for(let nx of shuffle(graph[node]||[])){ if(obstacles.has(nx)) continue; let nc=cur.cost+getEdgeWeight(node,nx);
        if(!best.has(nx)||nc<best.get(nx)){ best.set(nx,nc); frontier.add(nx); pq.push({path:[...cur.path,nx],cost:nc}); } }
    }
  } else if(algo==="greedy"){
    let pq=[], explored=new Set(), frontier=new Set([start]);
    for(let t of targetSet) pq.push({path:[start],h:heuristic(start,t),target:t});
    pq.sort((a,b)=>a.h-b.h);
    pq=[{path:[start],h:Math.min(...[...targetSet].map(t=>heuristic(start,t)))}];
    while(pq.length){
      pq.sort((a,b)=>a.h-b.h); let cur=pq.shift(), node=cur.path[cur.path.length-1];
      frontier.delete(node);
      if(explored.has(node)) continue; explored.add(node);
      steps.push({current:node,explored:new Set(explored),frontier:new Set(frontier),from:cur.path.length>1?cur.path[cur.path.length-2]:-1});
      if(targetSet.has(node)) return {path:cur.path,steps,visitedCount:explored.size,cost:cur.path.length-1,foundTarget:node};
      for(let nx of shuffle(graph[node]||[])){ if(!explored.has(nx)&&!obstacles.has(nx)){
        frontier.add(nx); pq.push({path:[...cur.path,nx],h:Math.min(...[...targetSet].map(t=>heuristic(nx,t)))}); } }
    }
  } else { // astar
    let pq=[{path:[start],g:0,f:Math.min(...[...targetSet].map(t=>heuristic(start,t)))}];
    let gS=new Map(), explored=new Set(), frontier=new Set([start]);
    gS.set(start,0);
    while(pq.length){
      pq.sort((a,b)=>a.f-b.f); let cur=pq.shift(), node=cur.path[cur.path.length-1];
      frontier.delete(node);
      if(explored.has(node)) continue; explored.add(node);
      steps.push({current:node,explored:new Set(explored),frontier:new Set(frontier),from:cur.path.length>1?cur.path[cur.path.length-2]:-1});
      if(targetSet.has(node)) return {path:cur.path,steps,visitedCount:explored.size,cost:cur.g,foundTarget:node};
      for(let nx of shuffle(graph[node]||[])){ if(obstacles.has(nx)||explored.has(nx)) continue; let ng=cur.g+getEdgeWeight(node,nx);
        if(!gS.has(nx)||ng<gS.get(nx)){ gS.set(nx,ng); frontier.add(nx);
          pq.push({path:[...cur.path,nx],g:ng,f:ng+Math.min(...[...targetSet].map(t=>heuristic(nx,t)))}); } }
    }
  }
  return null;
}

function buildSegments(sn, tarArr, algo){
  let ordered=[], currentS=sn, remaining=new Set(tarArr);
  const dm=document.getElementById("deliveryModeSel").value;
  if(dm==="express"&&tarArr.length>1){
    ordered.push(tarArr[tarArr.length-1]); remaining.delete(tarArr[tarArr.length-1]);
  }
  while(remaining.size){
    const r = findPathToNearest(currentS, remaining, algo);
    if(!r) break;
    ordered.push(r.foundTarget); remaining.delete(r.foundTarget); currentS=r.foundTarget;
  }
  let segs=[], fullPath=[sn]; currentS=sn;
  let totalSteps=0, totalExplored=0, totalCost=0;
  for(let tar of ordered){
    const r=findPathAnimated(currentS,tar,algo); if(!r) continue;
    segs.push({steps:r.steps,path:r.path,from:currentS,to:tar,visitedCount:r.visitedCount,cost:r.cost});
    totalSteps+=r.path.length-1; totalExplored+=r.visitedCount; totalCost+=r.cost;
    fullPath=fullPath.concat(r.path.slice(1)); currentS=tar;
  }
  return {segs,fullPath,ordered,totalSteps,totalExplored,totalCost};
}

function flattenSegments(segs){
  let all=[];
  for(let si=0;si<segs.length;si++){
    const seg=segs[si];
    for(let j=0;j<seg.steps.length;j++){
      all.push({...seg.steps[j], segIdx:si, isLast:j===seg.steps.length-1, segPath:seg.path});
    }
  }
  return all;
}

function animStart(){
  const sn=parseInt(document.getElementById("startNode").value);
  const tarArr=Array.from(targets);
  if(!tarArr.length){ showToast("Chọn điểm đích trước!","error"); return; }
  const algo=document.getElementById("algoSel").value;
  const {segs,fullPath,ordered,totalSteps,totalExplored,totalCost}=buildSegments(sn,tarArr,algo);
  if(!segs.length){ showToast("Không tìm được đường do vật cản!","error"); return; }
  animStop(); animClearVis();
  anim.steps=flattenSegments(segs); anim.segPaths=segs.map(s=>s.path);
  anim.fullPath=fullPath; anim.path=fullPath;
  anim.currentStep=0; anim.active=true;
  anim.algoName=getAlgoName(algo); anim.resultSteps=totalSteps;
  anim.resultExplored=totalExplored; anim.resultCost=totalCost;
  anim.segCount=segs.length; anim.prevSegIdx=-1;
  updateNodeVisuals();
  document.getElementById("animationControls").classList.remove("hidden");
  document.getElementById("pathLayer").innerHTML='';
  updateAnimUI();
  anim.playing=true; updateAnimUI(); animTick();
}

function animRunSingle(result, algoName, prefix){
  animStop(); animClearVis();
  anim.steps=result.steps.map((s,j)=>({...s,segIdx:0,isLast:j===result.steps.length-1,segPath:result.path}));
  anim.segPaths=[result.path]; anim.fullPath=result.path; anim.path=result.path;
  anim.currentStep=0; anim.active=true;
  anim.algoName=algoName; anim.resultSteps=result.path.length-1;
  anim.resultExplored=result.visitedCount; anim.resultCost=result.cost;
  anim.segCount=1; anim.prevSegIdx=-1;
  updateNodeVisuals();
  document.getElementById("animationControls").classList.remove("hidden");
  document.getElementById("pathLayer").innerHTML='';
  updateAnimUI();
  anim.playing=true; updateAnimUI(); animTick();
}

function animPlay(){
  if(!anim.active) return;
  if(anim.currentStep>=anim.steps.length){ animShowFinalPath(); return; }
  anim.playing=true; updateAnimUI(); animTick();
}
function animTick(){
  if(!anim.playing||anim.currentStep>=anim.steps.length){
    if(anim.currentStep>=anim.steps.length) animShowFinalPath();
    anim.playing=false; updateAnimUI(); return;
  }
  animRenderStep(anim.steps[anim.currentStep]); anim.currentStep++;
  updateAnimUI();
  anim.timer=setTimeout(animTick,anim.speed);
}
function animPause(){ anim.playing=false; if(anim.timer){clearTimeout(anim.timer);anim.timer=null;} updateAnimUI(); }
function animStepFwd(){
  if(!anim.active) return;
  if(anim.currentStep>=anim.steps.length){ animShowFinalPath(); return; }
  animPause(); animRenderStep(anim.steps[anim.currentStep]); anim.currentStep++; updateAnimUI();
}
function animReset(){
  animStop(); animClearVis(); anim.currentStep=0; anim.active=false;
  document.getElementById("animationControls").classList.add("hidden");
  document.getElementById("pathLayer").innerHTML='';
  updateNodeVisuals();
  calculateOverallPath(); // Cập nhật lại UI sau khi reset
}
function animStop(){ anim.playing=false; if(anim.timer){clearTimeout(anim.timer);anim.timer=null;} }

function animClearVis(){
  for(let i=0;i<20;i++) document.getElementById("svg-node-"+i).classList.remove("node-explored","node-frontier","node-current","node-final-path","node-skipped");
  const fl=document.getElementById("flowLayer"); if(fl) fl.innerHTML='';
}

function drawSegPath(path, idx){
  const col=SEG_COLORS[idx%SEG_COLORS.length];
  const pts=path.map(n=>coords[n].join(",")).join(" ");
  const poly=document.createElementNS("http://www.w3.org/2000/svg","polyline");
  poly.setAttribute("class","segment-path"); poly.setAttribute("points",pts);
  poly.setAttribute("stroke",col); poly.setAttribute("style",`filter:drop-shadow(0 0 5px ${col}40)`);
  document.getElementById("pathLayer").appendChild(poly);
}

function getFlowLayer(){
  let fl=document.getElementById("flowLayer");
  if(!fl){
    fl=document.createElementNS("http://www.w3.org/2000/svg","g");
    fl.id="flowLayer";
    const nodeLayer=document.getElementById("nodeLayer");
    nodeLayer.parentNode.insertBefore(fl,nodeLayer);
  }
  return fl;
}
function drawFlowEdge(layer, fromN, toN, cls){
  const [x1,y1]=coords[fromN],[x2,y2]=coords[toN];
  const line=document.createElementNS("http://www.w3.org/2000/svg","line");
  line.setAttribute("x1",x1);line.setAttribute("y1",y1);
  line.setAttribute("x2",x2);line.setAttribute("y2",y2);
  line.setAttribute("class","flow-edge "+cls);
  const len=Math.sqrt((x2-x1)**2+(y2-y1)**2);
  line.style.strokeDasharray=len;
  line.style.strokeDashoffset=len;
  layer.appendChild(line);
  line.animate([{strokeDashoffset:len+'px'},{strokeDashoffset:'0px'}],
    {duration:220,fill:'forwards',easing:'ease-out'});
}

function animRenderStep(step){
  const sn=parseInt(document.getElementById("startNode").value);
  const flowLayer=getFlowLayer();
  if(step.segIdx!==anim.prevSegIdx){
    if(anim.prevSegIdx>=0){
      drawSegPath(anim.segPaths[anim.prevSegIdx], anim.prevSegIdx);
      for(let n of anim.segPaths[anim.prevSegIdx]){
        if(n===sn||targets.has(n)||obstacles.has(n)) continue;
        const el=document.getElementById("svg-node-"+n);
        el.classList.remove("node-explored","node-frontier","node-current");
        el.classList.add("node-final-path");
      }
    }
    for(let i=0;i<20;i++){
      document.getElementById("svg-node-"+i).classList.remove("node-explored","node-frontier","node-current");
    }
    flowLayer.innerHTML='';
    anim.prevSegIdx=step.segIdx;
    if(anim.segCount>1) showToast(`🚚 Đoạn ${step.segIdx+1}/${anim.segCount}: đang tìm đường...`,"info");
  }

  flowLayer.querySelectorAll(".edge-current").forEach(el=>{
    el.classList.remove("edge-current"); el.classList.add("edge-explored");
    el.style.strokeDashoffset='0px';
  });
  if(step.from!==undefined && step.from>=0){
    drawFlowEdge(flowLayer, step.from, step.current, "edge-current");
  }

  for(let n of step.explored){
    if(n===sn||targets.has(n)||obstacles.has(n)) continue;
    const el=document.getElementById("svg-node-"+n);
    if(!el.classList.contains("node-final-path")) el.classList.add("node-explored");
  }
  for(let n of step.frontier){
    if(n===sn||targets.has(n)||obstacles.has(n)) continue;
    const el=document.getElementById("svg-node-"+n);
    if(!el.classList.contains("node-final-path")) el.classList.add("node-frontier");
  }
  if(step.current!==sn&&!targets.has(step.current)&&!obstacles.has(step.current)){
    const el=document.getElementById("svg-node-"+step.current);
    el.classList.remove("node-explored","node-frontier");
    el.classList.add("node-current");
  }
  document.getElementById("svg-node-"+sn).classList.add("node-start");
  let ti=0; for(let t of targets){ document.getElementById("svg-node-"+t).setAttribute("class","node-group target-"+(Math.min(ti,5)+1)); ti++; }
  for(let o of obstacles) document.getElementById("svg-node-"+o).classList.add("node-obstacle");
}

function animShowFinalPath(){
  anim.playing=false; updateAnimUI();
  if(!anim.fullPath||!anim.fullPath.length) return;
  const sn=parseInt(document.getElementById("startNode").value);
  const fl=document.getElementById("flowLayer"); if(fl) fl.innerHTML='';
  if(anim.prevSegIdx>=0) drawSegPath(anim.segPaths[anim.prevSegIdx], anim.prevSegIdx);
  
  for(let n of anim.fullPath){
    if(n===sn||targets.has(n)) continue;
    const el=document.getElementById("svg-node-"+n);
    el.classList.remove("node-explored","node-frontier","node-current");
    el.classList.add("node-final-path");
  }
  
  document.getElementById("autoStepsVal").innerHTML=anim.resultSteps+' <span class="text-xs font-medium text-on-surface-variant italic">bước</span>';
  document.getElementById("autoVisitedVal").textContent=anim.resultExplored;
  const resEl=document.getElementById("pathResult");
  
  // Highlight các đơn bị kẹt vật cản
  let msg=`✅ ${anim.algoName}: ${anim.fullPath.join(" → ")}  |  ${anim.resultSteps} bước  |  ${anim.resultExplored} node duyệt`;
  let skipped = Array.from(targets).filter(t => !anim.fullPath.includes(t) && t !== sn);
  
  if(skipped.length > 0) {
    msg += `\n⚠️ BỎ QUA: Node ${skipped.join(", ")} (Vướng vật cản)`;
    resEl.className="p-4 text-sm text-center warn-msg font-bold";
    skipped.forEach(t => document.getElementById("svg-node-"+t).classList.add("node-skipped"));
    showToast(`✅ Hoàn tất! Bỏ qua ${skipped.length} đơn bị kẹt`,"warning");
  } else {
    resEl.className="p-4 text-sm text-center has-path font-bold";
    showToast(`✅ Hoàn thành! ${anim.resultSteps} bước — ${anim.resultExplored} node duyệt`,"success");
  }
  
  resEl.innerText=msg;
  finalCalculatedPath=anim.fullPath;
  
  if(anim.fullPath.length>=2){
    const lastDir=getDirBetweenNodes(anim.fullPath[anim.fullPath.length-2],anim.fullPath[anim.fullPath.length-1]);
    updateRobotIndicator(anim.fullPath[anim.fullPath.length-1], lastDir);
  }
}

function updateAnimUI(){
  document.getElementById("animStepNum").textContent=anim.currentStep;
  document.getElementById("animTotalSteps").textContent=anim.steps.length;
  if(anim.currentStep>0&&anim.currentStep<=anim.steps.length){
    const s=anim.steps[anim.currentStep-1];
    document.getElementById("animExploredCount").textContent=s.explored.size;
    document.getElementById("animFrontierCount").textContent=s.frontier.size;
  } else { document.getElementById("animExploredCount").textContent='0'; document.getElementById("animFrontierCount").textContent='0'; }
}

// ==================== Path Calculation ====================
function calculateOverallPath(){
  if(anim.active) return; // Khóa UI không tính lại nếu đang chạy animation
  const sn=parseInt(document.getElementById("startNode").value);
  const deliveryMode=document.getElementById("deliveryModeSel").value;
  for(let i=0;i<20;i++){
    let cls="node-group";
    if(i===sn) cls+=" node-start"; else if(obstacles.has(i)) cls+=" node-obstacle";
    document.getElementById("svg-node-"+i).setAttribute("class",cls);
  }
  const pathLayer=document.getElementById("pathLayer"); pathLayer.innerHTML="";
  const resEl=document.getElementById("pathResult");
  const legendBox=document.getElementById("deliveryLegend");
  const orderInfoEl=document.getElementById("deliveryOrderInfo");
  const routeInfoEl=document.getElementById("deliveryRouteInfo");

  if(!targets.size){
    resEl.innerText="Chọn điểm giao hàng trên bản đồ"; resEl.className="p-4 text-sm text-on-surface-variant text-center";
    legendBox.classList.add("hidden"); finalCalculatedPath=[];
    document.getElementById("autoStepsVal").innerHTML='0 <span class="text-xs font-medium text-on-surface-variant italic">bước</span>';
    document.getElementById("autoVisitedVal").textContent='0';
    orderInfoEl.textContent='Chưa có đơn hàng'; routeInfoEl.textContent='—'; return;
  }

  let fullPath=[sn],curS=sn,rem=new Set(targets),totalSteps=0,totalVis=0;
  let ordTar=[],segs=[];

  if(deliveryMode==="express"&&targets.size>1){
    const tarArr=Array.from(targets), expressTarget=tarArr[tarArr.length-1];
    const er=findPath(sn,expressTarget); if(er){ ordTar.push(expressTarget); segs.push(er.path); fullPath=er.path; totalSteps+=er.path.length-1; totalVis+=er.visitedCount; curS=expressTarget; rem.delete(expressTarget); }
    while(rem.size){ let best=null,bestT=null,sl=Infinity; for(let t of rem){ let r=findPath(curS,t); if(r&&r.path.length<sl){sl=r.path.length;best=r;bestT=t;} } if(!best) break; ordTar.push(bestT); segs.push(best.path); fullPath=fullPath.concat(best.path.slice(1)); totalSteps+=best.path.length-1; totalVis+=best.visitedCount; curS=bestT; rem.delete(bestT); }
  } else {
    while(rem.size){ 
      let best=null,bestT=null,sl=Infinity; 
      for(let t of rem){ let r=findPath(curS,t); if(r&&r.path.length<sl){sl=r.path.length;best=r;bestT=t;} } 
      if(!best) break; // Chỉ break ra ngoài nếu kẹt, vẫn lưu lộ trình những đơn đã đi được
      ordTar.push(bestT); segs.push(best.path); fullPath=fullPath.length>1?fullPath.concat(best.path.slice(1)):best.path; totalSteps+=best.path.length-1; totalVis+=best.visitedCount; curS=bestT; rem.delete(bestT); 
    }
  }

  if(ordTar.length){
    finalCalculatedPath=fullPath;
    let msg="LỘ TRÌNH: "+fullPath.join(" ➔ ");
    resEl.className="p-4 text-sm text-center has-path font-bold";
    if(rem.size){ 
      msg+="\n⚠️ BỎ QUA: "+Array.from(rem).join(", ")+" (Vướng vật cản)"; 
      resEl.className="p-4 text-sm text-center warn-msg font-bold"; 
      Array.from(rem).forEach(t=>document.getElementById("svg-node-"+t).classList.add("node-skipped")); 
    }
    resEl.innerText=msg;
    orderInfoEl.textContent=ordTar.map((n,i)=>`Đơn ${i+1} → Node ${n}`+(deliveryMode==='express'&&i===0?' 🚀':'')).join('  |  ');
    routeInfoEl.textContent=fullPath.join(' → ');
    document.getElementById("autoStepsVal").innerHTML=totalSteps+' <span class="text-xs font-medium text-on-surface-variant italic">bước</span>';
    document.getElementById("autoVisitedVal").textContent=totalVis;
    document.getElementById("autoOrderNum").textContent='#01/'+(ordTar.length<10?'0'+ordTar.length:ordTar.length);

    let legHTML='<div class="w-full text-center font-bold text-xs text-on-surface-variant mb-1">THỨ TỰ GIAO HÀNG';
    if(deliveryMode==='express') legHTML+=' <span class="text-red-500">(CHẾ ĐỘ HỎA TỐC)</span>';
    legHTML+='</div>';
    ordTar.forEach((node,idx)=>{
      let ci=Math.min(idx,5),col=TARGET_COLORS[ci];
      document.getElementById("svg-node-"+node).classList.add("target-"+(ci+1));
      let pts=segs[idx].map(n=>coords[n].join(",")).join(" ");
      let poly=document.createElementNS("http://www.w3.org/2000/svg","polyline");
      poly.setAttribute("class","segment-path"); poly.setAttribute("points",pts);
      poly.setAttribute("stroke",col); poly.setAttribute("style",`filter:drop-shadow(0 0 4px ${col}40)`);
      pathLayer.appendChild(poly);
      let eTag=(deliveryMode==='express'&&idx===0)?' 🚀':'';
      legHTML+=`<span class="flex items-center gap-1 px-2 py-1 bg-surface-container rounded-lg"><span class="w-2 h-2 rounded-full" style="background:${col}"></span>Đơn ${idx+1} (N${node})${eTag}</span>`;
      if(idx<ordTar.length-1) legHTML+='<span class="text-outline-variant">➔</span>';
    });
    legendBox.innerHTML=legHTML; legendBox.classList.remove("hidden");
    for(let i=1;i<fullPath.length-1;i++){ if(!targets.has(fullPath[i])) document.getElementById("svg-node-"+fullPath[i]).classList.add("node-path"); }
  } else {
    resEl.innerText="LỖI: Không tìm được đường! Vật cản chặn toàn bộ lộ trình."; resEl.className="p-4 text-sm text-center err-msg font-bold";
    legendBox.classList.add("hidden"); finalCalculatedPath=[];
    orderInfoEl.textContent='Không tìm được đường!'; routeInfoEl.textContent='—';
    targets.forEach(t=>document.getElementById("svg-node-"+t).classList.add("target-1"));
  }
}

// ==================== Path to Commands ====================
function pathToCommands(path,initialDir){
  let dir=parseInt(initialDir||1);
  const cmds=[];
  const dd=[{dx:0,dy:-1},{dx:1,dy:0},{dx:0,dy:1},{dx:-1,dy:0}];
  for(let i=0;i<path.length-1;i++){
    const dx=Math.sign(coords[path[i+1]][0]-coords[path[i]][0]);
    const dy=Math.sign(coords[path[i+1]][1]-coords[path[i]][1]);
    let td=-1;
    for(let d=0;d<4;d++) if(dd[d].dx===dx&&dd[d].dy===dy){td=d;break;}
    if(td===-1){cmds.push("F");continue;}
    const diff=(td-dir+4)%4;
    if(diff===1) cmds.push("R"); else if(diff===3) cmds.push("L"); else if(diff===2){cmds.push("R");cmds.push("R");} else cmds.push("F");
    dir=td;
  }
  return cmds;
}

// ==================== Deliver Button ====================
document.getElementById("deliverBtn").addEventListener("click",async()=>{
  if(!finalCalculatedPath || finalCalculatedPath.length<2) return showToast("Lộ trình không hợp lệ! Vui lòng chọn điểm giao.","error");
  const cmds=pathToCommands(finalCalculatedPath,robotInitialDir);
  const algo=document.getElementById("algoSel").value;
  const btn=document.getElementById("deliverBtn");
  btn.innerHTML='⏳ ĐANG TRUYỀN...';
  deliveryStartTime=Date.now(); deliveryExpectedTime=finalCalculatedPath.length*2; deliveryInProgress=true;
  const banner=document.getElementById('deliveryStatusBanner');
  const icon=document.getElementById('deliveryStatusIcon');
  const text=document.getElementById('deliveryStatusText');
  banner.classList.remove('hidden');
  banner.className='flex items-center gap-2 rounded-xl p-3 bg-amber-50 border border-amber-200';
  icon.textContent='🚚'; icon.className='text-lg text-amber-600 animate-pulse';
  text.textContent=`🚚 Đang giao hàng... (Dự kiến: ~${deliveryExpectedTime}s)`; text.className='text-sm font-bold text-amber-700';
  if(ws&&ws.readyState===WebSocket.OPEN){
    wsSend({type:"ROUTE",commands:cmds,total_steps:cmds.length,algorithm:algo,path:finalCalculatedPath,startNode:parseInt(document.getElementById("startNode").value),initialDir:JS_TO_CPP_DIR[robotInitialDir]});
  } else { try{await fetch(`/deliver?dir=1&path=${finalCalculatedPath.join(",")}`);}catch(e){} }
  setTimeout(()=>{
    btn.innerHTML='✅ ĐÃ NẠP!';
    showToast("Lộ trình đã được gửi đến robot","success");
    setTimeout(()=>{ btn.innerHTML='▶ NẠP LỘ TRÌNH & BẮT ĐẦU'; },2000);
  },1000);
});

// ==================== Algorithm Comparison ====================
document.getElementById("compareAllBtn").addEventListener("click",()=>{
  const sn=parseInt(document.getElementById("startNode").value);
  const tarArr=Array.from(targets);
  if(!tarArr.length){showToast("Chọn điểm giao!","error");return;}
  const algos=["bfs","dfs","ucs","astar","greedy"];
  const names={bfs:"BFS",dfs:"DFS",ucs:"UCS",astar:"A*",greedy:"Greedy"};
  const results=[];
  for(let algo of algos){
    const t0=performance.now();
    let tv=0,tc=0,pl=0,ok=true,cs=sn;
    for(let t of tarArr){const r=findPath(cs,t,algo);if(r){tv+=r.visitedCount;tc+=(r.cost||r.path.length-1);pl+=r.path.length-1;cs=t;}else{ok=false;break;}}
    results.push({algo,name:names[algo],visited:tv,pathLen:pl,cost:tc,time:parseFloat((performance.now()-t0).toFixed(3)),success:ok});
  }
  results.sort((a,b)=>{if(a.success&&!b.success)return-1;if(!a.success&&b.success)return 1;if(a.pathLen!==b.pathLen)return a.pathLen-b.pathLen;if(a.visited!==b.visited)return a.visited-b.visited;return a.time-b.time;});
  const medals=['🥇','🥈','🥉'];
  results.forEach((r,i)=>{r.rank=i+1;r.medal=i<3?medals[i]:'';});
  const labels=results.map(r=>`${r.medal} #${r.rank} ${r.name}`);
  const ctx=document.getElementById('algoComparisonChart').getContext('2d');
  if(algoChartInstance) algoChartInstance.destroy();
  algoChartInstance=new Chart(ctx,{
    type:'bar', data:{ labels, datasets:[
      {label:'Số bước',data:results.map(r=>r.success?r.pathLen:0),backgroundColor:'#005ab4',borderRadius:6,barPercentage:0.7},
      {label:'Node duyệt',data:results.map(r=>r.success?r.visited:0),backgroundColor:'#aac7ff',borderRadius:6,barPercentage:0.7},
      {label:'ms',data:results.map(r=>r.success?r.time:0),backgroundColor:'#bd5700',borderRadius:6,barPercentage:0.7}
    ]},
    options:{responsive:true,maintainAspectRatio:false,animation:{duration:600,easing:'easeOutQuart'},
      scales:{y:{beginAtZero:true,grid:{color:'#e1e3e4'},ticks:{color:'#414753',font:{size:11}}},x:{grid:{display:false},ticks:{color:'#414753',font:{size:12,weight:'bold'}}}},
      plugins:{legend:{position:'top',labels:{color:'#191c1d',font:{size:12,family:'Inter',weight:'600'},boxWidth:14,padding:16}},tooltip:{backgroundColor:'#191c1d',cornerRadius:10,padding:12}}}
  });
  let html=`<table class="w-full text-left text-xs border-separate border-spacing-y-1 mt-2"><thead><tr class="text-[10px] uppercase font-bold text-on-surface-variant tracking-widest"><th class="px-2 py-1">Hạng</th><th class="px-2 py-1">Thuật toán</th><th class="px-2 py-1 text-center">Node</th><th class="px-2 py-1 text-center">Bước</th><th class="px-2 py-1 text-center">ms</th><th class="px-2 py-1 text-center">KQ</th></tr></thead><tbody>`;
  for(let r of results){
    const bg=r.rank===1?'bg-green-50':r.rank===2?'bg-blue-50':r.rank===3?'bg-amber-50':'bg-surface-container-low/40';
    const badge=r.success?'<span class="px-2 py-0.5 bg-green-100 text-green-700 rounded-full text-[10px] font-bold">✓</span>':'<span class="px-2 py-0.5 bg-red-100 text-red-700 rounded-full text-[10px] font-bold">✗</span>';
    html+=`<tr class="${bg} rounded"><td class="px-2 py-2 font-black text-center">${r.medal||r.rank}</td><td class="px-2 py-2 font-bold text-primary">${r.name}</td><td class="text-center px-2">${r.success?r.visited:'—'}</td><td class="text-center px-2">${r.success?r.pathLen:'—'}</td><td class="text-center px-2">${r.time.toFixed(3)}</td><td class="text-center px-2">${badge}</td></tr>`;
  }
  html+='</tbody></table>';
  document.getElementById("algoCompareResult").innerHTML=html;
  showToast("So sánh hoàn tất!","success");
});

// ==================== LỆNH VỀ KHO TỰ ĐỘNG ====================
function returnHome() {
  const warehouseNode = 0; 
  let currentRobotNode = parseInt(document.getElementById("startNode").value);
  
  if (finalCalculatedPath && finalCalculatedPath.length > 0) {
    currentRobotNode = finalCalculatedPath[finalCalculatedPath.length - 1];
    document.getElementById("startNode").value = currentRobotNode; 
  }
  
  if (currentRobotNode === warehouseNode) {
    showToast("Xe hiện đang ở kho rồi!", "info");
    return;
  }

  targets.clear(); 
  targets.add(warehouseNode);
  updateNodeVisuals(); 
  calculateOverallPath(); // Tính đường trước khi gửi
  
  document.getElementById("deliverBtn").click();
  showToast("Tự động quay về kho...", "success");
}

// ==================== Event Listeners ====================
document.querySelectorAll("#startNode,#algoSel,#orderModeSel,#deliveryModeSel").forEach(el=>
  el.addEventListener("change",()=>{
    if(el.id==="orderModeSel"&&el.value==="single"&&targets.size>1){const f=Array.from(targets)[0];targets.clear();targets.add(f);}
    obstacles.delete(parseInt(document.getElementById("startNode").value));
    updateNodeVisuals(); 
    calculateOverallPath(); // ĐÃ SỬA: Đảm bảo luôn tính toán lại đường đi khi thay đổi config
  })
);
document.getElementById("animSpeedSlider").addEventListener("input",(e)=>{
  const v=parseInt(e.target.value); const t=1-v/100; anim.speed=Math.round(1500*t*t+30);
});

// ==================== Manual D-Pad ====================
async function sendCommand(cmd){try{await fetch(cmd);showToast("Lệnh: "+cmd,"info");}catch(e){}}
async function send(path){try{await fetch(path);}catch(e){}}
let activeHold={btn:null,pid:null};
function guard(fn){return(e)=>{if(currentMode!=="manual"){e.preventDefault();return;}return fn(e);};}
document.querySelectorAll(".hold").forEach(btn=>{
  btn.addEventListener("pointerdown",guard(e=>{e.preventDefault();activeHold={btn,pid:e.pointerId};btn.classList.add("active-hold");btn.setPointerCapture(e.pointerId);send(btn.dataset.path);}),{passive:false});
  const release=guard(e=>{e.preventDefault();if(activeHold.btn===btn&&activeHold.pid===e.pointerId){btn.classList.remove("active-hold");send("/stop");activeHold={btn:null,pid:null};}try{btn.releasePointerCapture(e.pointerId);}catch(_){}});
  btn.addEventListener("pointerup",release,{passive:false});
  btn.addEventListener("pointercancel",release,{passive:false});
});
document.getElementById("stopBtn").addEventListener("pointerdown",guard(e=>{e.preventDefault();send("/stop");}),{passive:false});

// ==================== Init ====================
initNodes();
renderEdges();
createRobotIndicator();
updateNodeVisuals();
calculateOverallPath(); // Vẽ sẵn lộ trình rỗng ban đầu
setTimeout(()=>connectWebSocket(),1000);