/**
 * AI Search Algorithms for Grid-based Pathfinding
 * Implements: BFS, DFS, UCS, A*, Greedy Best-First Search
 * 
 * Grid values: 0 = empty, -1 = wall, >0 = weight/cost
 * Returns: { path, exploredHistory, frontierHistory, stats }
 */

// ==================== Priority Queue (Min-Heap) ====================
class PriorityQueue {
  constructor() {
    this.heap = [];
  }
  
  push(item, priority) {
    this.heap.push({ item, priority });
    this._bubbleUp(this.heap.length - 1);
  }
  
  pop() {
    if (this.heap.length === 0) return null;
    const top = this.heap[0];
    const last = this.heap.pop();
    if (this.heap.length > 0) {
      this.heap[0] = last;
      this._sinkDown(0);
    }
    return top.item;
  }
  
  isEmpty() {
    return this.heap.length === 0;
  }
  
  size() {
    return this.heap.length;
  }
  
  _bubbleUp(i) {
    while (i > 0) {
      const parent = Math.floor((i - 1) / 2);
      if (this.heap[parent].priority <= this.heap[i].priority) break;
      [this.heap[parent], this.heap[i]] = [this.heap[i], this.heap[parent]];
      i = parent;
    }
  }
  
  _sinkDown(i) {
    const n = this.heap.length;
    while (true) {
      let smallest = i;
      const left = 2 * i + 1;
      const right = 2 * i + 2;
      if (left < n && this.heap[left].priority < this.heap[smallest].priority) smallest = left;
      if (right < n && this.heap[right].priority < this.heap[smallest].priority) smallest = right;
      if (smallest === i) break;
      [this.heap[smallest], this.heap[i]] = [this.heap[i], this.heap[smallest]];
      i = smallest;
    }
  }
  
  toArray() {
    return this.heap.map(h => h.item);
  }
}

// ==================== Heuristic Functions ====================
function manhattanDistance(a, b) {
  return Math.abs(a.row - b.row) + Math.abs(a.col - b.col);
}

function euclideanDistance(a, b) {
  return Math.sqrt((a.row - b.row) ** 2 + (a.col - b.col) ** 2);
}

// ==================== Utility ====================
function getNeighbors(grid, node) {
  const rows = grid.length;
  const cols = grid[0].length;
  const dirs = [[-1, 0], [1, 0], [0, -1], [0, 1]]; // up, down, left, right
  const neighbors = [];
  
  for (const [dr, dc] of dirs) {
    const r = node.row + dr;
    const c = node.col + dc;
    if (r >= 0 && r < rows && c >= 0 && c < cols && grid[r][c] !== -1) {
      neighbors.push({ row: r, col: c });
    }
  }
  return neighbors;
}

function getCellCost(grid, node) {
  const val = grid[node.row][node.col];
  return val <= 0 ? 1 : val; // default cost 1 for empty cells
}

function key(node) {
  return `${node.row},${node.col}`;
}

function reconstructPath(cameFrom, end) {
  const path = [];
  let current = key(end);
  while (current !== null) {
    const [r, c] = current.split(',').map(Number);
    path.unshift({ row: r, col: c });
    current = cameFrom[current] || null;
  }
  return path;
}

// ==================== BFS (Breadth-First Search) ====================
function bfs(grid, start, end) {
  const t0 = performance.now();
  const queue = [start];
  const visited = new Set();
  const cameFrom = {};
  visited.add(key(start));
  cameFrom[key(start)] = null;
  
  const exploredHistory = []; // each step: array of explored nodes
  const frontierHistory = []; // each step: array of frontier nodes
  let nodesExplored = 0;
  
  while (queue.length > 0) {
    const current = queue.shift();
    nodesExplored++;
    
    exploredHistory.push({ node: { ...current }, explored: [...visited] });
    frontierHistory.push([...queue.map(n => ({ ...n }))]);
    
    if (current.row === end.row && current.col === end.col) {
      const path = reconstructPath(cameFrom, end);
      const elapsed = performance.now() - t0;
      return {
        path,
        exploredHistory,
        frontierHistory,
        stats: {
          algorithm: 'BFS',
          nodesExplored,
          pathLength: path.length,
          pathCost: path.length - 1,
          timeMs: elapsed.toFixed(2),
          optimal: true
        }
      };
    }
    
    for (const neighbor of getNeighbors(grid, current)) {
      const k = key(neighbor);
      if (!visited.has(k)) {
        visited.add(k);
        cameFrom[k] = key(current);
        queue.push(neighbor);
      }
    }
  }
  
  return { path: null, exploredHistory, frontierHistory, stats: { algorithm: 'BFS', nodesExplored, pathLength: 0, pathCost: Infinity, timeMs: (performance.now() - t0).toFixed(2), optimal: false } };
}

// ==================== DFS (Depth-First Search) ====================
function dfs(grid, start, end) {
  const t0 = performance.now();
  const stack = [start];
  const visited = new Set();
  const cameFrom = {};
  cameFrom[key(start)] = null;
  
  const exploredHistory = [];
  const frontierHistory = [];
  let nodesExplored = 0;
  
  while (stack.length > 0) {
    const current = stack.pop();
    const k = key(current);
    
    if (visited.has(k)) continue;
    visited.add(k);
    nodesExplored++;
    
    exploredHistory.push({ node: { ...current }, explored: [...visited] });
    frontierHistory.push([...stack.map(n => ({ ...n }))]);
    
    if (current.row === end.row && current.col === end.col) {
      const path = reconstructPath(cameFrom, end);
      const elapsed = performance.now() - t0;
      return {
        path,
        exploredHistory,
        frontierHistory,
        stats: {
          algorithm: 'DFS',
          nodesExplored,
          pathLength: path.length,
          pathCost: path.length - 1,
          timeMs: elapsed.toFixed(2),
          optimal: false
        }
      };
    }
    
    const neighbors = getNeighbors(grid, current);
    for (let i = neighbors.length - 1; i >= 0; i--) {
      const neighbor = neighbors[i];
      const nk = key(neighbor);
      if (!visited.has(nk)) {
        cameFrom[nk] = k;
        stack.push(neighbor);
      }
    }
  }
  
  return { path: null, exploredHistory, frontierHistory, stats: { algorithm: 'DFS', nodesExplored, pathLength: 0, pathCost: Infinity, timeMs: (performance.now() - t0).toFixed(2), optimal: false } };
}

// ==================== UCS (Uniform-Cost Search) ====================
function ucs(grid, start, end) {
  const t0 = performance.now();
  const pq = new PriorityQueue();
  pq.push({ ...start, cost: 0 }, 0);
  const visited = new Set();
  const cameFrom = {};
  const costSoFar = {};
  cameFrom[key(start)] = null;
  costSoFar[key(start)] = 0;
  
  const exploredHistory = [];
  const frontierHistory = [];
  let nodesExplored = 0;
  
  while (!pq.isEmpty()) {
    const current = pq.pop();
    const k = key(current);
    
    if (visited.has(k)) continue;
    visited.add(k);
    nodesExplored++;
    
    exploredHistory.push({ node: { row: current.row, col: current.col }, explored: [...visited] });
    frontierHistory.push(pq.toArray().map(n => ({ row: n.row, col: n.col })));
    
    if (current.row === end.row && current.col === end.col) {
      const path = reconstructPath(cameFrom, end);
      const elapsed = performance.now() - t0;
      return {
        path,
        exploredHistory,
        frontierHistory,
        stats: {
          algorithm: 'UCS',
          nodesExplored,
          pathLength: path.length,
          pathCost: costSoFar[k],
          timeMs: elapsed.toFixed(2),
          optimal: true
        }
      };
    }
    
    for (const neighbor of getNeighbors(grid, current)) {
      const nk = key(neighbor);
      const newCost = costSoFar[k] + getCellCost(grid, neighbor);
      if (!visited.has(nk) && (!(nk in costSoFar) || newCost < costSoFar[nk])) {
        costSoFar[nk] = newCost;
        cameFrom[nk] = k;
        pq.push({ ...neighbor, cost: newCost }, newCost);
      }
    }
  }
  
  return { path: null, exploredHistory, frontierHistory, stats: { algorithm: 'UCS', nodesExplored, pathLength: 0, pathCost: Infinity, timeMs: (performance.now() - t0).toFixed(2), optimal: false } };
}

// ==================== A* Search ====================
function aStar(grid, start, end, heuristicFn = manhattanDistance) {
  const t0 = performance.now();
  const pq = new PriorityQueue();
  const startKey = key(start);
  pq.push({ ...start, cost: 0 }, 0);
  const visited = new Set();
  const cameFrom = {};
  const gScore = {};
  cameFrom[startKey] = null;
  gScore[startKey] = 0;
  
  const exploredHistory = [];
  const frontierHistory = [];
  let nodesExplored = 0;
  
  while (!pq.isEmpty()) {
    const current = pq.pop();
    const k = key(current);
    
    if (visited.has(k)) continue;
    visited.add(k);
    nodesExplored++;
    
    exploredHistory.push({ node: { row: current.row, col: current.col }, explored: [...visited] });
    frontierHistory.push(pq.toArray().map(n => ({ row: n.row, col: n.col })));
    
    if (current.row === end.row && current.col === end.col) {
      const path = reconstructPath(cameFrom, end);
      const elapsed = performance.now() - t0;
      return {
        path,
        exploredHistory,
        frontierHistory,
        stats: {
          algorithm: 'A*',
          nodesExplored,
          pathLength: path.length,
          pathCost: gScore[k],
          timeMs: elapsed.toFixed(2),
          optimal: true
        }
      };
    }
    
    for (const neighbor of getNeighbors(grid, current)) {
      const nk = key(neighbor);
      const tentativeG = gScore[k] + getCellCost(grid, neighbor);
      if (!visited.has(nk) && (!(nk in gScore) || tentativeG < gScore[nk])) {
        gScore[nk] = tentativeG;
        const f = tentativeG + heuristicFn(neighbor, end);
        cameFrom[nk] = k;
        pq.push({ ...neighbor, cost: tentativeG }, f);
      }
    }
  }
  
  return { path: null, exploredHistory, frontierHistory, stats: { algorithm: 'A*', nodesExplored, pathLength: 0, pathCost: Infinity, timeMs: (performance.now() - t0).toFixed(2), optimal: false } };
}

// ==================== Greedy Best-First Search ====================
function greedy(grid, start, end, heuristicFn = manhattanDistance) {
  const t0 = performance.now();
  const pq = new PriorityQueue();
  pq.push(start, heuristicFn(start, end));
  const visited = new Set();
  const cameFrom = {};
  cameFrom[key(start)] = null;
  
  const exploredHistory = [];
  const frontierHistory = [];
  let nodesExplored = 0;
  
  while (!pq.isEmpty()) {
    const current = pq.pop();
    const k = key(current);
    
    if (visited.has(k)) continue;
    visited.add(k);
    nodesExplored++;
    
    exploredHistory.push({ node: { ...current }, explored: [...visited] });
    frontierHistory.push(pq.toArray().map(n => ({ ...n })));
    
    if (current.row === end.row && current.col === end.col) {
      const path = reconstructPath(cameFrom, end);
      const elapsed = performance.now() - t0;
      // Calculate actual path cost
      let pathCost = 0;
      for (let i = 1; i < path.length; i++) {
        pathCost += getCellCost(grid, path[i]);
      }
      return {
        path,
        exploredHistory,
        frontierHistory,
        stats: {
          algorithm: 'Greedy',
          nodesExplored,
          pathLength: path.length,
          pathCost: pathCost,
          timeMs: elapsed.toFixed(2),
          optimal: false
        }
      };
    }
    
    for (const neighbor of getNeighbors(grid, current)) {
      const nk = key(neighbor);
      if (!visited.has(nk) && !(nk in cameFrom)) {
        cameFrom[nk] = k;
        pq.push(neighbor, heuristicFn(neighbor, end));
      }
    }
  }
  
  return { path: null, exploredHistory, frontierHistory, stats: { algorithm: 'Greedy', nodesExplored, pathLength: 0, pathCost: Infinity, timeMs: (performance.now() - t0).toFixed(2), optimal: false } };
}

// ==================== Path → Commands Conversion ====================
/**
 * Convert grid path to robot commands (F/L/R)
 * Assumes robot starts facing DOWN (+row direction)
 * @param {Array} path - Array of {row, col} nodes
 * @returns {Array} commands - Array of "F", "L", "R"
 */
function pathToCommands(path) {
  if (!path || path.length < 2) return [];
  
  // Direction vectors: [dRow, dCol]
  const DIR_DOWN  = 0; // +row
  const DIR_UP    = 1; // -row
  const DIR_RIGHT = 2; // +col
  const DIR_LEFT  = 3; // -col
  
  const dirVectors = {
    [DIR_DOWN]:  [1, 0],
    [DIR_UP]:    [-1, 0],
    [DIR_RIGHT]: [0, 1],
    [DIR_LEFT]:  [0, -1]
  };
  
  function getDirection(from, to) {
    const dr = to.row - from.row;
    const dc = to.col - from.col;
    if (dr === 1 && dc === 0) return DIR_DOWN;
    if (dr === -1 && dc === 0) return DIR_UP;
    if (dr === 0 && dc === 1) return DIR_RIGHT;
    if (dr === 0 && dc === -1) return DIR_LEFT;
    return -1;
  }
  
  // turnNeeded: given current facing direction and desired direction, return command
  function turnCommand(currentDir, nextDir) {
    if (currentDir === nextDir) return 'F';
    
    // Right turns
    const rightTurn = {
      [DIR_DOWN]: DIR_LEFT,
      [DIR_LEFT]: DIR_UP,
      [DIR_UP]: DIR_RIGHT,
      [DIR_RIGHT]: DIR_DOWN
    };
    
    // Left turns
    const leftTurn = {
      [DIR_DOWN]: DIR_RIGHT,
      [DIR_RIGHT]: DIR_UP,
      [DIR_UP]: DIR_LEFT,
      [DIR_LEFT]: DIR_DOWN
    };
    
    if (rightTurn[currentDir] === nextDir) return 'R';
    if (leftTurn[currentDir] === nextDir) return 'L';
    return 'U'; // U-turn (shouldn't happen in grid pathfinding normally)
  }
  
  const commands = [];
  let currentDir = getDirection(path[0], path[1]);
  commands.push('F'); // First move: go forward in initial direction
  
  for (let i = 1; i < path.length - 1; i++) {
    const nextDir = getDirection(path[i], path[i + 1]);
    const cmd = turnCommand(currentDir, nextDir);
    commands.push(cmd);
    currentDir = nextDir;
  }
  
  return commands;
}

// ==================== Run All Algorithms ====================
function runAllAlgorithms(grid, start, end, heuristicFn = manhattanDistance) {
  return [
    bfs(grid, start, end),
    dfs(grid, start, end),
    ucs(grid, start, end),
    aStar(grid, start, end, heuristicFn),
    greedy(grid, start, end, heuristicFn)
  ];
}
