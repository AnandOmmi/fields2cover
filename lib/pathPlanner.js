const turf = require('@turf/turf');
const { dubinsCandidates, sampleDubins } = require('./mowerLoopFallback');
const { areaFromGeoJSON, freeSpaceReflexVertices } = require('./geometry');
const { bufferMeters } = require('./headland');

// Port of Fields2Cover's pp::PathPlanning plus the v2 route planner's
// headland transit. Every connection is a minimum-radius Dubins path that
// must stay inside the drive area (boundary and obstacle clearance already
// applied), and never loops. When no single Dubins word fits, the vehicle
// either swings out wide and comes back (two smooth turns), or transits
// through the free space: shortest path over a visibility graph of
// free-space corners, then a chain of Dubins hops along it.

const CHECK_STEP_M = 0.25;
const OUTPUT_STEP_M = 0.1; // same point density the vehicle already gets on turns
const NODE_SIMPLIFY_TOLERANCE_DEG = 0.000004; // ~0.4m, keeps the graph small
const TWO_HOP_RADII = [1.5, 2.5, 4, 5]; // intermediate poses this many turning radii out
const TWO_HOP_BEARINGS = 12;
const TWO_HOP_HEADINGS = 8;
const TWO_HOP_CHECKS = 60;
const ESCAPE_RADII = [4, 5]; // far enough that reaching it means actually leaving a pocket
const ESCAPE_CHECKS = 40;

const pose = (x, y, th) => ({ x, y, th });

// No loops: a single manoeuvre may turn at most a U-turn plus a little
// (enough for a U-turn between rows whose ends are not level). Anything more
// is a circle/loop, which the operator does not want.
const MAX_TURN_RAD = Math.PI + 0.35;
const L_SEG = 0;
const S_SEG = 1;
const turnOf = (word) => word.params.reduce((sum, value, i) => (word.types[i] === S_SEG ? sum : sum + value), 0);

// No semicircles either: a U-turn must be quarter arc, straight, quarter
// arc. A single arc this long only appears in a bare semicircle U-turn
// (rows exactly one turn diameter apart) or a near-loop.
const MAX_SINGLE_ARC_RAD = (150 * Math.PI) / 180;
// Two same-way arcs read as one arc (a semicircle) unless a straight of at
// least this × R separates them — the same flat U-turn the row order plans.
const UTURN_STRAIGHT_FACTOR = 0.5;
// Longest same-way arc across one or more consecutive words (hops of one
// connection chain their arcs just like segments of one word do).
function longestArc(words) {
  let longest = 0;
  let run = 0;
  let runType = null;
  [].concat(words).forEach((word) => word.types.forEach((type, i) => {
    const amount = word.params[i];
    if (amount < 1e-9) return;
    if (type === S_SEG) {
      if (amount >= UTURN_STRAIGHT_FACTOR - 1e-6) {
        run = 0;
        runType = null;
      }
      return;
    }
    run = type === runType ? run + amount : amount;
    runType = type;
    longest = Math.max(longest, run);
  }));
  return longest;
}

// Number of U-turn sized turns in a chain: turning (either way) between
// straights of at least the flat-U-turn straight, totalling over 120°.
function uTurnCount(words) {
  let count = 0;
  let group = 0;
  const close = () => {
    if (group > U_TURN_SIZED_RAD) count += 1;
    group = 0;
  };
  [].concat(words).forEach((word) => word.types.forEach((type, i) => {
    const amount = word.params[i];
    if (type === S_SEG) {
      if (amount >= UTURN_STRAIGHT_FACTOR - 1e-6) close();
      return;
    }
    group += amount;
  }));
  close();
  return count;
}

// A chain of hops is acceptable when, taken as one manoeuvre, it neither
// loops, nor bends through a semicircle, nor swings out and back with two
// U-turns.
//
// `wrapping`: a lap going round an obstacle corner, where one long arc is the
// ring itself rather than a U-turn — only the no-loop limit applies.
const chainIsSmooth = (words, { wrapping = false } = {}) => longestTurn(words) <= MAX_TURN_RAD
  && (wrapping || (longestArc(words) <= MAX_SINGLE_ARC_RAD && uTurnCount(words) <= 1));

// Dubins words between two poses, shortest first, without loops or (unless
// `wrapping`, see chainIsSmooth) semicircles.
function smoothWords(from, to, radius, { wrapping = false } = {}) {
  return dubinsCandidates(from, to, radius)
    .filter((word) => turnOf(word) <= MAX_TURN_RAD && (wrapping || longestArc(word) <= MAX_SINGLE_ARC_RAD));
}

// A turn this big counts as a U-turn; two in one connection is a swing out
// and back, which the operator does not want.
const U_TURN_SIZED_RAD = (120 * Math.PI) / 180;

// Longest continuous same-direction turn across consecutive words (a hop
// ending in a left arc followed by one starting with a left arc is one turn).
// Two loop-free U-turns back to back the same way are still a full circle.
function longestTurn(words) {
  let longest = 0;
  let run = 0;
  words.forEach((word) => word.types.forEach((type, i) => {
    const amount = word.params[i];
    if (amount < 1e-9) return;
    if (type === S_SEG) {
      run = 0;
      return;
    }
    const signed = type === L_SEG ? amount : -amount;
    run = Math.sign(run) === Math.sign(signed) ? run + signed : signed;
    longest = Math.max(longest, Math.abs(run));
  }));
  return longest;
}

class PathPlanner {
  constructor({ driveArea, driveFeature, toLocal, radius }) {
    this.driveArea = driveArea;
    this.driveFeature = driveFeature;
    this.toLocal = toLocal;
    this.radius = radius;
    this.graphs = new Map();
    // Larger margins first: waypoints further from obstacles leave the Dubins
    // hops room to turn; smaller ones still get through narrow passages.
    this.margins = [...new Set([radius, radius / 2, 2, 1].map((m) => Math.max(0.5, m)))];
  }

  // Shortest loop-free Dubins word that stays inside the drive area, or null.
  direct(from, to) {
    for (const word of smoothWords(from, to, this.radius)) {
      if (this.driveArea.containsAll(sampleDubins(word, CHECK_STEP_M))) {
        return { points: sampleDubins(word, OUTPUT_STEP_M), length: word.length, words: [word] };
      }
    }
    return null;
  }

  connect(from, to) {
    return this.direct(from, to) || this.transit(from, to);
  }

  transit(from, to) {
    const turnaround = this.twoHop(from, to);
    if (turnaround) return turnaround;
    for (const margin of this.margins) {
      const graph = this.graph(margin);
      if (!graph) continue;
      const waypoints = this.shortestPath(graph, [from.x, from.y], [to.x, to.y]);
      if (!waypoints || waypoints.length === 0) continue;
      const result = this.followWaypoints(from, waypoints, to);
      if (result) return result;
    }
    return null;
  }

  // Target the shortest smooth turn cannot reach (it would leave the drive
  // area, or needs more than one manoeuvre's worth of turning — e.g. onto
  // the perpendicular rows of a criss-cross pass): detour through an
  // intermediate pose. At most one of the two hops may be a U-turn — no
  // swinging out with a U-turn and back with another. Candidates are ranked
  // by unchecked length and only the best few are validated.
  twoHop(from, to) {
    const candidates = [];
    const centres = [[from.x, from.y], [(from.x + to.x) / 2, (from.y + to.y) / 2], [to.x, to.y]];
    centres.forEach(([cx, cy]) => {
      TWO_HOP_RADII.forEach((k) => {
        for (let b = 0; b < TWO_HOP_BEARINGS; b += 1) {
          const angle = (2 * Math.PI * b) / TWO_HOP_BEARINGS;
          const x = cx + k * this.radius * Math.cos(angle);
          const y = cy + k * this.radius * Math.sin(angle);
          if (!this.driveArea.contains(x, y)) continue;
          for (let h = 0; h < TWO_HOP_HEADINGS; h += 1) {
            const mid = pose(x, y, (2 * Math.PI * h) / TWO_HOP_HEADINGS);
            const first = smoothWords(from, mid, this.radius)[0];
            if (!first) continue;
            const second = smoothWords(mid, to, this.radius)[0];
            if (!second || !chainIsSmooth([first, second])) continue;
            candidates.push({ mid, length: first.length + second.length });
          }
        }
      });
    });
    candidates.sort((a, b) => a.length - b.length);
    for (const { mid } of candidates.slice(0, TWO_HOP_CHECKS)) {
      const first = this.direct(from, mid);
      if (!first) continue;
      const second = this.direct(mid, to);
      if (!second || !chainIsSmooth([...first.words, ...second.words])) continue;
      return { points: [...first.points, ...second.points.slice(1)], length: first.length + second.length, words: [...first.words, ...second.words] };
    }
    return null;
  }

  // True when some smooth, valid manoeuvre leads away from `from` — i.e. the
  // vehicle is not stuck facing a wall in a pocket too narrow to turn in.
  // `radii`: how far out (in turning radii) the manoeuvre must get.
  canEscape(from, radii = ESCAPE_RADII) {
    const candidates = [];
    radii.forEach((k) => {
      for (let b = 0; b < TWO_HOP_BEARINGS; b += 1) {
        const angle = (2 * Math.PI * b) / TWO_HOP_BEARINGS;
        const x = from.x + k * this.radius * Math.cos(angle);
        const y = from.y + k * this.radius * Math.sin(angle);
        if (!this.driveArea.contains(x, y)) continue;
        for (let h = 0; h < TWO_HOP_HEADINGS; h += 1) {
          const target = pose(x, y, (2 * Math.PI * h) / TWO_HOP_HEADINGS);
          const word = smoothWords(from, target, this.radius)[0];
          if (word) candidates.push({ target, length: word.length });
        }
      }
    });
    candidates.sort((a, b) => a.length - b.length);
    return candidates.slice(0, ESCAPE_CHECKS).some(({ target }) => this.direct(from, target));
  }

  graph(margin) {
    if (this.graphs.has(margin)) return this.graphs.get(margin);
    let graph = null;
    try {
      const shrunk = bufferMeters(this.driveFeature, -margin);
      if (shrunk) {
        const simplified = turf.simplify(shrunk, { tolerance: NODE_SIMPLIFY_TOLERANCE_DEG });
        const nodes = freeSpaceReflexVertices(areaFromGeoJSON(simplified, this.toLocal))
          .filter(([x, y]) => this.driveArea.contains(x, y));
        graph = { nodes, adjacency: new Map() };
      }
    } catch (err) {
      console.warn(`Transit graph unavailable at ${margin} m margin:`, err.message);
    }
    this.graphs.set(margin, graph);
    return graph;
  }

  neighbours(graph, index) {
    if (!graph.adjacency.has(index)) {
      const a = graph.nodes[index];
      const list = [];
      graph.nodes.forEach((b, j) => {
        if (j !== index && this.driveArea.segmentClear(a, b)) list.push(j);
      });
      graph.adjacency.set(index, list);
    }
    return graph.adjacency.get(index);
  }

  // A* over the visibility graph; returns the intermediate corner points
  // (start and goal excluded), or null when the goal is unreachable.
  shortestPath(graph, start, goal) {
    const { nodes } = graph;
    const START = nodes.length;
    const GOAL = nodes.length + 1;
    const point = (i) => (i === START ? start : i === GOAL ? goal : nodes[i]);
    const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

    // Checked only for the nodes the search actually expands.
    const goalVisible = new Map();
    const seesGoal = (j) => {
      if (!goalVisible.has(j)) goalVisible.set(j, this.driveArea.segmentClear(nodes[j], goal));
      return goalVisible.get(j);
    };
    const edgesOf = (i) => {
      if (i === START) {
        const list = [];
        nodes.forEach((n, j) => {
          if (this.driveArea.segmentClear(start, n)) list.push(j);
        });
        if (this.driveArea.segmentClear(start, goal)) list.push(GOAL);
        return list;
      }
      const list = this.neighbours(graph, i);
      return seesGoal(i) ? [...list, GOAL] : list;
    };

    const g = new Map([[START, 0]]);
    const parent = new Map();
    const open = new Set([START]);
    const closed = new Set();
    while (open.size) {
      let current = null;
      let best = Infinity;
      open.forEach((i) => {
        const f = g.get(i) + dist(point(i), goal);
        if (f < best) {
          best = f;
          current = i;
        }
      });
      if (current === GOAL) {
        const path = [];
        for (let i = parent.get(GOAL); i !== START; i = parent.get(i)) path.unshift(nodes[i]);
        return path;
      }
      open.delete(current);
      closed.add(current);
      for (const j of edgesOf(current)) {
        if (closed.has(j)) continue;
        const cost = g.get(current) + dist(point(current), point(j));
        if (cost < (g.has(j) ? g.get(j) : Infinity)) {
          g.set(j, cost);
          parent.set(j, current);
          open.add(j);
        }
      }
    }
    return null;
  }

  // Dubins hops along the waypoints, each waypoint passed with the heading
  // bisecting its incoming and outgoing legs. Greedily skips ahead to the
  // furthest pose a single valid hop can reach, so the result is as smooth
  // as the free space allows.
  followWaypoints(from, waypoints, to) {
    const chain = [[from.x, from.y], ...waypoints, [to.x, to.y]];
    const targets = waypoints.map((w, i) => {
      const prev = chain[i];
      const next = chain[i + 2];
      const inA = Math.atan2(w[1] - prev[1], w[0] - prev[0]);
      const outA = Math.atan2(next[1] - w[1], next[0] - w[0]);
      const sx = Math.cos(inA) + Math.cos(outA);
      const sy = Math.sin(inA) + Math.sin(outA);
      const th = Math.hypot(sx, sy) < 1e-6 ? inA : Math.atan2(sy, sx);
      return pose(w[0], w[1], th);
    });
    targets.push(to);

    const points = [[from.x, from.y]];
    const words = [];
    let length = 0;
    let current = from;
    let index = 0;
    while (index < targets.length) {
      let hop = null;
      let reached = -1;
      for (let j = targets.length - 1; j >= index; j -= 1) {
        const candidate = this.direct(current, targets[j]);
        if (candidate && chainIsSmooth([...words, ...candidate.words])) {
          hop = candidate;
          reached = j;
          break;
        }
      }
      if (!hop) return null;
      points.push(...hop.points.slice(1));
      words.push(...hop.words);
      length += hop.length;
      current = targets[reached];
      index = reached + 1;
    }
    return { points, length, words };
  }
}

module.exports = { PathPlanner, pose, smoothWords, chainIsSmooth, UTURN_STRAIGHT_FACTOR };
