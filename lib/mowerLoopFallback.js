// Fallback planner for "Join start & end".
//
// mowerLoopPath.js builds the connector with a fixed arc-straight-arc template
// (ported from generate_loop_path.py). That template assumes a free row to the
// right of the end point, so on irregular boundaries or with obstacles it can
// leave the safe area. This planner always searches for a forward-only,
// smooth (Dubins arc/straight) route from the path end to the start pose that
// stays inside the safe area:
//
//   1. short straight lead-out, so the mower does not turn on the spot,
//   2. direct Dubins connection (all six words, shortest valid one),
//   3. one intermediate waypoint pose,
//   4. A* over a coarse pose graph (multiple waypoints),
//
// using exactly the configured turning radius (never a tighter one: that is the
// mower's physical limit). The caller runs one strict validation pass on the
// result.

const { createArea } = require('./geometry');

const METERS_PER_DEGREE_LATITUDE = 110540;
const METERS_PER_DEGREE_LONGITUDE = 111320;
const TWO_PI = 2 * Math.PI;

const CHECK_STEP_M = 0.2;
const OUTPUT_STEP_M = 0.1;
const ENDPOINT_GRACE_M = 0; // no exemptions: every sample must be inside the safe area
const HEADING_COUNT = 8;
const ONE_HOP_GRID = 34;
const GRAPH_POINT_LIMIT = 150;
const MAX_GRAPH_EXPANSIONS = 1200;

const mod2pi = (angle) => ((angle % TWO_PI) + TWO_PI) % TWO_PI;

// ---------------------------------------------------------------------------
// Dubins (six words), after Shkel & Lumelsky / Walker's dubins.c
// ---------------------------------------------------------------------------
const L_SEG = 0;
const S_SEG = 1;
const R_SEG = 2;
const WORDS = {
  LSL: [L_SEG, S_SEG, L_SEG],
  RSR: [R_SEG, S_SEG, R_SEG],
  LSR: [L_SEG, S_SEG, R_SEG],
  RSL: [R_SEG, S_SEG, L_SEG],
  RLR: [R_SEG, L_SEG, R_SEG],
  LRL: [L_SEG, R_SEG, L_SEG]
};

function wordParams(name, alpha, beta, d) {
  const sa = Math.sin(alpha);
  const sb = Math.sin(beta);
  const ca = Math.cos(alpha);
  const cb = Math.cos(beta);
  const cab = Math.cos(alpha - beta);
  switch (name) {
    case "LSL": {
      const p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb);
      if (p2 < 0) return null;
      const tmp = Math.atan2(cb - ca, d + sa - sb);
      return [mod2pi(-alpha + tmp), Math.sqrt(p2), mod2pi(beta - tmp)];
    }
    case "RSR": {
      const p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa);
      if (p2 < 0) return null;
      const tmp = Math.atan2(ca - cb, d - sa + sb);
      return [mod2pi(alpha - tmp), Math.sqrt(p2), mod2pi(-beta + tmp)];
    }
    case "LSR": {
      const p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb);
      if (p2 < 0) return null;
      const p = Math.sqrt(p2);
      const tmp = Math.atan2(-ca - cb, d + sa + sb) - Math.atan2(-2, p);
      return [mod2pi(-alpha + tmp), p, mod2pi(-mod2pi(beta) + tmp)];
    }
    case "RSL": {
      const p2 = d * d - 2 + 2 * cab - 2 * d * (sa + sb);
      if (p2 < 0) return null;
      const p = Math.sqrt(p2);
      const tmp = Math.atan2(ca + cb, d - sa - sb) - Math.atan2(2, p);
      return [mod2pi(alpha - tmp), p, mod2pi(beta - tmp)];
    }
    case "RLR": {
      const tmp = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8;
      if (Math.abs(tmp) > 1) return null;
      const p = mod2pi(TWO_PI - Math.acos(tmp));
      const t = mod2pi(alpha - Math.atan2(ca - cb, d - sa + sb) + mod2pi(p / 2));
      return [t, p, mod2pi(alpha - beta - t + mod2pi(p))];
    }
    case "LRL": {
      const tmp = (6 - d * d + 2 * cab + 2 * d * (sb - sa)) / 8;
      if (Math.abs(tmp) > 1) return null;
      const p = mod2pi(TWO_PI - Math.acos(tmp));
      const t = mod2pi(-alpha - Math.atan2(ca - cb, d + sa - sb) + p / 2);
      return [t, p, mod2pi(mod2pi(beta) - alpha - t + mod2pi(p))];
    }
    default:
      return null;
  }
}

// All feasible Dubins words between two poses ({x, y, th}), shortest first.
function dubinsCandidates(from, to, radius) {
  const dx = to.x - from.x;
  const dy = to.y - from.y;
  const d = Math.hypot(dx, dy) / radius;
  const theta = mod2pi(Math.atan2(dy, dx));
  const alpha = mod2pi(from.th - theta);
  const beta = mod2pi(to.th - theta);
  const out = [];
  for (const name of Object.keys(WORDS)) {
    const params = wordParams(name, alpha, beta, d);
    if (!params || params.some((value) => !Number.isFinite(value))) continue;
    out.push({
      from,
      radius,
      types: WORDS[name],
      params,
      length: (params[0] + params[1] + params[2]) * radius
    });
  }
  return out.sort((a, b) => a.length - b.length);
}

function stepSegment(x, y, th, type, amount) {
  if (type === L_SEG) {
    return [x + Math.sin(th + amount) - Math.sin(th), y - Math.cos(th + amount) + Math.cos(th), th + amount];
  }
  if (type === R_SEG) {
    return [x - Math.sin(th - amount) + Math.sin(th), y + Math.cos(th - amount) - Math.cos(th), th - amount];
  }
  return [x + Math.cos(th) * amount, y + Math.sin(th) * amount, th];
}

// Samples a Dubins word every `step` metres (endpoints included).
function sampleDubins(word, step) {
  const { from, radius, types, params } = word;
  const total = word.length;
  const count = Math.max(Math.ceil(total / step), 1);
  const points = [];
  for (let index = 0; index <= count; index += 1) {
    let remaining = (index / count) * total / radius;
    let x = 0;
    let y = 0;
    let th = from.th;
    for (let seg = 0; seg < 3; seg += 1) {
      const amount = Math.min(remaining, params[seg]);
      [x, y, th] = stepSegment(x, y, th, types[seg], amount);
      remaining -= amount;
      if (remaining <= 1e-12) break;
    }
    points.push([from.x + x * radius, from.y + y * radius]);
  }
  return points;
}

// ---------------------------------------------------------------------------
// Safe-area helpers (local metre frame)
// ---------------------------------------------------------------------------
function buildArea(safeArea, toLocal) {
  const geometry = safeArea?.geometry || safeArea;
  if (!geometry) return null;
  const polygons = geometry.type === "Polygon" ? [geometry.coordinates]
    : geometry.type === "MultiPolygon" ? geometry.coordinates : null;
  if (!polygons) return null;
  const local = polygons.map((rings) => rings.map((ring) => ring.map(toLocal)));
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  local.forEach(([outer]) => outer.forEach(([x, y]) => {
    minX = Math.min(minX, x);
    minY = Math.min(minY, y);
    maxX = Math.max(maxX, x);
    maxY = Math.max(maxY, y);
  }));
  // Grid-indexed containment: with many obstacles (holes) a plain scan of
  // every ring per sample point made the connector search take minutes.
  const indexed = createArea(local);
  return {
    bounds: { minX, minY, maxX, maxY },
    contains: (x, y) => indexed.contains(x, y)
  };
}

function isInside(area, points, keepClearOf) {
  for (const [x, y] of points) {
    if (keepClearOf.some(([cx, cy]) => Math.hypot(x - cx, y - cy) < ENDPOINT_GRACE_M)) continue;
    if (!area.contains(x, y)) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Planning
// ---------------------------------------------------------------------------
function headingOf(points, fromEnd) {
  const count = points.length;
  for (let i = 1; i < count; i += 1) {
    const a = fromEnd ? points[count - 1 - i] : points[0];
    const b = fromEnd ? points[count - 1] : points[i];
    if (Math.hypot(b[0] - a[0], b[1] - a[1]) > 0.05) return Math.atan2(b[1] - a[1], b[0] - a[0]);
  }
  return 0;
}

const advance = (pose, distance) => ({
  x: pose.x + Math.cos(pose.th) * distance,
  y: pose.y + Math.sin(pose.th) * distance,
  th: pose.th
});

// Shortest word between two poses whose samples all stay in the safe area.
function bestLeg(area, from, to, radius, keepClearOf) {
  for (const word of dubinsCandidates(from, to, radius)) {
    const points = sampleDubins(word, CHECK_STEP_M);
    if (isInside(area, points, keepClearOf)) return word;
  }
  return null;
}

function poseNodes(area, count, radius) {
  const { minX, minY, maxX, maxY } = area.bounds;
  const spacing = Math.max(radius, Math.max(maxX - minX, maxY - minY) / count);
  const points = [];
  for (let x = minX; x <= maxX; x += spacing) {
    for (let y = minY; y <= maxY; y += spacing) {
      if (area.contains(x, y)) points.push([x, y]);
    }
  }
  return points;
}

function oneHop(area, start, goal, radius, keepClearOf) {
  const points = poseNodes(area, ONE_HOP_GRID, radius);
  const ranked = [];
  for (const [x, y] of points) {
    for (let k = 0; k < HEADING_COUNT; k += 1) {
      const pose = { x, y, th: (k * TWO_PI) / HEADING_COUNT };
      const first = dubinsCandidates(start, pose, radius)[0];
      const second = dubinsCandidates(pose, goal, radius)[0];
      if (first && second) ranked.push({ pose, cost: first.length + second.length });
    }
  }
  ranked.sort((a, b) => a.cost - b.cost);
  for (const { pose } of ranked) {
    const first = bestLeg(area, start, pose, radius, keepClearOf);
    if (!first) continue;
    const second = bestLeg(area, pose, goal, radius, keepClearOf);
    if (second) return [first, second];
  }
  return null;
}

function poseGraph(area, start, goal, radius, keepClearOf) {
  const points = poseNodes(area, Math.sqrt(GRAPH_POINT_LIMIT), radius).slice(0, GRAPH_POINT_LIMIT * 2);
  const nodes = [start, goal];
  points.forEach(([x, y]) => {
    for (let k = 0; k < HEADING_COUNT; k += 1) nodes.push({ x, y, th: (k * TWO_PI) / HEADING_COUNT });
  });
  const heuristic = (node) => Math.hypot(goal.x - node.x, goal.y - node.y);
  const best = new Map([[0, 0]]);
  const cameFrom = new Map();
  const open = [{ index: 0, priority: heuristic(start) }];
  const legs = new Map();
  let expansions = 0;

  while (open.length && expansions < MAX_GRAPH_EXPANSIONS) {
    open.sort((a, b) => a.priority - b.priority);
    const { index } = open.shift();
    if (index === 1) {
      const route = [];
      for (let at = 1; at !== 0; at = cameFrom.get(at)) route.unshift(legs.get(`${cameFrom.get(at)}>${at}`));
      return route;
    }
    expansions += 1;
    const node = nodes[index];
    for (let next = 1; next < nodes.length; next += 1) {
      if (next === index) continue;
      const target = nodes[next];
      if (Math.hypot(target.x - node.x, target.y - node.y) > 10 * radius && next !== 1) continue;
      const word = dubinsCandidates(node, target, radius)[0];
      if (!word) continue;
      const cost = best.get(index) + word.length;
      if (cost >= (best.get(next) ?? Infinity)) continue;
      const leg = bestLeg(area, node, target, radius, keepClearOf);
      if (!leg) continue;
      const legCost = best.get(index) + leg.length;
      if (legCost >= (best.get(next) ?? Infinity)) continue;
      best.set(next, legCost);
      cameFrom.set(next, index);
      legs.set(`${index}>${next}`, leg);
      open.push({ index: next, priority: legCost + heuristic(target) });
    }
  }
  return null;
}

function planLegs(area, start, goal, radius, keepClearOf) {
  const direct = bestLeg(area, start, goal, radius, keepClearOf);
  if (direct) return { legs: [direct], strategy: "direct_dubins" };
  const hop = oneHop(area, start, goal, radius, keepClearOf);
  if (hop) return { legs: hop, strategy: "via_waypoint" };
  const graph = poseGraph(area, start, goal, radius, keepClearOf);
  if (graph) return { legs: graph, strategy: "pose_graph" };
  return null;
}

/**
 * Builds a smooth, forward-only connector from the last point of `coordinates`
 * to `gap` metres behind the first point (heading along the first segment),
 * staying inside `safeArea`. Returns null when no such route exists.
 */
function closeMowerPathFallback(coordinates, { turningRadius, gap = 0, safeArea } = {}) {
  const origin = coordinates[coordinates.length - 1];
  const kx = METERS_PER_DEGREE_LONGITUDE * Math.cos((origin[1] * Math.PI) / 180);
  const toLocal = ([lon, lat]) => [(lon - origin[0]) * kx, (lat - origin[1]) * METERS_PER_DEGREE_LATITUDE];
  const toLonLat = ([x, y]) => [x / kx + origin[0], y / METERS_PER_DEGREE_LATITUDE + origin[1]];

  const area = buildArea(safeArea, toLocal);
  if (!area) return null;

  const local = coordinates.map(toLocal);
  const endPose = { x: local[local.length - 1][0], y: local[local.length - 1][1], th: headingOf(local, true) };
  const startTh = headingOf(local, false);
  const startPose = { x: local[0][0], y: local[0][1], th: startTh };
  // Keep as much of the requested gap as fits: the point behind the start can
  // itself fall outside the safe area when the start is near the boundary.
  let goal = startPose;
  for (const fraction of [1, 0.75, 0.5, 0.25, 0]) {
    const candidate = advance(startPose, -gap * fraction);
    if (area.contains(candidate.x, candidate.y)) {
      goal = candidate;
      break;
    }
  }
  const keepClearOf = [[endPose.x, endPose.y], [goal.x, goal.y]];

  {
    const radius = turningRadius;
    // Straight lead-out and lead-in keep the turns away from the end/start point.
    for (const lead of [Math.min(turningRadius, 3), 0]) {
      const leadOut = advance(endPose, lead);
      const leadIn = advance(goal, -lead);
      const straightOut = lead ? [[endPose.x, endPose.y], [leadOut.x, leadOut.y]] : [];
      const straightIn = lead ? [[leadIn.x, leadIn.y], [goal.x, goal.y]] : [];
      if (lead && !(isInside(area, straightOut, keepClearOf) && isInside(area, straightIn, keepClearOf))) continue;

      const plan = planLegs(area, leadOut, leadIn, radius, keepClearOf);
      if (!plan) continue;

      const points = [];
      const push = (list) => list.forEach((point) => {
        const last = points[points.length - 1];
        if (!last || Math.hypot(point[0] - last[0], point[1] - last[1]) > 1e-6) points.push(point);
      });
      if (lead) push(sampleStraight(straightOut, OUTPUT_STEP_M));
      plan.legs.forEach((leg) => push(sampleDubins(leg, OUTPUT_STEP_M)));
      if (lead) push(sampleStraight(straightIn, OUTPUT_STEP_M));
      points[points.length - 1] = [goal.x, goal.y];

      const connectorCoordinates = points.slice(1).map(toLonLat);
      const closed = [...coordinates, ...connectorCoordinates];
      if (gap <= 0) closed[closed.length - 1] = [...coordinates[0]];
      return {
        coordinates: closed,
        connector: [coordinates[coordinates.length - 1], ...connectorCoordinates],
        properties: {
          join_start_end: true,
          gap_to_start_meters: Math.round(Math.hypot(goal.x - startPose.x, goal.y - startPose.y) * 1000) / 1000,
          connector_strategy: plan.strategy,
          connector_turn_radius_meters: Math.round(radius * 1000) / 1000
        }
      };
    }
  }
  return null;
}

function sampleStraight([from, to], step) {
  const length = Math.hypot(to[0] - from[0], to[1] - from[1]);
  const count = Math.max(Math.ceil(length / step), 1);
  return Array.from({ length: count + 1 }, (_, i) => [
    from[0] + ((to[0] - from[0]) * i) / count,
    from[1] + ((to[1] - from[1]) * i) / count
  ]);
}

module.exports = { closeMowerPathFallback, dubinsCandidates, sampleDubins };
