// Fields2Cover-style coverage planner, ported to JS (turf + local-metre
// geometry). Same pipeline and stage boundaries as F2C v2:
//
//   hg      headland.js       constant-width headland around boundary and obstacles
//   decomp  decomposition.js  boustrophedon cells, so rows never straddle an obstacle
//   sg      swaths.js         parallel swaths along the reference line, per column
//   hg      headlandLaps.js   headland swaths: laps that mow the headland band itself
//   rp      routeOrder.js     leapfrog row order inside each cell + greedy cell order
//   pp      pathPlanner.js    min-radius, loop-free Dubins turns, transit around obstacles
//
// Differences from F2C: the swath angle comes from the user's reference line
// (no brute-force angle search), cells are ordered greedily by reachable
// connection length instead of OR-tools, and every turn/transit is validated
// against the real drive area — rows whose connection cannot fit are retried
// later from elsewhere and only reported as skipped if nothing reaches them.

const { createLocalFrame, createSwathFrame } = require('./frame');
const { areaFromGeoJSON } = require('./geometry');
const { fieldArea, generateMainland } = require('./headland');
const { generateSwathColumns } = require('./swaths');
const { decomposeIntoCells } = require('./decomposition');
const { generateCustomOrder } = require('./routeOrder');
const { PathPlanner, pose, smoothWords } = require('./pathPlanner');
const { generateHeadlandLaps } = require('./headlandLaps');

const CONNECTOR_BOUNDARY_CLEARANCE_M = 0.3; // just enough to not sit exactly on the true boundary
const CONNECTOR_OBSTACLE_CLEARANCE_M = 1.0; // minimum physical safety gap from an obstacle during a turn
// Rows shorter than one turning radius (slivers where an obstacle's headland
// nearly meets the boundary's) each cost a full turn for almost no coverage.
const MIN_SWATH_LENGTH_M = 1;
const ROW_LOOKAHEAD = 4; // rows further along the order tried when the next one's turn does not fit
// A row end also counts as escapable when one turn gets 2-3 turning radii
// out: in a mainland narrower than a U-turn, a middle row ends with only the
// headland ahead (no single turn gets 4R away), yet the vehicle can still
// turn onto a headland lap and leave. Laps keep the stricter default.
const ROW_EXIT_ESCAPE_RADII = [2, 3, 4, 5];
const CELL_DIRECT_CANDIDATES = 12; // best-ranked cell entries tried with a single Dubins turn
const CELL_TRANSIT_CANDIDATES = 4; // nearest cell entries compared when a transit is needed
const LAP_CLEARANCE_MARGIN_M = 0.1; // keeps the outermost laps off the drive area's own edge
const LAP_ENTRY_STEP_M = 3; // spacing of candidate entry points along a lap
const LAP_ENTRY_MAX_CURVATURE_FACTOR = 0.8; // laps are entered only where they bend gentler than this × 1/R
const LAP_ENTRY_CLEARANCE_M = 3; // ...for at least this far either side of the entry point
const LAP_ENTRY_DENSIFY_M = 1;
const LAP_DIRECT_CANDIDATES = 40;
const LAP_TRANSIT_CANDIDATES = 4;
const MAX_LAP_BACKTRACKS = 60; // undo budget when the lap router paints itself into a corner
const NEXT_RING_DIRECT_CANDIDATES = 12; // entries on the neighbouring ring tried first, with a single turn
// ...only within this × R: a lane change onto the next ring, not a hop across
// the field that leaves the vehicle in a pocket between obstacles.
const NEXT_RING_REACH_FACTOR = 1;
const MAX_LAP_TRANSIT_FAILURES = 3; // a lap entry transits failed to this often (from different places) is not tried again
const TRIM_STEP_M = 2; // how far an open lap end is pulled back per escape check
const RANKED_ENTRY_POOL = 400; // nearest entries (straight-line) re-ranked by smooth turn length
const VIA_LAP_ON_CANDIDATES = 12; // lap entries tried for hopping onto a lap in a via-lap transit
const VIA_LAP_REACH_M = 30; // targets tried from points along that lap within this distance
const VIA_LAP_MAX_TURN_RAD = Math.PI + 0.35; // same no-loop limit as the path planner's turns
const VIA_LAP_STRAIGHT_BREAK_M = 0.5; // a straight this long ends one continuous turn

const statusError = (message, status = 400) => Object.assign(new Error(message), { status });
const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

// Longest continuous same-direction turn along a polyline: a hop's arc, the
// lap's own bend and the next hop's arc chained together must not add up to
// a loop.
function longestPolylineTurn(points) {
  let longest = 0;
  let run = 0;
  let straight = 0;
  for (let i = 1; i + 1 < points.length; i += 1) {
    const a = points[i - 1];
    const b = points[i];
    const c = points[i + 1];
    const length = distance(b, c);
    if (length < 1e-6 || distance(a, b) < 1e-6) continue;
    let turn = Math.atan2(c[1] - b[1], c[0] - b[0]) - Math.atan2(b[1] - a[1], b[0] - a[0]);
    turn = Math.atan2(Math.sin(turn), Math.cos(turn));
    if (Math.abs(turn) < 2e-3) {
      straight += length;
      if (straight >= VIA_LAP_STRAIGHT_BREAK_M) run = 0;
      continue;
    }
    straight = 0;
    run = Math.sign(run) === Math.sign(turn) ? run + turn : turn;
    longest = Math.max(longest, Math.abs(run));
  }
  return longest;
}

const rowStart = (row, dir) => (dir > 0 ? row.a : row.b);
const rowEnd = (row, dir) => (dir > 0 ? row.b : row.a);
const rowHeading = (row, dir) => (dir > 0 ? row.heading : row.heading + Math.PI);
const entryPose = (row, dir) => pose(...rowStart(row, dir), rowHeading(row, dir));
const exitPose = (row, dir) => pose(...rowEnd(row, dir), rowHeading(row, dir));

// Adds points every LAP_ENTRY_DENSIFY_M along segments longer than that.
function densifyPath(points, closed) {
  const out = [];
  const n = points.length;
  for (let i = 0; i < n; i += 1) {
    const a = points[i];
    out.push(a);
    if (!closed && i === n - 1) break;
    const b = points[(i + 1) % n];
    const count = Math.floor(distance(a, b) / LAP_ENTRY_DENSIFY_M);
    for (let k = 1; k < count; k += 1) {
      out.push([a[0] + ((b[0] - a[0]) * k) / count, a[1] + ((b[1] - a[1]) * k) / count]);
    }
  }
  return out;
}

// Orders candidate entries by the length of the shortest loop-free turn into
// them (ignoring obstacles — that is checked afterwards). Straight-line
// distance alone puts targets right beside the vehicle first, and those are
// exactly the ones only a loop can reach.
function rankBySmoothTurn(from, radius, entries) {
  return entries
    .map((entry) => ({ entry, d: Math.hypot(entry.target.x - from.x, entry.target.y - from.y) }))
    .sort((a, b) => a.d - b.d)
    .slice(0, RANKED_ENTRY_POOL)
    .map(({ entry, d }) => {
      const word = smoothWords(from, entry.target, radius)[0];
      return { entry, cost: word ? word.length : Infinity, d };
    })
    .sort((a, b) => a.cost - b.cost || a.d - b.d)
    .map(({ entry }) => entry);
}

function buildCells(mainland, direction, spacing, turningRadius, pass) {
  const swathFrame = createSwathFrame(direction);
  const columns = generateSwathColumns(mainland, swathFrame, spacing, Math.max(MIN_SWATH_LENGTH_M, turningRadius));
  return decomposeIntoCells(columns).map(({ rows }) => ({
    pass,
    rows: rows.map(({ u, v0, v1 }) => ({
      u,
      v0,
      a: swathFrame.fromUV([u, v0]),
      b: swathFrame.fromUV([u, v1]),
      heading: swathFrame.heading
    })),
    order: generateCustomOrder(rows.length, spacing, turningRadius)
  }));
}

class CoverageRoute {
  constructor(planner, roads = []) {
    this.planner = planner;
    this.points = [];
    this.pose = null;
    this.escapable = new Map(); // row -> { f, r }: exit pose can still turn away
    this.roads = roads; // headland laps, also usable as smooth roads for transits
    this.roadStops = null;
    this.transitFailures = new Map(); // lap entry target -> failed transits to it
  }

  // Last-resort transit for targets in a cul-de-sac that a direct turn or
  // the visibility-graph transit cannot reach at the right heading: hop onto
  // a headland lap, follow it, and hop off onto the target. Laps are already
  // smooth, drivable curves, entered and left only where they bend gently;
  // the whole chain is still checked so hop, lap bend and hop never add up
  // to a loop.
  viaLaps(targets) {
    if (!this.roads.length || !targets.length) return null;
    if (!this.roadStops) {
      this.roadStops = new Map(); // pts -> entries along that lap direction, by index
      this.roads.flatMap((lap) => this.lapEntries(lap)).forEach((stop) => {
        if (!this.roadStops.has(stop.pts)) this.roadStops.set(stop.pts, []);
        this.roadStops.get(stop.pts).push(stop);
      });
    }
    const allStops = [...this.roadStops.values()].flat();
    const ons = rankBySmoothTurn(this.pose, this.planner.radius, allStops).slice(0, VIA_LAP_ON_CANDIDATES);
    let best = null;
    for (const on of ons) {
      const hopOn = this.planner.direct(this.pose, on.target);
      if (!hopOn) continue;
      const { pts, lap } = on;
      const n = pts.length;
      const ahead = (k) => (lap.closed ? (k - on.index + n) % n : k - on.index);
      const stops = this.roadStops.get(pts)
        .filter((stop) => ahead(stop.index) > 0)
        .sort((a, b) => ahead(a.index) - ahead(b.index));
      let along = 0;
      let k = on.index;
      for (const stop of stops) {
        while (k !== stop.index) {
          const next = (k + 1) % n;
          along += distance(pts[k], pts[next]);
          k = next;
        }
        const sofar = hopOn.length + along;
        if (best && sofar >= best.connection.length) break;
        let found = false;
        for (const entry of targets) {
          if (distance([stop.target.x, stop.target.y], [entry.target.x, entry.target.y]) > VIA_LAP_REACH_M) continue;
          const hopOff = this.planner.direct(stop.target, entry.target);
          if (!hopOff || (best && sofar + hopOff.length >= best.connection.length)) continue;
          const road = [];
          for (let j = on.index; j !== stop.index; j = (j + 1) % n) road.push(pts[(j + 1) % n]);
          const points = [...hopOn.points, ...road.slice(0, -1), ...hopOff.points];
          if (longestPolylineTurn(points) > VIA_LAP_MAX_TURN_RAD) continue;
          best = { ...entry, connection: { points, length: sofar + hopOff.length } };
          found = true;
        }
        if (found) break;
      }
    }
    return best;
  }

  // True when the vehicle can still drive away smoothly after mowing `row`
  // in direction `dir`. A row ending in a pocket too tight to turn in leaves
  // the route stuck there, and every row after it becomes unreachable.
  exitEscapable(row, dir) {
    const key = dir > 0 ? 'f' : 'r';
    let known = this.escapable.get(row);
    if (!known) this.escapable.set(row, (known = {}));
    if (!(key in known)) known[key] = this.planner.canEscape(exitPose(row, dir), ROW_EXIT_ESCAPE_RADII);
    return known[key];
  }

  // Tries `connect` on each candidate entry and keeps the shortest result
  // (or the first one, when stopAtFirst).
  pick(candidates, connect, stopAtFirst) {
    let best = null;
    for (const entry of candidates) {
      const connection = connect(this.pose, entry.target);
      if (connection && (!best || connection.length < best.connection.length)) {
        best = { ...entry, connection };
        if (stopAtFirst) break;
      }
    }
    return best;
  }

  driveRow(row, dir, connection) {
    if (connection) this.points.push(...connection.points.slice(1));
    else this.points.push(rowStart(row, dir));
    this.points.push(rowEnd(row, dir));
    this.pose = exitPose(row, dir);
  }

  // Cheapest valid connection into `row`, either direction (the U-turn
  // direction wins on length whenever it fits). Within a cell no obstacle
  // lies between rows, so the fallback is only the two-hop turnaround — the
  // full obstacle transit is left to the cell-level router.
  connectRow(row, allowTurnaround) {
    const tryWith = (connect) => {
      let best = null;
      for (const dir of [1, -1]) {
        if (!this.exitEscapable(row, dir)) continue;
        const connection = connect(this.pose, entryPose(row, dir));
        if (connection && (!best || connection.length < best.connection.length)) best = { dir, connection };
      }
      return best;
    };
    return tryWith((a, b) => this.planner.direct(a, b))
      || (allowTurnaround ? tryWith((a, b) => this.planner.twoHop(a, b)) : null);
  }

  // Drives one cell along its leapfrog sequence. A row whose turn does not
  // fit is passed over for the next few in order and retried afterwards;
  // rows still unreachable from inside the cell are returned as orphans for
  // the cell-level router to reach from somewhere else.
  runCell(cell, entry) {
    const pending = entry.sequence.map((i) => cell.rows[i]);
    this.driveRow(pending.shift(), entry.dir, entry.connection);
    const orphans = [];
    while (pending.length) {
      let taken = -1;
      let best = null;
      for (let i = 0; i < Math.min(ROW_LOOKAHEAD, pending.length); i += 1) {
        best = this.connectRow(pending[i], false);
        if (best) {
          taken = i;
          break;
        }
      }
      if (taken < 0) {
        best = this.connectRow(pending[0], true);
        if (best) taken = 0;
      }
      if (taken < 0) {
        orphans.push(pending.shift());
        continue;
      }
      const [row] = pending.splice(taken, 1);
      this.driveRow(row, best.dir, best.connection);
    }
    return orphans;
  }

  // Each cell can be entered from either end of its sweep, on either row
  // direction. Picks the cheapest entry a single smooth turn reaches; falls
  // back to transiting when none does.
  chooseNextCell(pool) {
    const here = [this.pose.x, this.pose.y];
    const entries = pool.flatMap((cell) => [false, true].flatMap((mirrored) => {
      const n = cell.rows.length;
      const sequence = mirrored ? cell.order.map((i) => n - 1 - i) : cell.order;
      const first = cell.rows[sequence[0]];
      return [1, -1].filter((dir) => this.exitEscapable(first, dir))
        .map((dir) => ({ cell, sequence, dir, target: entryPose(first, dir) }));
    }));
    const ranked = rankBySmoothTurn(this.pose, this.planner.radius, entries);
    entries.sort((a, b) => distance(here, [a.target.x, a.target.y]) - distance(here, [b.target.x, b.target.y]));
    return this.pick(ranked.slice(0, CELL_DIRECT_CANDIDATES), (a, b) => this.planner.direct(a, b), false)
      || this.pick(entries.slice(0, CELL_TRANSIT_CANDIDATES), (a, b) => this.planner.transit(a, b), false)
      || this.pick(entries.slice(CELL_TRANSIT_CANDIDATES), (a, b) => this.planner.transit(a, b), true)
      || this.viaLaps(entries);
  }

  // Covers every cell of one pass. Returns the number of rows that could
  // not be reached at all.
  // Covers every cell of one pass. Returns the number of rows that could
  // not be reached at all.
  coverPass(cells) {
    const pool = cells.filter((cell) => cell.rows.length > 0);
    if (pool.length === 0) return 0;

    if (!this.pose) {
      // Start in the biggest cell, on its leftmost row (relative to the
      // reference direction), driving along the reference line. Starting in
      // a small side cell cut off by obstacles can strand the whole route
      // there; from the main cell, side cells are reached afterwards or at
      // worst skipped on their own. If the first row ends in a pocket the
      // vehicle cannot turn out of, take the next start (other cell end,
      // other direction, next cell) whose first row can be left again.
      const sorted = [...pool].sort((a, b) => b.rows.length - a.rows.length
        || a.rows[0].u - b.rows[0].u || a.rows[0].v0 - b.rows[0].v0);
      const starts = sorted.flatMap((cell) => [false, true].flatMap((mirrored) => [1, -1].map((dir) => {
        const n = cell.rows.length;
        return { cell, dir, sequence: mirrored ? cell.order.map((i) => n - 1 - i) : cell.order };
      })));
      const start = starts.find(({ cell, sequence, dir }) => this.exitEscapable(cell.rows[sequence[0]], dir)) || starts[0];
      pool.splice(pool.indexOf(start.cell), 1);
      this.runCell(start.cell, { sequence: start.sequence, dir: start.dir, connection: null })
        .forEach((row) => pool.push({ pass: start.cell.pass, rows: [row], order: [0] }));
    }

    while (pool.length) {
      const next = this.chooseNextCell(pool);
      if (!next) {
        // Nothing left is reachable from here, so nothing left ever will be.
        return pool.reduce((sum, cell) => sum + cell.rows.length, 0);
      }
      pool.splice(pool.indexOf(next.cell), 1);
      this.runCell(next.cell, next)
        .forEach((row) => pool.push({ pass: next.cell.pass, rows: [row], order: [0] }));
    }
    return 0;
  }

  // Entry options along a lap (sampled), in both driving directions. A
  // closed lap is driven fully round from its entry; an open piece from its
  // entry to its end — the part behind the entry goes back in the pool.
  lapEntries(lap) {
    const out = [];
    [false, true].forEach((reversed) => {
      // Densified copy, so long straight sides (stored as just their two end
      // points) get candidate entry points along them.
      const pts = densifyPath(reversed ? [...lap.points].reverse() : lap.points, lap.closed);
      const n = pts.length;
      const headingAt = (k) => {
        const a = pts[k];
        const b = lap.closed ? pts[(k + 1) % n] : pts[Math.min(k + 1, n - 1)];
        return Math.atan2(b[1] - a[1], b[0] - a[0]);
      };
      // Only where the lap bends gently over LAP_ENTRY_CLEARANCE_M either
      // side (straights, wide orbits round an obstacle): entering — and, for
      // a closed lap, leaving — at a tight corner manoeuvre would chain that
      // arc with the connecting turn into a semicircle. Averaged over the
      // window, so a wide orbit built from short straights and corner arcs
      // still counts as the wide curve it is.
      const pointAlong = (k, step) => {
        let along = 0;
        let j = k;
        while (along < LAP_ENTRY_CLEARANCE_M) {
          const next = j + step;
          if (!lap.closed && (next < 0 || next >= n)) return null;
          along += distance(pts[(j + n) % n], pts[(next + n) % n]);
          j = next;
          if (Math.abs(j - k) >= n) return null;
        }
        return { index: (j + n) % n, along };
      };
      const clearAround = (k) => {
        if (!lap.closed && (k === 0 || k === n - 1)) return true;
        const back = pointAlong(k, -1);
        const ahead = pointAlong(k, 1);
        if (!back || !ahead) return !lap.closed;
        const hBack = Math.atan2(pts[(back.index + 1) % n][1] - pts[back.index][1], pts[(back.index + 1) % n][0] - pts[back.index][0]);
        const hAhead = Math.atan2(pts[ahead.index][1] - pts[(ahead.index - 1 + n) % n][1], pts[ahead.index][0] - pts[(ahead.index - 1 + n) % n][0]);
        let turn = hAhead - hBack;
        turn = Math.abs(Math.atan2(Math.sin(turn), Math.cos(turn)));
        return turn / (back.along + ahead.along) < LAP_ENTRY_MAX_CURVATURE_FACTOR / this.planner.radius;
      };
      let travelled = Infinity;
      for (let k = 0; k < (lap.closed ? n : n - 1); k += 1) {
        if (k > 0) travelled += distance(pts[k - 1], pts[k]);
        if (travelled >= LAP_ENTRY_STEP_M && clearAround(k)) {
          out.push({ lap, pts, index: k, target: pose(pts[k][0], pts[k][1], headingAt(k)) });
          travelled = 0;
        }
      }
    });
    return out;
  }

  // Drives a lap from the chosen entry. For an open piece entered part way
  // along, returns the part behind the entry as a new open piece (or null).
  driveLap({ lap, pts, index, connection }) {
    const sequence = lap.closed ? [...pts.slice(index), ...pts.slice(0, index + 1)] : pts.slice(index);
    const behind = lap.closed ? [] : pts.slice(0, index + 1);
    if (connection) this.points.push(...connection.points.slice(1));
    else this.points.push(sequence[0]);
    this.points.push(...sequence.slice(1));
    const a = sequence[sequence.length - 2];
    const b = sequence[sequence.length - 1];
    this.pose = pose(b[0], b[1], Math.atan2(b[1] - a[1], b[0] - a[0]));
    const behindLength = behind.reduce((sum, p, i) => (i ? sum + distance(behind[i - 1], p) : 0), 0);
    return behindLength >= this.planner.radius ? { closed: false, points: behind, depth: lap.depth } : null;
  }

  // Shortens an open piece from its end until the vehicle can still drive
  // away smoothly from there; null when nothing of it is escapable.
  trimToEscapableEnd(points) {
    const pts = points.slice();
    while (pts.length >= 2) {
      const a = pts[pts.length - 2];
      const b = pts[pts.length - 1];
      if (this.planner.canEscape(pose(b[0], b[1], Math.atan2(b[1] - a[1], b[0] - a[0])))) return pts;
      let removed = 0;
      while (pts.length >= 2 && removed < TRIM_STEP_M) {
        removed += distance(pts[pts.length - 2], pts[pts.length - 1]);
        pts.pop();
      }
    }
    return null;
  }

  // Open pieces end where a ring got too tight to follow — often a pocket
  // the vehicle could drive into but not out of. Trim both ends back to
  // where it can still leave smoothly.
  escapablePiece(lap) {
    if (lap.closed) return lap;
    const forward = this.trimToEscapableEnd(lap.points);
    const both = forward && this.trimToEscapableEnd([...forward].reverse());
    if (!both) return null;
    const length = both.reduce((sum, p, i) => (i ? sum + distance(both[i - 1], p) : 0), 0);
    return length >= this.planner.radius ? { closed: false, points: both.reverse(), depth: lap.depth } : null;
  }

  // Mows the headland laps after the rows. From the end of each lap the
  // cheapest reachable entry is usually the neighbouring lap one coverage
  // width away, reached with a short lane-change S-curve. Returns the number
  // of laps (or lap pieces) that could not be reached.
  coverLaps(laps) {
    const pool = laps.map((lap) => this.escapablePiece(lap)).filter(Boolean);
    const dropped = laps.length - pool.length;
    if (pool.length === 0) return dropped;
    if (!this.pose) {
      const first = pool.shift();
      const remainder = this.driveLap({ lap: first, pts: first.points, index: 0, connection: null });
      if (remainder) pool.push(remainder);
      this.lapDepth = first.depth;
    }
    const entriesOf = new Map(pool.map((lap) => [lap, this.lapEntries(lap)]));

    // Greedy, with bounded backtracking: when nothing left is reachable from
    // where the last lap ended (a corridor too narrow to turn in, say), undo
    // the last choices and try a different lap or entry instead of giving up
    // on everything still in the pool.
    const ids = new WeakMap(); // WeakMap has no size, so ids come from a counter
    let nextId = 1;
    const idOf = (obj) => {
      if (!ids.has(obj)) ids.set(obj, nextId++);
      return ids.get(obj);
    };
    const history = [];
    const banned = new Set();
    const stateKey = () => history.map((step) => step.key).join('>');
    const entryKey = (entry) => `${idOf(entry.pts)}:${entry.index}`;
    let backtracks = 0;

    while (pool.length) {
      const prefix = stateKey();
      const allowed = (entry) => !banned.has(`${prefix}|${entryKey(entry)}`);
      const next = this.chooseNextLap(pool, entriesOf, allowed);
      if (!next) {
        if (!history.length || backtracks >= MAX_LAP_BACKTRACKS) return dropped + pool.length;
        backtracks += 1;
        const step = history.pop();
        this.points.length = step.pointsLength;
        this.pose = step.pose;
        this.lapDepth = step.depth;
        if (step.remainder) {
          pool.splice(pool.indexOf(step.remainder), 1);
          entriesOf.delete(step.remainder);
        }
        pool.push(step.lap);
        banned.add(`${stateKey()}|${step.key}`);
        continue;
      }
      const step = { key: entryKey(next), lap: next.lap, pointsLength: this.points.length, pose: this.pose, depth: this.lapDepth, remainder: null };
      pool.splice(pool.indexOf(next.lap), 1);
      const remainder = this.driveLap(next);
      if (remainder) {
        pool.push(remainder);
        entriesOf.set(remainder, this.lapEntries(remainder));
        step.remainder = remainder;
      }
      this.lapDepth = next.lap.depth;
      history.push(step);
    }
    return dropped;
  }

  // Best reachable lap entry from the current pose, among `allowed` ones.
  // The ring nearest in depth to the one just mowed comes first (the deeper
  // one on a tie, and the innermost ring, bordering the rows, when starting),
  // so the laps are mowed ring by ring, each hop a short lane change of one
  // coverage width, instead of hopping across rings the nearest turn happens
  // to reach. Otherwise the cheapest smooth turn first, then transits.
  chooseNextLap(pool, entriesOf, allowed) {
    const depths = [...new Set(pool.map((lap) => lap.depth))];
    const from = this.lapDepth == null ? Infinity : this.lapDepth;
    const nextDepth = depths.reduce((best, d) => {
      const gap = Math.abs(d - from);
      const bestGap = Math.abs(best - from);
      return gap < bestGap || (gap === bestGap && d > best) ? d : best;
    });
    const reach = NEXT_RING_REACH_FACTOR * this.planner.radius;
    const nextRing = pool.filter((lap) => lap.depth === nextDepth).flatMap((lap) => entriesOf.get(lap)).filter(allowed)
      .filter((entry) => Math.hypot(entry.target.x - this.pose.x, entry.target.y - this.pose.y) <= reach);
    const onNextRing = this.pick(
      rankBySmoothTurn(this.pose, this.planner.radius, nextRing).slice(0, NEXT_RING_DIRECT_CANDIDATES),
      (a, b) => this.planner.direct(a, b),
      false
    );
    if (onNextRing) return onNextRing;

    const { x, y } = this.pose;
    const entries = pool
      .flatMap((lap) => entriesOf.get(lap))
      .filter(allowed)
      .map((entry) => ({ entry, d: Math.hypot(entry.target.x - x, entry.target.y - y) }))
      .sort((a, b) => a.d - b.d)
      .map(({ entry }) => entry);
    const ranked = rankBySmoothTurn(this.pose, this.planner.radius, entries);
    // Transit fallback over the rest tries just one (nearest) entry per lap.
    const seen = new Set();
    const onePerLap = entries.slice(LAP_TRANSIT_CANDIDATES).filter((e) => !seen.has(e.lap) && seen.add(e.lap));
    return this.pick(ranked.slice(0, LAP_DIRECT_CANDIDATES), (a, b) => this.planner.direct(a, b), false)
      || this.pick(entries.slice(0, LAP_TRANSIT_CANDIDATES).filter((e) => this.transitWorthTrying(e)), (a, b) => this.lapTransit(a, b), false)
      || this.pick(onePerLap.filter((e) => this.transitWorthTrying(e)), (a, b) => this.lapTransit(a, b), true)
      || this.viaLaps(entries);
  }

  // Lap entries no transit reaches (a ring round a small obstacle in a
  // pocket, say) would otherwise be retried from every new position, each a
  // full visibility-graph search that fails — most of the planning time with
  // many obstacles. After a few failures from different places, give up on
  // them; they stay reachable by direct turns and via other laps.
  transitWorthTrying(entry) {
    return (this.transitFailures.get(entry.target) || 0) < MAX_LAP_TRANSIT_FAILURES;
  }

  lapTransit(from, target) {
    const connection = this.planner.transit(from, target);
    if (!connection) this.transitFailures.set(target, (this.transitFailures.get(target) || 0) + 1);
    return connection;
  }
}

function planCoverage({ polygonCoords, obstacleRings, referenceLineCoords, headlandWidth, spacing, turningRadius, crissCross }) {
  if (!Number.isFinite(turningRadius) || turningRadius <= 0) {
    throw statusError("A positive turning radius is required to generate the mower path.");
  }
  if (!Number.isFinite(spacing) || spacing <= 0) {
    throw statusError("Spacing must be a positive number of metres.");
  }

  const frame = createLocalFrame(referenceLineCoords[0]);
  const refStart = frame.toLocal(referenceLineCoords[0]);
  const refEnd = frame.toLocal(referenceLineCoords[referenceLineCoords.length - 1]);
  const refLength = distance(refStart, refEnd);
  if (!(refLength > 1e-6)) throw statusError("The reference line needs two distinct end points.");
  const direction = [(refEnd[0] - refStart[0]) / refLength, (refEnd[1] - refStart[1]) / refLength];

  const mainlandFeature = generateMainland(polygonCoords, obstacleRings, headlandWidth);
  const driveFeature = fieldArea(
    polygonCoords,
    obstacleRings,
    Math.min(headlandWidth, CONNECTOR_BOUNDARY_CLEARANCE_M),
    Math.min(headlandWidth, CONNECTOR_OBSTACLE_CLEARANCE_M)
  );
  const mainland = areaFromGeoJSON(mainlandFeature, frame.toLocal);
  const driveArea = areaFromGeoJSON(driveFeature, frame.toLocal);
  const planner = new PathPlanner({ driveArea, driveFeature, toLocal: frame.toLocal, radius: turningRadius });

  // Criss-cross: the full primary grid first, then a second grid at 90°.
  const directions = crissCross ? [direction, [direction[1], -direction[0]]] : [direction];
  const passes = directions.map((dir, pass) => buildCells(mainland, dir, spacing, turningRadius, pass));

  // Headland laps fill the band the rows leave for turning, so everything
  // but the obstacles (and their clearance) is mowed.
  const laps = generateHeadlandLaps({
    polygonCoords,
    obstacleRings,
    headlandWidth,
    spacing,
    boundaryStart: Math.max(spacing / 2, CONNECTOR_BOUNDARY_CLEARANCE_M + LAP_CLEARANCE_MARGIN_M),
    obstacleStart: Math.max(spacing / 2, CONNECTOR_OBSTACLE_CLEARANCE_M + LAP_CLEARANCE_MARGIN_M),
    toLocal: frame.toLocal,
    turningRadius,
    driveArea
  });

  const route = new CoverageRoute(planner, laps);
  let skipped = 0;
  passes.forEach((cells) => {
    skipped += route.coverPass(cells);
  });
  const skippedLaps = route.coverLaps(laps);

  return {
    coordinates: route.points.map(frame.toLonLat),
    driveFeature,
    rowCounts: passes.map((cells) => cells.reduce((sum, cell) => sum + cell.rows.length, 0)),
    cellCount: passes.reduce((sum, cells) => sum + cells.length, 0),
    skippedRows: skipped,
    lapCount: laps.length,
    skippedLaps
  };
}

module.exports = { planCoverage, CONNECTOR_BOUNDARY_CLEARANCE_M, CONNECTOR_OBSTACLE_CLEARANCE_M };
