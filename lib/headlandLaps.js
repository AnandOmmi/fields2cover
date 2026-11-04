const turf = require('@turf/turf');
const { sampleDubins } = require('./mowerLoopFallback');
const { smoothWords, chainIsSmooth } = require('./pathPlanner');
const { areaFromGeoJSON } = require('./geometry');
const { fieldArea } = require('./headland');

// Port of Fields2Cover's hg::ConstHL::generateHeadlandSwaths: the headland
// band the rows leave for turning (around the boundary and around every
// obstacle) is mowed as laps, one coverage width apart, so only the
// obstacles themselves stay uncut.
//
// A lap follows an offset ring, but a ring can bend tighter than the vehicle
// can turn: sharp field corners, inward notches, and the corners of any
// obstacle closer than one turning radius. Those stretches are replaced by a
// validated, loop-free Dubins manoeuvre (a rounded corner). A stretch with no
// such manoeuvre cuts the lap there; the pieces are then driven as open
// paths.

const TIGHT_CURVATURE_FACTOR = 1; // ring vertices bending tighter than 1/R need a manoeuvre
const MAX_PLAIN_VERTEX_TURN_RAD = 0.2; // ~11.5°: any sharper polyline kink gets a manoeuvre too, however long its sides
const CHECK_STEP_M = 0.25;
const OUTPUT_STEP_M = 0.1;
const RING_DENSIFY_STEP_M = 1; // so a long straight side has easy vertices between its corners
const RING_SMOOTHING_PASSES = 3;
const RING_SAMPLE_STEP_M = 0.5;
const MAX_MANOEUVRE_CHECKS = 80;
const MERGE_ATTEMPTS = 5;
const WIDE_CORNER_OFFSETS = [0.25, 0.5, 0.75, 1, 1.5]; // apex pulled out by this × R from the corner's middle
const MAX_LAP_ARC_RAD = (150 * Math.PI) / 180; // same no-semicircle limit as the path planner
const STRAIGHT_BREAK_FACTOR = 0.5; // a straight this × R separates two arcs (the flat-U-turn straight)
const OPENING_STEPS = 32; // buffer arc steps per quarter circle, so rounded corners stay smooth polylines

const headingOf = (a, b) => Math.atan2(b[1] - a[1], b[0] - a[0]);
const wrapAngle = (a) => Math.atan2(Math.sin(a), Math.cos(a));

function cleanRing(ring) {
  const out = [];
  ring.forEach((p) => {
    const last = out[out.length - 1];
    if (!last || Math.hypot(p[0] - last[0], p[1] - last[1]) > 1e-3) out.push(p);
  });
  while (out.length > 1 && Math.hypot(out[0][0] - out[out.length - 1][0], out[0][1] - out[out.length - 1][1]) <= 1e-3) out.pop();
  const dense = [];
  out.forEach((a, i) => {
    const b = out[(i + 1) % out.length];
    const count = Math.max(1, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / RING_DENSIFY_STEP_M));
    for (let k = 0; k < count; k += 1) dense.push([a[0] + ((b[0] - a[0]) * k) / count, a[1] + ((b[1] - a[1]) * k) / count]);
  });
  return dense;
}

// Morphological opening (shrink by R, grow back by R): rounds every convex
// corner of the offset area to at least the turning radius, so the lap can
// follow its ring there instead of swapping the corner for a manoeuvre that
// leaves the ring and runs over the neighbouring laps. Openings of nested
// areas stay nested, so rounded laps never cross each other. An opening that
// changes the area's shape — a corridor narrower than 2R between an obstacle
// and the boundary pinched shut, an area too thin to open at all — would
// drop those laps, so the plain area is kept there instead.
const ringCounts = (area) => {
  const { type, coordinates } = area.geometry;
  const polygons = type === "MultiPolygon" ? coordinates : [coordinates];
  return `${polygons.length}:${polygons.reduce((sum, rings) => sum + rings.length, 0)}`;
};
function openArea(area, radius) {
  try {
    const shrunk = turf.buffer(area, -radius / 1000, { units: "kilometers", steps: OPENING_STEPS });
    const opened = shrunk && turf.buffer(shrunk, radius / 1000, { units: "kilometers", steps: OPENING_STEPS });
    return opened && ringCounts(opened) === ringCounts(area) ? opened : area;
  } catch {
    return area;
  }
}

// Offset rings for every lap, one coverage width apart. Boundary laps start
// half a coverage width in from the boundary; obstacle laps start at the
// obstacle clearance. Each lap's rings are the outer ring and hole rings of
// that offset area, so an obstacle close to the boundary merges into it
// naturally.
function generateLapRings({ polygonCoords, obstacleRings, headlandWidth, spacing, boundaryStart, obstacleStart, toLocal, turningRadius }) {
  const rings = [];
  const count = Math.ceil(headlandWidth / spacing - 1e-9);
  for (let j = 0; j < count; j += 1) {
    let area;
    try {
      area = fieldArea(polygonCoords, obstacleRings, boundaryStart + j * spacing, obstacleStart + j * spacing);
    } catch {
      break; // offset has consumed the whole field
    }
    areaFromGeoJSON(openArea(area, turningRadius), toLocal).polygons
      .forEach((polygon) => polygon.forEach((ring) => rings.push({ depth: j, points: cleanRing(ring) })));
  }
  return rings.filter((ring) => ring.points.length >= 3);
}

// Closed polyline with arc-length lookup.
function ringGeometry(points) {
  const n = points.length;
  const starts = new Array(n);
  let total = 0;
  for (let i = 0; i < n; i += 1) {
    starts[i] = total;
    const a = points[i];
    const b = points[(i + 1) % n];
    total += Math.hypot(b[0] - a[0], b[1] - a[1]);
  }
  const segmentAt = (s) => {
    let lo = 0;
    let hi = n - 1;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (starts[mid] <= s) lo = mid;
      else hi = mid - 1;
    }
    return lo;
  };
  return {
    n,
    total,
    starts,
    // Point and travel heading at arc length s (wrapped onto the ring).
    at(s) {
      const t = ((s % total) + total) % total;
      const i = segmentAt(t);
      const a = points[i];
      const b = points[(i + 1) % n];
      const len = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1;
      const f = (t - starts[i]) / len;
      return { p: [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f], th: headingOf(a, b) };
    },
    // Ring vertices strictly between arc lengths from < to (to may exceed total).
    verticesBetween(from, to) {
      const out = [];
      for (let lap = 0; lap <= 1; lap += 1) {
        for (let i = 0; i < n; i += 1) {
          const s = starts[i] + lap * total;
          if (s > from + 1e-6 && s < to - 1e-6) out.push(points[i]);
        }
      }
      return out;
    }
  };
}

// Removes the in-line points densification added on straight sides (Dubins
// samples on arcs are kept — they are never collinear).
function dropCollinear(points, closed) {
  const n = points.length;
  if (n < 3) return points;
  return points.filter((v, i) => {
    if (!closed && (i === 0 || i === n - 1)) return true;
    const p = points[(i - 1 + n) % n];
    const q = points[(i + 1) % n];
    const cross = (v[0] - p[0]) * (q[1] - v[1]) - (v[1] - p[1]) * (q[0] - v[0]);
    const dot = (v[0] - p[0]) * (q[0] - v[0]) + (v[1] - p[1]) * (q[1] - v[1]);
    return dot <= 0 || Math.abs(cross) > 1e-6 * Math.max(dot, 1e-9) * 10;
  });
}

function straight(a, b, step) {
  const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
  const count = Math.max(1, Math.ceil(len / step));
  return Array.from({ length: count + 1 }, (_, i) => [a[0] + ((b[0] - a[0]) * i) / count, a[1] + ((b[1] - a[1]) * i) / count]);
}

// Best drivable replacement for the ring between arc lengths a..b (both tight
// vertices). Entry is either on the ring up to roomBack before a, or an
// overshoot straight past a; exit is on the ring up to roomFwd after b, or a
// straight run-in onto b. Candidates are checked shortest/closest-first.
function bestManoeuvre(ring, a, b, roomBack, roomFwd, radius, driveArea) {
  const offsets = [0, radius / 2, radius, 1.5 * radius, 2 * radius];
  const pa = ring.at(a).p;
  const pb = ring.at(b).p;
  const tIn = ring.at(a - 1e-3).th;
  const tOut = ring.at(b + 1e-3).th;

  const entries = [];
  offsets.forEach((s) => {
    if (s > roomBack + 1e-9) return;
    const { p, th } = s === 0 ? { p: pa, th: tIn } : ring.at(a - s);
    entries.push({ arc: a - s, pose: { x: p[0], y: p[1], th }, lead: null, penalty: s });
  });
  offsets.slice(1).forEach((e) => {
    const p = [pa[0] + e * Math.cos(tIn), pa[1] + e * Math.sin(tIn)];
    entries.push({ arc: a, pose: { x: p[0], y: p[1], th: tIn }, lead: [pa, p], penalty: e });
  });

  const exits = [];
  offsets.forEach((s) => {
    if (s > roomFwd + 1e-9) return;
    const { p, th } = s === 0 ? { p: pb, th: tOut } : ring.at(b + s);
    exits.push({ arc: b + s, pose: { x: p[0], y: p[1], th }, tail: null, penalty: s });
  });
  offsets.slice(1).forEach((f) => {
    const p = [pb[0] - f * Math.cos(tOut), pb[1] - f * Math.sin(tOut)];
    exits.push({ arc: b, pose: { x: p[0], y: p[1], th: tOut }, tail: [p, pb], penalty: f });
  });

  const candidates = [];
  entries.forEach((entry) => exits.forEach((exit) => {
    smoothWords(entry.pose, exit.pose, radius).forEach((word) => {
      const extra = (entry.lead ? entry.penalty : 0) + (exit.tail ? exit.penalty : 0);
      candidates.push({ entry, exit, word, score: word.length + extra + 0.5 * (entry.penalty + exit.penalty) });
    });
  }));
  candidates.sort((x, y) => x.score - y.score);

  for (const { entry, exit, word } of candidates.slice(0, MAX_MANOEUVRE_CHECKS)) {
    const check = [
      ...(entry.lead ? straight(entry.lead[0], entry.lead[1], CHECK_STEP_M) : []),
      ...sampleDubins(word, CHECK_STEP_M),
      ...(exit.tail ? straight(exit.tail[0], exit.tail[1], CHECK_STEP_M) : [])
    ];
    if (!driveArea.containsAll(check)) continue;
    const points = [];
    if (entry.lead) points.push(entry.lead[1]);
    points.push(...sampleDubins(word, OUTPUT_STEP_M).slice(1));
    if (exit.tail) points.push(exit.tail[1]);
    return { entryArc: entry.arc, exitArc: exit.arc, points };
  }
  return wideCorner(ring, a, b, entries, exits, radius, driveArea);
}

// A corner the lap wraps round (an obstacle corner on the inside of the
// turn) closer than one turning radius cannot be followed: any smooth turn
// tangent to both sides cuts into the obstacle, and the only one-manoeuvre
// alternative is a loop. Swing wide instead — through an apex pose pulled
// out from the corner, so the turn there has its full radius — as two hops.
// Going round the corner may take one long arc (it is the ring round the
// obstacle, not a U-turn), but never a loop.
function wideCorner(ring, a, b, entries, exits, radius, driveArea) {
  const mid = ring.at((a + b) / 2);
  const tIn = ring.at(a - 1e-3).th;
  const tOut = ring.at(b + 1e-3).th;
  const turnSign = Math.sign(wrapAngle(tOut - tIn)) || 1;
  const outward = mid.th - turnSign * Math.PI / 2; // away from the inside of the turn
  const plainEntries = entries.filter((entry) => !entry.lead && entry.penalty > 0);
  const plainExits = exits.filter((exit) => !exit.tail && exit.penalty > 0);

  const candidates = [];
  WIDE_CORNER_OFFSETS.forEach((k) => {
    const apex = {
      x: mid.p[0] + k * radius * Math.cos(outward),
      y: mid.p[1] + k * radius * Math.sin(outward),
      th: mid.th
    };
    if (!driveArea.contains(apex.x, apex.y)) return;
    plainEntries.forEach((entry) => {
      const first = smoothWords(entry.pose, apex, radius, { wrapping: true })[0];
      if (!first) return;
      plainExits.forEach((exit) => {
        const second = smoothWords(apex, exit.pose, radius, { wrapping: true })[0];
        if (!second || !chainIsSmooth([first, second], { wrapping: true })) return;
        candidates.push({ entry, exit, first, second, score: first.length + second.length + 0.5 * (entry.penalty + exit.penalty) });
      });
    });
  });
  candidates.sort((x, y) => x.score - y.score);

  for (const { entry, exit, first, second } of candidates.slice(0, MAX_MANOEUVRE_CHECKS)) {
    if (!driveArea.containsAll(sampleDubins(first, CHECK_STEP_M)) || !driveArea.containsAll(sampleDubins(second, CHECK_STEP_M))) continue;
    const points = [...sampleDubins(first, OUTPUT_STEP_M).slice(1), ...sampleDubins(second, OUTPUT_STEP_M).slice(1)];
    return { entryArc: entry.arc, exitArc: exit.arc, points };
  }
  return null;
}

// Turns one offset ring into drivable lap paths: [{ closed, points }].
function smoothLap(rawRing, radius, driveArea) {
  if (rawRing.length < 3) return [];
  const n0 = rawRing.length;
  // Rings arrive resampled at an even RING_SAMPLE_STEP_M (see smoothRing),
  // so the per-vertex turn is a fair curvature measure: GPS-scale wiggles
  // stay below it, any bend the vehicle cannot follow exceeds it.
  const bends = rawRing.map((v, i) => {
    const p = rawRing[(i - 1 + n0) % n0];
    const q = rawRing[(i + 1) % n0];
    const turn = Math.abs(wrapAngle(headingOf(v, q) - headingOf(p, v)));
    const meanLen = (Math.hypot(v[0] - p[0], v[1] - p[1]) + Math.hypot(q[0] - v[0], q[1] - v[1])) / 2;
    return turn > MAX_PLAIN_VERTEX_TURN_RAD || turn / Math.max(meanLen, 1e-6) > TIGHT_CURVATURE_FACTOR / radius;
  });
  // Widen each tight run by a vertex either side: the vertex where an arc
  // meets a straight side still carries part of the bend, and the manoeuvre
  // must start from a straight heading, not from that kink.
  const tight0 = bends.map((b, i) => b || bends[(i - 1 + n0) % n0] || bends[(i + 1) % n0]);
  if (tight0.every(Boolean)) return []; // e.g. a circle smaller than the turning circle
  if (!tight0.some(Boolean)) return [{ closed: true, points: dropCollinear(rawRing, true) }];

  // Start the ring in the middle of its longest easy stretch, so no tight
  // stretch wraps across the start.
  let bestStart = 0;
  let bestLen = -1;
  for (let i = 0; i < n0; i += 1) {
    if (tight0[i] || !tight0[(i - 1 + n0) % n0]) continue;
    let j = i;
    let len = 0;
    while (!tight0[j % n0] && j < i + n0) {
      const a = rawRing[j % n0];
      const b = rawRing[(j + 1) % n0];
      len += Math.hypot(b[0] - a[0], b[1] - a[1]);
      j += 1;
    }
    if (len > bestLen) {
      bestLen = len;
      bestStart = (i + Math.floor((j - i) / 2)) % n0;
    }
  }
  const points = [...rawRing.slice(bestStart), ...rawRing.slice(0, bestStart)];
  const tight = [...tight0.slice(bestStart), ...tight0.slice(0, bestStart)];
  const ring = ringGeometry(points);

  // Runs of consecutive tight vertices, as [startArc, endArc].
  const stretches = [];
  for (let i = 0; i < ring.n; i += 1) {
    if (!tight[i]) continue;
    if (i > 0 && tight[i - 1]) stretches[stretches.length - 1][1] = ring.starts[i];
    else stretches.push([ring.starts[i], ring.starts[i]]);
  }

  // Manoeuvre for stretches k..m (merged), entering after `from`. It may use
  // only half the easy ring before the next stretch, leaving the rest for
  // that stretch's own run-up — unless `m` is the last stretch.
  const attempt = (k, from) => {
    const a = stretches[k][0];
    for (let m = k; m < Math.min(stretches.length, k + 1 + MERGE_ATTEMPTS); m += 1) {
      const b = stretches[m][1];
      const roomFwd = m + 1 < stretches.length ? (stretches[m + 1][0] - b) / 2 : ring.total - b;
      const found = bestManoeuvre(ring, a, b, a - from, roomFwd, radius, driveArea);
      if (found) return { found, last: m };
    }
    return null;
  };

  const pieces = [];
  let current = [ring.at(0).p];
  let cursor = 0;
  let previous = null; // last manoeuvre, kept so a failing next stretch can merge back into it
  let k = 0;
  while (k < stretches.length) {
    let done = attempt(k, cursor);
    if (!done && previous && previous.last === k - 1) {
      // The previous manoeuvre left too little run-up: redo it merged with
      // this stretch as one manoeuvre.
      const merged = attempt(previous.first, previous.from);
      if (merged && merged.last >= k) {
        current.length = previous.length;
        cursor = previous.from;
        done = merged;
        k = previous.first;
      }
    }
    if (done) {
      previous = { first: k, last: done.last, from: cursor, length: current.length };
      current.push(...ring.verticesBetween(cursor, done.found.entryArc), ring.at(done.found.entryArc).p, ...done.found.points);
      cursor = done.found.exitArc;
      k = done.last + 1;
      continue;
    }
    // Nothing drivable gets round this stretch: end the piece before it and
    // start a new one after it.
    const a = stretches[k][0];
    current.push(...ring.verticesBetween(cursor, a), ring.at(a).p);
    pieces.push(current);
    cursor = stretches[k][1];
    current = [ring.at(cursor).p];
    previous = null;
    k += 1;
  }
  current.push(...ring.verticesBetween(cursor, ring.total), ring.at(0).p);

  if (pieces.length === 0) {
    current.pop(); // closing duplicate of the start
    return [{ closed: true, points: dropCollinear(current, true) }];
  }
  // The last piece runs through the ring start into the first one.
  pieces[0] = [...current, ...pieces[0].slice(1)];
  const pathLength = (pts) => pts.reduce((sum, p, i) => (i ? sum + Math.hypot(p[0] - pts[i - 1][0], p[1] - pts[i - 1][1]) : 0), 0);
  return pieces
    .filter((pts) => pts.length >= 2 && pathLength(pts) >= radius)
    .map((pts) => ({ closed: false, points: dropCollinear(pts, false) }));
}

// Final guard on a finished lap: back-to-back corner manoeuvres with too
// little straight between them can still chain into one tight arc of a
// semicircle or more. Cut each such arc out and split the lap there. Only
// tight arcs count — a wide orbit round an obstacle is a ring, not a
// semicircle.
// Only arcs this tight (× 1/R) are corner manoeuvres; anything gentler is
// the ring following the boundary or orbiting an obstacle.
const TIGHT_ARC_CURVATURE_FACTOR = 0.8;
function splitAtTightArcs(lap, radius) {
  const pts = lap.points;
  const n = pts.length;
  if (n < 3) return [lap];
  const cuts = []; // [startIndex, endIndex] of each offending arc
  let groupStart = -1;
  let group = 0;
  let sign = 0;
  let straight = 0;
  const close = (end) => {
    if (groupStart >= 0 && Math.abs(group) > MAX_LAP_ARC_RAD) cuts.push([groupStart, end]);
    groupStart = -1;
    group = 0;
    sign = 0;
  };
  const last = lap.closed ? n : n - 1;
  for (let i = 1; i < last; i += 1) {
    const a = pts[i - 1];
    const b = pts[i % n];
    const c = pts[(i + 1) % n];
    const len = Math.hypot(c[0] - b[0], c[1] - b[1]);
    const turn = wrapAngle(headingOf(b, c) - headingOf(a, b));
    if (Math.abs(turn) < 1e-4) {
      straight += len;
      if (straight >= STRAIGHT_BREAK_FACTOR * radius) close(i);
      continue;
    }
    straight = 0;
    const prevLen = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const tight = Math.abs(turn) / Math.max((len + prevLen) / 2, 1e-6) >= TIGHT_ARC_CURVATURE_FACTOR / radius;
    if (!tight || (sign !== 0 && Math.sign(turn) !== sign)) close(i);
    if (!tight) continue;
    if (groupStart < 0) groupStart = i - 1;
    group += turn;
    sign = Math.sign(turn);
  }
  close(last);
  if (cuts.length === 0) return [lap];

  // Keep everything outside the cut arcs, as open pieces (a closed lap's
  // last piece runs on into its first).
  const pieces = [];
  let from = 0;
  cuts.forEach(([start, end]) => {
    pieces.push(pts.slice(from, start + 1));
    from = end;
  });
  const tail = pts.slice(from);
  if (lap.closed && pieces.length) pieces[0] = [...tail, ...pieces[0]];
  else pieces.push(tail);
  const length = (p) => p.reduce((sum, q, i) => (i ? sum + Math.hypot(q[0] - p[i - 1][0], q[1] - p[i - 1][1]) : 0), 0);
  return pieces.filter((p) => p.length >= 2 && length(p) >= radius).map((p) => ({ closed: false, points: p }));
}

// An open piece must start and end on a straight: it was cut where its ring
// got too tight, so an end can sit part-way round a corner arc, and the turn
// connecting to it would carry that arc on into a loop. Trims the curved
// tail off each end.
function trimCurvedEnds(lap, radius) {
  if (lap.closed) return [lap];
  // Walks back from the end: tight arcs and the short straights between
  // them are dropped until a straight of at least the flat-U-turn length, or
  // a gentle curve, is reached.
  const trimEnd = (pts) => {
    const n = pts.length;
    const seg = (i) => Math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]);
    const turnAt = (i) => Math.abs(wrapAngle(headingOf(pts[i], pts[i + 1]) - headingOf(pts[i - 1], pts[i])));
    let cutAt = null;
    let straight = 0;
    for (let i = n - 2; i >= 1; i -= 1) {
      const turn = turnAt(i);
      if (turn < 1e-4) {
        straight += seg(i);
        if (straight >= STRAIGHT_BREAK_FACTOR * radius) break;
        continue;
      }
      if (turn / Math.max((seg(i - 1) + seg(i)) / 2, 1e-6) < TIGHT_ARC_CURVATURE_FACTOR / radius) break; // gentle curve
      cutAt = i;
      straight = 0;
    }
    return cutAt === null ? pts : pts.slice(0, cutAt + 1);
  };
  const trimmed = trimEnd(trimEnd(lap.points).reverse()).reverse();
  const length = trimmed.reduce((sum, q, i) => (i ? sum + Math.hypot(q[0] - trimmed[i - 1][0], q[1] - trimmed[i - 1][1]) : 0), 0);
  return trimmed.length >= 2 && length >= radius ? [{ closed: false, points: trimmed }] : [];
}

// Chaikin corner cutting on a closed ring: rings offset from a recorded
// (GPS-jittery) boundary are full of small kinks a metre or two apart, each
// of which would otherwise need its own corner manoeuvre with no room to fit
// one. Each pass moves the ring only within its own corners; the result is
// kept only if it stays inside the drive area.
function chaikin(ring, passes) {
  let pts = ring;
  for (let pass = 0; pass < passes; pass += 1) {
    const out = [];
    pts.forEach((a, i) => {
      const b = pts[(i + 1) % pts.length];
      out.push([0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1]]);
      out.push([0.25 * a[0] + 0.75 * b[0], 0.25 * a[1] + 0.75 * b[1]]);
    });
    pts = out;
  }
  return pts;
}

// Evenly spaced points along a closed ring.
function resampleRing(ring, step) {
  const geometry = ringGeometry(ring);
  const count = Math.max(3, Math.round(geometry.total / step));
  return Array.from({ length: count }, (_, i) => geometry.at((i * geometry.total) / count).p);
}

function smoothRing(ring, driveArea) {
  const smoothed = chaikin(ring, RING_SMOOTHING_PASSES);
  return resampleRing(driveArea.containsAll(smoothed) ? smoothed : ring, RING_SAMPLE_STEP_M);
}

// Every lap (or lap piece) carries `depth`, its ring's index counted in from
// the boundary/obstacle, so the router can mow the laps ring by ring.
function generateHeadlandLaps(options) {
  const { turningRadius, driveArea } = options;
  return generateLapRings(options).flatMap(({ depth, points }) =>
    smoothLap(smoothRing(points, driveArea), turningRadius, driveArea)
      .flatMap((lap) => splitAtTightArcs(lap, turningRadius))
      .flatMap((lap) => trimCurvedEnds(lap, turningRadius))
      .map((lap) => ({ ...lap, depth })));
}

module.exports = { generateHeadlandLaps };
