// Planar geometry in the local metre frame (see frame.js). An "area" is a set
// of polygons-with-holes, [[outerRing, ...holeRings], ...], each ring an array
// of [x, y] (closed or not) — the shape GeoJSON Polygon/MultiPolygon already
// uses, just projected to metres.

const VISIBILITY_SAMPLE_STEP_M = 2;

function ringContains(ring, x, y) {
  let inside = false;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i, i += 1) {
    const [xi, yi] = ring[i];
    const [xj, yj] = ring[j];
    if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

function ringBounds(ring) {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  ring.forEach(([x, y]) => {
    if (x < minX) minX = x;
    if (y < minY) minY = y;
    if (x > maxX) maxX = x;
    if (y > maxY) maxY = y;
  });
  return { minX, minY, maxX, maxY };
}

const orient = (ax, ay, bx, by, cx, cy) => (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);

// Uniform grid over the boundary edges, so containment and visibility checks
// look only at the edges near the query instead of every edge of the
// boundary and every obstacle — those checks are most of the planning time
// once there are obstacles.
const GRID_TARGET_CELLS = 250000;
const GRID_MIN_CELL_M = 1;
const GRID_EPS = 1e-6;
function createEdgeGrid(edges) {
  if (edges.length < 64) return null; // a plain scan is as fast
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  edges.forEach((e) => {
    minX = Math.min(minX, e.minX);
    minY = Math.min(minY, e.minY);
    maxX = Math.max(maxX, e.maxX);
    maxY = Math.max(maxY, e.maxY);
  });
  const size = Math.max(GRID_MIN_CELL_M, Math.sqrt(((maxX - minX) * (maxY - minY)) / GRID_TARGET_CELLS));
  const cols = Math.floor((maxX - minX) / size) + 1;
  const rows = Math.floor((maxY - minY) / size) + 1;
  const col = (x) => Math.floor((x - minX) / size);
  const row = (y) => Math.floor((y - minY) / size);
  const clampCol = (c) => Math.min(cols - 1, Math.max(0, c));
  const clampRow = (r) => Math.min(rows - 1, Math.max(0, r));

  const cells = new Array(cols * rows).fill(null);
  edges.forEach((e, index) => {
    const c0 = clampCol(col(e.minX - GRID_EPS));
    const c1 = clampCol(col(e.maxX + GRID_EPS));
    const r0 = clampRow(row(e.minY - GRID_EPS));
    const r1 = clampRow(row(e.maxY + GRID_EPS));
    for (let r = r0; r <= r1; r += 1) {
      for (let c = c0; c <= c1; c += 1) {
        const k = r * cols + c;
        (cells[k] || (cells[k] = [])).push(index);
      }
    }
  });

  const stamps = new Int32Array(edges.length);
  let stampCounter = 0;
  const nextStamp = () => {
    stampCounter += 1;
    if (stampCounter === 2147483647) {
      stamps.fill(0);
      stampCounter = 1;
    }
    return stampCounter;
  };
  const uniform = new Int8Array(cols * rows); // 0 unknown, 1 inside, 2 outside
  const uniformState = (k, containsSlow) => {
    if (!uniform[k]) {
      const c = k % cols;
      const r = (k - c) / cols;
      uniform[k] = containsSlow(minX + (c + 0.5) * size, minY + (r + 0.5) * size) ? 1 : 2;
    }
    return uniform[k] === 1;
  };

  // Edges in the cells the segment a→b passes through (each once), walking
  // the grid along the segment; the segment's ends are clamped onto the grid,
  // as nothing outside it has edges.
  const edgesNear = (ax, ay, bx, by) => {
    const out = [];
    const stamp = nextStamp();
    const visit = (c, r) => {
      if (c < 0 || r < 0 || c >= cols || r >= rows) return;
      const cell = cells[r * cols + c];
      if (!cell) return;
      for (const index of cell) {
        if (stamps[index] === stamp) continue;
        stamps[index] = stamp;
        out.push(edges[index]);
      }
    };
    let c = col(ax);
    let r = row(ay);
    const cEnd = col(bx);
    const rEnd = row(by);
    const dx = bx - ax;
    const dy = by - ay;
    const stepC = dx > 0 ? 1 : -1;
    const stepR = dy > 0 ? 1 : -1;
    const tDeltaC = dx !== 0 ? size / Math.abs(dx) : Infinity;
    const tDeltaR = dy !== 0 ? size / Math.abs(dy) : Infinity;
    let tMaxC = dx !== 0 ? ((minX + (c + (dx > 0 ? 1 : 0)) * size) - ax) / dx : Infinity;
    let tMaxR = dy !== 0 ? ((minY + (r + (dy > 0 ? 1 : 0)) * size) - ay) / dy : Infinity;
    const limit = Math.abs(cEnd - c) + Math.abs(rEnd - r) + 2;
    for (let n = 0; n <= limit; n += 1) {
      // Edges sit in every cell within GRID_EPS of them, so a walk rounded
      // past a cell corner still meets any edge the segment crosses there.
      visit(c, r);
      if (c === cEnd && r === rEnd) break;
      if (tMaxC < tMaxR) {
        c += stepC;
        tMaxC += tDeltaC;
      } else {
        r += stepR;
        tMaxR += tDeltaR;
      }
    }
    return out;
  };

  return { cols, rows, col, row, cells, stamps, nextStamp, uniformState, edgesNear };
}

function createArea(polygons) {
  const parts = polygons
    .filter((rings) => rings.length && rings[0].length >= 3)
    .map((rings) => ({ rings, bounds: ringBounds(rings[0]), holes: rings.slice(1).map((ring) => ({ ring, bounds: ringBounds(ring) })) }));

  const edges = [];
  parts.forEach(({ rings }) => rings.forEach((ring) => {
    for (let i = 0; i < ring.length; i += 1) {
      const [ax, ay] = ring[i];
      const [bx, by] = ring[(i + 1) % ring.length];
      if (ax === bx && ay === by) continue;
      edges.push({
        ax, ay, bx, by,
        minX: Math.min(ax, bx), maxX: Math.max(ax, bx),
        minY: Math.min(ay, by), maxY: Math.max(ay, by)
      });
    }
  }));

  const inBounds = (b, x, y) => x >= b.minX && x <= b.maxX && y >= b.minY && y <= b.maxY;
  const containsSlow = (x, y) => parts.some(({ rings, bounds, holes }) =>
    inBounds(bounds, x, y) && ringContains(rings[0], x, y) &&
    !holes.some((hole) => inBounds(hole.bounds, x, y) && ringContains(hole.ring, x, y)));

  const grid = createEdgeGrid(edges);
  // Same answer as containsSlow, from local edges only: a cell no edge
  // touches is wholly inside or wholly outside, so a point is classified by
  // counting the edge crossings of a ray to the next such cell along its row.
  const contains = (x, y) => {
    if (!grid) return containsSlow(x, y);
    const cx = grid.col(x);
    const cy = grid.row(y);
    if (cx < 0 || cy < 0 || cx >= grid.cols || cy >= grid.rows) return containsSlow(x, y);
    let inside = false;
    const stamp = grid.nextStamp();
    for (let c = cx; c < grid.cols; c += 1) {
      const cell = grid.cells[cy * grid.cols + c];
      if (!cell) {
        if (c === cx) return grid.uniformState(cy * grid.cols + c, containsSlow);
        return inside !== grid.uniformState(cy * grid.cols + c, containsSlow);
      }
      for (const index of cell) {
        if (grid.stamps[index] === stamp) continue;
        grid.stamps[index] = stamp;
        const e = edges[index];
        if ((e.ay > y) !== (e.by > y) && x < ((e.bx - e.ax) * (y - e.ay)) / (e.by - e.ay) + e.ax) inside = !inside;
      }
    }
    return inside; // ray left the grid, and nothing lies beyond it
  };

  return {
    polygons: parts.map(({ rings }) => rings),
    edges,
    contains,
    containsAll: (points) => points.every(([x, y]) => contains(x, y)),
    // True when the straight segment a→b stays inside the area: it properly
    // crosses no boundary edge and its interior samples are all inside
    // (catches the rare pass exactly through a vertex).
    segmentClear([ax, ay], [bx, by]) {
      const minX = Math.min(ax, bx);
      const maxX = Math.max(ax, bx);
      const minY = Math.min(ay, by);
      const maxY = Math.max(ay, by);
      for (const e of grid ? grid.edgesNear(ax, ay, bx, by) : edges) {
        if (e.maxX < minX || e.minX > maxX || e.maxY < minY || e.minY > maxY) continue;
        const o1 = orient(ax, ay, bx, by, e.ax, e.ay);
        const o2 = orient(ax, ay, bx, by, e.bx, e.by);
        const o3 = orient(e.ax, e.ay, e.bx, e.by, ax, ay);
        const o4 = orient(e.ax, e.ay, e.bx, e.by, bx, by);
        if (o1 * o2 < 0 && o3 * o4 < 0) return false;
      }
      const length = Math.hypot(bx - ax, by - ay);
      const count = Math.max(2, Math.ceil(length / VISIBILITY_SAMPLE_STEP_M));
      for (let i = 1; i < count; i += 1) {
        const t = i / count;
        if (!contains(ax + (bx - ax) * t, ay + (by - ay) * t)) return false;
      }
      return true;
    }
  };
}

// GeoJSON Polygon/MultiPolygon (Feature or geometry) → area in local metres.
function areaFromGeoJSON(feature, toLocal) {
  const geometry = feature?.geometry || feature;
  if (!geometry) return createArea([]);
  const polygons = geometry.type === "Polygon" ? [geometry.coordinates]
    : geometry.type === "MultiPolygon" ? geometry.coordinates : [];
  return createArea(polygons.map((rings) => rings.map((ring) => ring.map(toLocal))));
}

function signedRingArea(ring) {
  let sum = 0;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i, i += 1) {
    sum += (ring[j][0] - ring[i][0]) * (ring[j][1] + ring[i][1]);
  }
  return sum / 2;
}

const openRing = (ring) => {
  const last = ring[ring.length - 1];
  return ring.length > 1 && last[0] === ring[0][0] && last[1] === ring[0][1] ? ring.slice(0, -1) : ring;
};

// Vertices where the free space bends around a corner (its interior angle is
// over 180°): convex corners of obstacles and inward notches of the outer
// boundary. A shortest path inside a polygon only ever bends at these.
function freeSpaceReflexVertices(area) {
  const out = [];
  area.polygons.forEach((rings) => rings.forEach((rawRing, ringIndex) => {
    let ring = openRing(rawRing);
    if (ring.length < 3) return;
    // Orient so free space is on the left: outer ring CCW, holes CW.
    const ccw = signedRingArea(ring) > 0;
    if ((ringIndex === 0) !== ccw) ring = [...ring].reverse();
    for (let i = 0; i < ring.length; i += 1) {
      const p = ring[(i - 1 + ring.length) % ring.length];
      const v = ring[i];
      const n = ring[(i + 1) % ring.length];
      if (orient(p[0], p[1], v[0], v[1], n[0], n[1]) < 0) out.push(v);
    }
  }));
  return out;
}

module.exports = { createArea, areaFromGeoJSON, freeSpaceReflexVertices };
