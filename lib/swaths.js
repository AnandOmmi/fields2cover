// Port of Fields2Cover's sg (swath generator) for a fixed angle: parallel
// swaths one coverage width apart, clipped to the mainland. The angle comes
// from the user's reference line rather than F2C's brute-force angle search,
// so the operator keeps control of the mowing direction.
//
// Output is per column (one column per swath line u = const), each with the
// inside intervals [v0, v1] where that line crosses the mainland — a column
// has several intervals wherever an obstacle or a concave notch cuts it.
function generateSwathColumns(area, swathFrame, spacing, minLength) {
  const rings = area.polygons.flat().map((ring) => ring.map(swathFrame.toUV));
  let uMin = Infinity;
  let uMax = -Infinity;
  rings.forEach((ring) => ring.forEach(([u]) => {
    if (u < uMin) uMin = u;
    if (u > uMax) uMax = u;
  }));
  const width = uMax - uMin;
  if (!Number.isFinite(width) || width <= 0) return [];

  // Centre the swaths across the mainland so both outer swaths sit the same
  // distance inside its edges (at most half a coverage width).
  const count = Math.max(1, Math.ceil(width / spacing - 1e-9));
  const first = uMin + (width - (count - 1) * spacing) / 2;

  const columns = [];
  for (let c = 0; c < count; c += 1) {
    const u = first + c * spacing;
    const crossings = [];
    rings.forEach((ring) => {
      for (let i = 0, j = ring.length - 1; i < ring.length; j = i, i += 1) {
        const [ua, va] = ring[j];
        const [ub, vb] = ring[i];
        if ((ua <= u) !== (ub <= u)) crossings.push(va + ((u - ua) * (vb - va)) / (ub - ua));
      }
    });
    crossings.sort((a, b) => a - b);
    const intervals = [];
    for (let k = 0; k + 1 < crossings.length; k += 2) {
      if (crossings[k + 1] - crossings[k] >= minLength) intervals.push([crossings[k], crossings[k + 1]]);
    }
    columns.push({ u, intervals });
  }
  return columns;
}

module.exports = { generateSwathColumns };
