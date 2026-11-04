const turf = require('@turf/turf');

// Port of Fields2Cover's hg::ConstHL (constant-width headland generator):
// the field is the boundary with obstacles as holes, and the "mainland" where
// swaths go is that field shrunk by the headland width on every side —
// including around every obstacle, so each obstacle gets its own turning
// ring. Polygon offsetting stays in lon/lat through turf.buffer (geodesic,
// handles holes and merging); everything downstream works in local metres.

// Boundaries/obstacles drawn from a dense recorded path (thousands of
// GPS-jitter-close points) make turf's buffer/clip math numerically
// unreliable. Simplifying to ~0.3 m first removes that noise without visibly
// changing the shape (well under 0.1% area change in practice).
const RING_SIMPLIFY_TOLERANCE_DEG = 0.000003; // ~0.3m
function simplifyRing(coords) {
  if (!Array.isArray(coords) || coords.length <= 20) return coords;
  const simplified = turf.simplify(turf.polygon([coords]), { tolerance: RING_SIMPLIFY_TOLERANCE_DEG, highQuality: true });
  const simplifiedRing = simplified.geometry.coordinates[0];
  return simplifiedRing.length >= 4 ? simplifiedRing : coords;
}

const bufferMeters = (feature, meters) => turf.buffer(feature, meters / 1000, { units: "kilometers" });

const statusError = (message, status = 400) => Object.assign(new Error(message), { status });

// Boundary shrunk by `boundaryInset` minus every obstacle grown by
// `obstacleGrow` (both metres). Returns a lon/lat Polygon/MultiPolygon.
function fieldArea(polygonCoords, obstacleRings, boundaryInset, obstacleGrow) {
  const boundary = turf.polygon([simplifyRing(polygonCoords)]);
  const area = boundaryInset > 0 ? bufferMeters(boundary, -boundaryInset) : boundary;
  if (!area) throw statusError("Boundary offset leaves no driveable area");

  const grown = (obstacleRings || [])
    .filter((coords) => Array.isArray(coords) && coords.length >= 4)
    .map((coords) => bufferMeters(turf.polygon([simplifyRing(coords)]), obstacleGrow))
    .filter(Boolean);
  if (grown.length === 0) return area;

  const obstacleUnion = grown.length === 1 ? grown[0] : turf.union(turf.featureCollection(grown));
  if (!obstacleUnion) return area;
  const diff = turf.difference(turf.featureCollection([area, obstacleUnion]));
  if (!diff) throw statusError("Obstacles fully cover the driveable area — nothing left to mow");
  return diff;
}

// Mainland for swaths: the constant-width headland on every side.
function generateMainland(polygonCoords, obstacleRings, headlandWidth) {
  return fieldArea(polygonCoords, obstacleRings, headlandWidth, headlandWidth);
}

module.exports = { fieldArea, generateMainland, bufferMeters, simplifyRing };
