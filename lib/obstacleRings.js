const turf = require('@turf/turf');

// Obstacle outlines usually come from a recorded drive around the obstacle
// (mission file LineStrings closed into Polygons on upload). On a long, thin
// obstacle the return leg wanders across the outbound leg, so the ring
// self-intersects. turf.buffer on such an invalid polygon returns garbage: a
// ~136 m x 5 m strip buffered by 0.5 m came back as 11 m² instead of ~700 m²,
// so the connector/transit drive area ignored the obstacle and turns were
// routed straight through it.
//
// Normalise every ring into a valid simple polygon covering everything the
// outline encloses: buffer the outline as a line by a few centimetres (always
// valid, whatever the self-crossings), union the pieces and keep only the
// exterior ring(s) so any enclosed pocket is filled rather than left as a hole.
const OUTLINE_BUFFER_M = 0.05;

function closeRing(coords) {
  const first = coords[0];
  const last = coords[coords.length - 1];
  return first[0] === last[0] && first[1] === last[1] ? coords : [...coords, first];
}

function exteriorRings(geometry) {
  if (!geometry) return [];
  if (geometry.type === 'Polygon') return [geometry.coordinates[0]];
  if (geometry.type === 'MultiPolygon') return geometry.coordinates.map((poly) => poly[0]);
  return [];
}

function normalizeObstacleRing(coords) {
  if (!Array.isArray(coords) || coords.length < 3) return [];
  const ring = closeRing(coords);
  if (ring.length < 4) return [];
  try {
    const outline = turf.buffer(turf.lineString(ring), OUTLINE_BUFFER_M / 1000, { units: 'kilometers' });
    const rings = exteriorRings(outline?.geometry).filter((r) => r.length >= 4);
    if (rings.length > 0) return rings;
  } catch (err) {
    // fall through to the raw ring
  }
  return [ring];
}

// Array of [lng, lat] rings in -> array of valid, closed, hole-free rings out.
function normalizeObstacleRings(rings) {
  if (!Array.isArray(rings)) return [];
  return rings.flatMap(normalizeObstacleRing);
}

module.exports = { normalizeObstacleRings };
