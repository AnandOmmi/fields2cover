const assert = require('assert');
const { computeF2cMowerPath } = require('..');

// ~100 m x 60 m rectangle near 0,0; rows run east-west.
const polygonCoords = [[0, 0], [0.0009, 0], [0.0009, 0.00054], [0, 0.00054], [0, 0]];
const out = computeF2cMowerPath({
  polygonCoords,
  referenceLineCoords: [[0, 0], [0.0009, 0]],
  offsetDistance: 3,
  spacing: 1,
  turningRadius: 2
});

assert.strictEqual(out.status, 200, JSON.stringify(out));
const path = out.result.features[0];
assert.strictEqual(path.geometry.type, 'LineString');
assert.ok(path.geometry.coordinates.length > 10);
console.log('ok', path.properties);

// Self-intersecting (bow-tie) obstacle: the path must still be generated and
// never pass through the area the outline encloses.
const turf = require('@turf/turf');
const bowTie = [[0.0004, 0.0002], [0.0005, 0.0003], [0.0005, 0.0002], [0.0004, 0.0003], [0.0004, 0.0002]];
const withObstacle = computeF2cMowerPath({
  polygonCoords,
  referenceLineCoords: [[0, 0], [0.0009, 0.0002]],
  offsetDistance: 3,
  spacing: 1,
  turningRadius: 2,
  obstacleAvoid: true,
  obstaclePolygons: [bowTie]
});
assert.strictEqual(withObstacle.status, 200, JSON.stringify(withObstacle));
const { normalizeObstacleRings } = require('../lib/obstacleRings');
const obstacleAreas = normalizeObstacleRings([bowTie]).map((ring) => turf.polygon([ring]));
const inside = withObstacle.result.features[0].geometry.coordinates
  .filter((p) => obstacleAreas.some((area) => turf.booleanPointInPolygon(turf.point(p), area)));
assert.strictEqual(inside.length, 0, `${inside.length} path points inside the obstacle`);
console.log('ok self-intersecting obstacle avoided');
