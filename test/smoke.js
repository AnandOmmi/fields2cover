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
