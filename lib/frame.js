// Local east/north metre frame around an origin (equirectangular with WGS84
// metre-per-degree terms). Accurate to well under a centimetre across a
// field-sized area and cheap enough to run on every sampled path point, so
// all planning (swaths, decomposition, Dubins turns, safety checks) happens
// in metres instead of mixing degrees with a metre-based turning radius.
function createLocalFrame(origin) {
  const lat0 = origin[1] * Math.PI / 180;
  const metersPerDegLat = 111132.92 - 559.82 * Math.cos(2 * lat0) + 1.175 * Math.cos(4 * lat0);
  const metersPerDegLon = 111412.84 * Math.cos(lat0) - 93.5 * Math.cos(3 * lat0);
  return {
    toLocal: ([lon, lat]) => [(lon - origin[0]) * metersPerDegLon, (lat - origin[1]) * metersPerDegLat],
    toLonLat: ([x, y]) => [origin[0] + x / metersPerDegLon, origin[1] + y / metersPerDegLat]
  };
}

// Swath frame (u, v): v runs along the swath direction, u across it (to the
// right of the direction). Swaths are then just vertical lines u = const.
function createSwathFrame(direction) {
  const [dx, dy] = direction;
  const nx = dy;
  const ny = -dx;
  return {
    heading: Math.atan2(dy, dx),
    toUV: ([x, y]) => [x * nx + y * ny, x * dx + y * dy],
    fromUV: ([u, v]) => [u * nx + v * dx, u * ny + v * dy]
  };
}

module.exports = { createLocalFrame, createSwathFrame };
