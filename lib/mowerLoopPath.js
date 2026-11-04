const METERS_PER_DEGREE_LATITUDE = 110540;
const METERS_PER_DEGREE_LONGITUDE = 111320;
const DEFAULT_POINT_SPACING = 0.1;
const MIN_ROW_LENGTH = 3;
const MAX_END_ROW_SKEW = 5 * Math.PI / 180; // last row must be this close to parallel with the first row

const distance = (a, b) => Math.hypot(b[0] - a[0], b[1] - a[1]);

const line = (from, to, step) => {
  const length = distance(from, to);
  if (length < 1e-6) return [];
  const count = Math.max(Math.ceil(length / step), 1);
  return Array.from({ length: count }, (_, index) => {
    const t = (index + 1) / count;
    return [from[0] + (to[0] - from[0]) * t, from[1] + (to[1] - from[1]) * t];
  });
};

const arc = (center, radius, startAngle, endAngle, step) => {
  const count = Math.max(Math.ceil(Math.abs(endAngle - startAngle) * radius / step), 1);
  return Array.from({ length: count }, (_, index) => {
    const angle = startAngle + ((index + 1) / count) * (endAngle - startAngle);
    return [center[0] + radius * Math.cos(angle), center[1] + radius * Math.sin(angle)];
  });
};

const concatenate = (...parts) => parts.flat();

// These primitives intentionally mirror generate_loop_path.py. They operate
// in a local metre frame, which avoids mixing longitude/latitude degrees with
// the mower's metre-based radius and spacing settings.
const uTurnRightTop = (fromX, fromY, toX, toY, radius, step) => {
  if (toX - fromX < 2 * radius - 1e-6) {
    throw new Error("The return row is too close to make the loop turn.");
  }
  const top = Math.max(fromY, toY) + radius;
  return concatenate(
    line([fromX, fromY], [fromX, top - radius], step),
    arc([fromX + radius, top - radius], radius, Math.PI, Math.PI / 2, step),
    line([fromX + radius, top], [toX - radius, top], step),
    arc([toX - radius, top - radius], radius, Math.PI / 2, 0, step),
    line([toX, top - radius], [toX, toY], step)
  );
};

const uTurnBottom = (fromX, fromY, toX, toY, radius, step) => {
  const direction = toX > fromX ? 1 : -1;
  if (Math.abs(toX - fromX) < 2 * radius - 1e-6) {
    throw new Error("The start and end rows are too close to make the loop turn.");
  }
  const bottom = Math.min(fromY, toY) - radius;
  const startAngle = direction > 0 ? Math.PI : 0;
  return concatenate(
    line([fromX, fromY], [fromX, bottom + radius], step),
    arc([fromX + direction * radius, bottom + radius], radius,
      startAngle, startAngle + direction * Math.PI / 2, step),
    line([fromX + direction * radius, bottom], [toX - direction * radius, bottom], step),
    arc([toX - direction * radius, bottom + radius], radius,
      startAngle + direction * Math.PI / 2, startAngle + direction * Math.PI, step),
    line([toX, bottom + radius], [toX, toY], step)
  );
};

const createFrame = (origin, rowHeading) => {
  const scaleX = METERS_PER_DEGREE_LONGITUDE * Math.cos(origin[1] * Math.PI / 180);
  const theta = Math.PI / 2 - rowHeading;
  const cos = Math.cos(theta);
  const sin = Math.sin(theta);
  return {
    toLocal([longitude, latitude]) {
      const east = (longitude - origin[0]) * scaleX;
      const north = (latitude - origin[1]) * METERS_PER_DEGREE_LATITUDE;
      return [east * cos - north * sin, east * sin + north * cos];
    },
    toLonLat([x, y]) {
      const east = x * cos + y * sin;
      const north = -x * sin + y * cos;
      return [east / scaleX + origin[0], north / METERS_PER_DEGREE_LATITUDE + origin[1]];
    }
  };
};

const rowSegments = (path) => path
  .slice(0, -1)
  .map((start, index) => ({ index, start, end: path[index + 1] }))
  .filter(({ start, end }) => distance(start, end) > MIN_ROW_LENGTH);

const buildConnector = (path, radius, step, gap) => {
  const start = path[0];
  const end = path[path.length - 1];
  const goal = [start[0], start[1] - gap];

  // Both U-turns below leave the end row straight along the local y axis, so
  // they only fit a path that ends on a row parallel to the first one. A
  // criss-cross path often ends on a cross row (90 degrees to the first row):
  // the template would leave it with a sharp corner, so let the caller use
  // the Dubins fallback instead.
  const before = path[path.length - 2];
  const endDx = end[0] - before[0];
  const endDy = end[1] - before[1];
  if (Math.abs(endDx) > Math.abs(endDy) * Math.tan(MAX_END_ROW_SKEW)) {
    throw new Error("The last row is not parallel to the first row.");
  }

  if (end[1] < before[1]) {
    return {
      points: uTurnBottom(end[0], end[1], goal[0], goal[1], radius, step),
      info: { returnRowX: null, returnRowIndex: null }
    };
  }

  const rows = rowSegments(path);
  const minimumReturnX = end[0] + 2 * radius;
  const medianTop = median(rows.map((candidate) => Math.max(candidate.start[1], candidate.end[1])));
  const candidates = rows
    .map((row) => ({ ...row, x: (row.start[0] + row.end[0]) / 2, top: Math.max(row.start[1], row.end[1]) }))
    .filter((row) => row.x >= minimumReturnX && row.top >= medianTop - 1.5)
    .sort((a, b) => a.x - b.x);

  if (!candidates.length) {
    throw new Error("No return row is available to join the mower path.");
  }
  const returnRow = candidates[0];
  const topJoin = Math.min(end[1], returnRow.top);
  const connector = concatenate(
    uTurnRightTop(end[0], end[1], returnRow.x, topJoin, radius, step),
    uTurnBottom(returnRow.x, topJoin, goal[0], goal[1], radius, step)
  );
  connector[connector.length - 1] = goal;
  return {
    points: connector,
    info: { returnRowX: returnRow.x, returnRowIndex: returnRow.index }
  };
};

const median = (values) => {
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.floor(sorted.length / 2)];
};

/**
 * Appends a forward-driving connector from a generated mower path's end to
 * its start. The implementation follows generate_loop_path.py's local-frame,
 * arc-straight-arc construction and returns a GeoJSON-ready feature.
 */
function closeMowerPath(coordinates, { turningRadius, gap = 0, spacing = DEFAULT_POINT_SPACING } = {}) {
  if (!Array.isArray(coordinates) || coordinates.length < 3) {
    throw new Error("A generated mower path needs at least three points to be closed.");
  }
  if (!Number.isFinite(turningRadius) || turningRadius <= 0) {
    throw new Error("A positive turning radius is required to close the mower path.");
  }
  const first = coordinates[0];
  const second = coordinates[1];
  const rowHeading = Math.atan2(
    (second[1] - first[1]) * METERS_PER_DEGREE_LATITUDE,
    (second[0] - first[0]) * METERS_PER_DEGREE_LONGITUDE * Math.cos(first[1] * Math.PI / 180)
  );
  const frame = createFrame(first, rowHeading);
  const localPath = coordinates.map(frame.toLocal);
  const { points, info } = buildConnector(localPath, turningRadius, spacing, gap);
  const connectorCoordinates = points.map(frame.toLonLat);
  const closedCoordinates = [...coordinates, ...connectorCoordinates];
  if (gap <= 0) closedCoordinates[closedCoordinates.length - 1] = [...first];

  return {
    coordinates: closedCoordinates,
    connector: [coordinates[coordinates.length - 1], ...connectorCoordinates],
    properties: {
      join_start_end: true,
      gap_to_start_meters: gap,
      return_row_x_meters: info.returnRowX,
      return_row_index: info.returnRowIndex
    }
  };
}

module.exports = { closeMowerPath };
