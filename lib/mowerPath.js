const turf = require('@turf/turf');
const { closeMowerPath } = require('./mowerLoopPath');
const { closeMowerPathFallback } = require('./mowerLoopFallback');
const { planCoverage, CONNECTOR_BOUNDARY_CLEARANCE_M, CONNECTOR_OBSTACLE_CLEARANCE_M } = require('./coverage');
const { fieldArea } = require('./headland');
const { createLocalFrame } = require('./frame');

// Request handling for the Fields2Cover-style planner (POST
// /api/generateF2cMowerPath). Same request body and response shape as the
// existing /api/generateMowerPath, which is left untouched — this is a
// separate generator alongside it.

// Keep the join connector's endpoint under 50 cm from the first mowing point,
// same as the existing generator.
const LOOP_GAP_M = 0.3;

// Turns, connections and laps are sampled every 0.1 m while planning, which
// left ~130k points (7 MB+) on a 2 ha field — mostly on straight stretches.
// The output keeps only the points needed to stay within this distance of
// the planned path: a straight is just its two end points, an R = 6 m arc
// keeps a point about every 0.5 m.
const OUTPUT_TOLERANCE_M = 0.005;

// Douglas–Peucker in local metres (iterative, so long paths cannot overflow
// the stack). First and last points are always kept.
function simplifyPath(coordinates, tolerance) {
  if (coordinates.length < 3) return coordinates;
  const frame = createLocalFrame(coordinates[0]);
  const pts = coordinates.map(frame.toLocal);
  const keep = new Uint8Array(pts.length);
  keep[0] = 1;
  keep[pts.length - 1] = 1;
  const stack = [[0, pts.length - 1]];
  while (stack.length) {
    const [first, last] = stack.pop();
    const [ax, ay] = pts[first];
    const dx = pts[last][0] - ax;
    const dy = pts[last][1] - ay;
    const len = Math.hypot(dx, dy);
    let worst = -1;
    let worstDist = tolerance;
    for (let i = first + 1; i < last; i += 1) {
      const px = pts[i][0] - ax;
      const py = pts[i][1] - ay;
      // Distance to the segment (to the end point when they coincide, e.g.
      // a closed lap that starts and ends at the same spot).
      let dist;
      if (len < 1e-9) {
        dist = Math.hypot(px, py);
      } else {
        const t = Math.max(0, Math.min(1, (px * dx + py * dy) / (len * len)));
        dist = Math.hypot(px - t * dx, py - t * dy);
      }
      if (dist > worstDist) {
        worstDist = dist;
        worst = i;
      }
    }
    if (worst >= 0) {
      keep[worst] = 1;
      stack.push([first, worst], [worst, last]);
    }
  }
  return coordinates.filter((_, i) => keep[i]);
}

// Join start and end, with the same template-then-fallback strategy and the
// same strict safety check as the existing generator's applyLoopClosure.
function applyLoopClosure(result, { joinStartAndEnd, turningRadius, safeArea }) {
  if (!joinStartAndEnd) return result;

  if (!Number.isFinite(turningRadius) || turningRadius <= 0) {
    const error = new Error('A positive turning radius is required to close the mower path.');
    error.status = 422;
    throw error;
  }

  const pathFeature = result.features.find((feature) => feature.geometry?.type === 'LineString');
  if (!pathFeature) throw new Error('Generated mower path has no LineString to close.');

  const isSafe = (connector) => {
    if (!safeArea || connector.length < 2) return false;
    if (!connector.every((point) => turf.booleanPointInPolygon(turf.point(point), safeArea))) return false;
    return turf.booleanWithin(turf.lineString(connector), safeArea);
  };

  let loop = null;
  try {
    const template = closeMowerPath(pathFeature.geometry.coordinates, { turningRadius, gap: LOOP_GAP_M });
    if (isSafe(template.connector)) loop = template;
  } catch (templateError) {
    console.warn("Loop template unavailable, using fallback planner:", templateError.message);
  }
  if (!loop) {
    loop = closeMowerPathFallback(pathFeature.geometry.coordinates, { turningRadius, gap: LOOP_GAP_M, safeArea });
    if (loop && !isSafe(loop.connector)) loop = null;
  }
  if (!loop) {
    const error = new Error(
      "Unable to join the mower path: no route at the configured turning radius from the end point back to the start stays inside the boundary and clear of obstacles. Try a smaller turning radius or a larger boundary offset."
    );
    error.code = "LOOP_CONNECTOR_OUTSIDE_SAFE_AREA";
    error.status = 422;
    throw error;
  }
  const totalLength = turf.length(turf.lineString(loop.coordinates), { units: 'meters' });
  pathFeature.geometry.coordinates = loop.coordinates;
  pathFeature.properties = {
    ...(pathFeature.properties || {}),
    ...loop.properties,
    total_length_meters: Math.round(totalLength * 100) / 100,
    is_closed_loop: false
  };
  return result;
}

// Pure — no req/res. Returns { status, result, contentDisposition } on
// success or { status, error, message? } on failure.
function computeF2cMowerPath(body) {
  try {
    if (!body) {
      return { status: 400, error: "Missing request body" };
    }
    const {
      polygonCoords,
      referenceLineCoords,
      offsetDistance,
      spacing,
      turningRadius,
      crissCross,
      obstacleAvoid,
      obstaclePolygons
    } = body;

    if (!polygonCoords || !referenceLineCoords || offsetDistance == null || offsetDistance === "" || !spacing) {
      return { status: 400, error: "Missing required parameters" };
    }
    // The headland is where every turn happens, so it must be a real width.
    const headlandWidth = Number(offsetDistance);
    if (!Number.isFinite(headlandWidth) || headlandWidth <= 0) {
      return { status: 400, error: "offsetDistance must be a number greater than 0." };
    }
    if (!Array.isArray(polygonCoords) || polygonCoords.length < 4) {
      return { status: 400, error: "polygonCoords must be an array of at least 4 [lng, lat] positions." };
    }
    if (!Array.isArray(referenceLineCoords) || referenceLineCoords.length < 2) {
      return { status: 400, error: "referenceLineCoords must be an array of at least 2 [lng, lat] positions." };
    }

    // Obstacles only shape the rows and turns when obstacle-avoid is on.
    const obstacles = Array.isArray(obstaclePolygons) ? obstaclePolygons : [];
    const obstacleAware = !!obstacleAvoid && obstacles.length > 0;

    const plan = planCoverage({
      polygonCoords,
      obstacleRings: obstacleAware ? obstacles : [],
      referenceLineCoords,
      headlandWidth,
      spacing: Number(spacing),
      turningRadius: Number(turningRadius),
      crissCross: !!crissCross
    });

    if (plan.coordinates.length < 2) {
      return { status: 400, error: "No swath lines could be generated. Offset/obstacles may be too large for this polygon." };
    }

    const totalLength = turf.length(turf.lineString(plan.coordinates), { units: "meters" });
    const result = {
      type: "FeatureCollection",
      features: [
        {
          type: "Feature",
          properties: {
            type: "complete_mower_path",
            total_length_meters: Math.round(totalLength * 100) / 100,
            is_computer_generated: true,
            planner: "fields2cover",
            criss_cross: !!crissCross,
            obstacle_aware: obstacleAware,
            obstacle_count: obstacleAware ? obstacles.length : 0
          },
          geometry: { type: "LineString", coordinates: plan.coordinates }
        }
      ]
    };

    // The join connector must stay inside the boundary and clear of obstacles
    // even when obstacle-avoid is off for the mowing rows themselves.
    let loopSafeArea = plan.driveFeature;
    if (body.joinStartAndEnd && !obstacleAware && obstacles.length) {
      loopSafeArea = fieldArea(
        polygonCoords,
        obstacles,
        Math.min(headlandWidth, CONNECTOR_BOUNDARY_CLEARANCE_M),
        Math.min(headlandWidth, CONNECTOR_OBSTACLE_CLEARANCE_M)
      );
    }
    applyLoopClosure(result, { joinStartAndEnd: !!body.joinStartAndEnd, turningRadius, safeArea: loopSafeArea });
    const pathGeometry = result.features[0].geometry;
    pathGeometry.coordinates = simplifyPath(pathGeometry.coordinates, OUTPUT_TOLERANCE_M);
    return {
      status: 200,
      result,
      contentDisposition: obstacleAware
        ? "attachment; filename=mower_path_f2c_obstacle_aware.geojson"
        : "attachment; filename=mower_path_f2c.geojson"
    };
  } catch (error) {
    console.error("Error generating F2C mower path:", error);
    return {
      status: error.status || 500,
      error: error.status ? error.message : "Failed to generate mower path",
      ...(error.status ? {} : { message: error.message })
    };
  }
}

module.exports = { computeF2cMowerPath };
