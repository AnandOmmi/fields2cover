const { planCoverage, CONNECTOR_BOUNDARY_CLEARANCE_M, CONNECTOR_OBSTACLE_CLEARANCE_M } = require('./lib/coverage');
const { computeF2cMowerPath } = require('./lib/mowerPath');
const { closeMowerPath } = require('./lib/mowerLoopPath');
const { closeMowerPathFallback } = require('./lib/mowerLoopFallback');

module.exports = {
  planCoverage,
  computeF2cMowerPath,
  closeMowerPath,
  closeMowerPathFallback,
  CONNECTOR_BOUNDARY_CLEARANCE_M,
  CONNECTOR_OBSTACLE_CLEARANCE_M
};
