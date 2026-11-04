// Swath order inside one cell — the F2C rp::CustomOrder slot, filled with a
// leapfrog skip pattern.
//
// k is the smallest jump whose *backward* step (k-1 rows) still leaves room
// for a flat U-turn: quarter arc, a straight of UTURN_STRAIGHT_FACTOR × R,
// quarter arc — never a bare semicircle, and never a loop. (With
// k = floor(2R/spacing) the backward step was narrower than 2R and every
// second turn became a loop.)
const { UTURN_STRAIGHT_FACTOR } = require('./pathPlanner'); // R=6: 3 m straight across the U-turn

function rowSkip(spacing, minTurnRadius) {
  const turnWidth = (2 + UTURN_STRAIGHT_FACTOR) * minTurnRadius;
  return Math.max(1, Math.ceil(turnWidth / spacing - 1e-9) + 1);
}

const generateCustomOrder = (totalSwaths, spacing, minTurnRadius) => {
  if (totalSwaths <= 0) return [];
  const k = rowSkip(spacing, minTurnRadius);
  const customOrder = [];
  const swathsPerCircuit = 2 * k;
  const numCircuits = Math.floor(totalSwaths / swathsPerCircuit);
  const remainingSwaths = totalSwaths % swathsPerCircuit;
  const circuits = [];
  for (let i = 0; i < numCircuits; i++) {
    circuits.push(i === 0 ? swathsPerCircuit + remainingSwaths : swathsPerCircuit);
  }
  if (circuits.length === 0) circuits.push(totalSwaths);

  let swathCounter = 1; // 1-based start index of the current circuit
  for (let c = 0; c < circuits.length; c++) {
    const circuitSwaths = circuits[c];
    if (c === 0) {
      // First circuit (absorbs the remainder): pair row i with row i+skip.
      const skip = circuitSwaths % 2 === 0 ? circuitSwaths / 2 : Math.floor(circuitSwaths / 2) + 1;
      for (let i = 1; i <= skip; i++) {
        const a = i;
        const b = i + skip;
        if (a <= circuitSwaths) customOrder.push(swathCounter + a - 1);
        if (b <= circuitSwaths) customOrder.push(swathCounter + b - 1);
      }
    } else {
      // Later circuits: start on the high half, then interleave.
      const skip = Math.floor(circuitSwaths / 2);
      for (let i = 0; i < skip; i++) {
        const high = skip + i;
        const low = i;
        if (high < circuitSwaths) customOrder.push(swathCounter + high);
        if (low < circuitSwaths) customOrder.push(swathCounter + low);
      }
    }
    swathCounter += circuitSwaths;
  }
  return customOrder.map((v) => v - 1); // 0-based
};

module.exports = { generateCustomOrder, rowSkip };
