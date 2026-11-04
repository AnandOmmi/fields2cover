// Port of Fields2Cover's decomp::BoustrophedonDecomp, done at swath
// resolution: sweep across the swath columns and keep extending a cell while
// each swath piece overlaps exactly one piece in the previous column (and
// vice versa). Where an obstacle or concave notch splits a column (one piece
// overlapping two) or two pieces merge back, the cell closes and new ones
// start — the same split/merge events BCD cuts at, without having to
// compute critical points on the polygon itself.
//
// Every cell is then a run of adjacent swaths, one piece per column, so a
// row pattern inside it never has to cross an obstacle between rows.

const overlaps = ([a0, a1], [b0, b1]) => Math.min(a1, b1) - Math.max(a0, b0) > 0;

function decomposeIntoCells(columns) {
  const cells = [];
  let open = []; // { cell, interval } from the previous column

  columns.forEach(({ u, intervals }) => {
    const prevMatches = open.map((o) => intervals.filter((q) => overlaps(o.interval, q)).length);
    const next = intervals.map((q) => {
      const matches = [];
      open.forEach((o, index) => {
        if (overlaps(o.interval, q)) matches.push(index);
      });
      let cell;
      if (matches.length === 1 && prevMatches[matches[0]] === 1) {
        cell = open[matches[0]].cell;
      } else {
        cell = { rows: [] };
        cells.push(cell);
      }
      cell.rows.push({ u, v0: q[0], v1: q[1] });
      return { cell, interval: q };
    });
    open = next;
  });

  return cells;
}

module.exports = { decomposeIntoCells };
