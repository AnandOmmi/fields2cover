# @anandommi/fields2cover

[![CI](https://github.com/AnandOmmi/fields2cover/actions/workflows/ci.yml/badge.svg)](https://github.com/AnandOmmi/fields2cover/actions/workflows/ci.yml)
[![npm](https://img.shields.io/npm/v/@anandommi/fields2cover)](https://www.npmjs.com/package/@anandommi/fields2cover)

Pure-JavaScript coverage path planner for robotic mowers, following the
[Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) v2 pipeline.
No native build — runs anywhere Node runs, on plain GeoJSON `[lng, lat]` coordinates.

| stage | file | what it does |
|---|---|---|
| headland | `lib/headland.js` | constant-width headland around boundary and obstacles |
| decomposition | `lib/decomposition.js` | boustrophedon cells, so rows never straddle an obstacle |
| swaths | `lib/swaths.js` | parallel swaths along the reference line |
| headland laps | `lib/headlandLaps.js` | laps that mow the headland band itself |
| route order | `lib/routeOrder.js` | leapfrog row order per cell + greedy cell order |
| path planning | `lib/pathPlanner.js` | min-radius Dubins turns, transit around obstacles |

Differences from Fields2Cover: the swath angle comes from a user reference
line (no angle search), cells are ordered greedily instead of with OR-tools,
and every turn/transit is validated against the real drive area.

## Install

```sh
npm install @anandommi/fields2cover
```

## Usage

```js
const { computeF2cMowerPath } = require('@anandommi/fields2cover');

const out = computeF2cMowerPath({
  polygonCoords: [[0, 0], [0.0009, 0], [0.0009, 0.00054], [0, 0.00054], [0, 0]],
  referenceLineCoords: [[0, 0], [0.0009, 0]], // rows run parallel to this
  offsetDistance: 3,     // headland width, m
  spacing: 1,            // row spacing, m
  turningRadius: 2,      // minimum turning radius, m
  crissCross: false,     // add a second pass at 90°
  obstacleAvoid: true,
  obstaclePolygons: [],  // array of [lng, lat] rings
  joinStartAndEnd: false // close the path back to its start
});

if (out.status === 200) {
  console.log(out.result); // GeoJSON FeatureCollection with one LineString
} else {
  console.error(out.status, out.error);
}
```

`computeF2cMowerPath` never throws: it returns `{ status, result }` on success
or `{ status, error }` on failure. The lower-level
`planCoverage({ polygonCoords, obstacleRings, referenceLineCoords, headlandWidth, spacing, turningRadius, crissCross })`
returns raw coordinates and stats, and throws on bad input.

For large fields, run it inside a `worker_threads` worker so it does not block the event loop.

## Releasing

1. Bump `version` in `package.json` and add a `CHANGELOG.md` entry.
2. Commit, tag `vX.Y.Z`, push the tag.
3. The `Publish to npm` workflow tests and publishes (needs repo secret `NPM_TOKEN`).

Manual alternative: `npm login` then `npm publish` (runs tests first via `prepublishOnly`).

## License

BSD-3-Clause
