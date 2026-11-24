# Changelog

## 0.1.1 — 2026-11-24

### Fixed
- Self-intersecting obstacle outlines (e.g. recorded drives around long, thin obstacles) are normalised into valid polygons, so turns and transits no longer cut through them.
- Removed small loops/circles from the mower path; turns may tighten to 65% of the turning radius where needed.
- Grid (criss-cross) crash, missing rows, obstacle-tip clipping and a lost half of the field on irregular boundaries.
- Row spacing measured in metres; dead-end rows turn round instead of being skipped.
- Faster drive-area checks.

### Changed
- Grid (criss-cross) pass works with obstacle avoidance.
- `contentDisposition` filename includes `_grid` for criss-cross paths.

## 0.1.0 — 2026-11-04

First release.

- Fields2Cover-style coverage planner in pure JavaScript (`planCoverage`).
- Mower path generator with validation and optional start/end join (`computeF2cMowerPath`).
- Headland generation, boustrophedon cell decomposition, swaths along a reference line.
- Headland laps with even spacing, leapfrog row order, min-radius Dubins turns and obstacle transits.
- Criss-cross (second pass at 90°) and obstacle avoidance.
