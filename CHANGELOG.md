# Changelog

## 0.1.0 — 2026-11-04

First release.

- Fields2Cover-style coverage planner in pure JavaScript (`planCoverage`).
- Mower path generator with validation and optional start/end join (`computeF2cMowerPath`).
- Headland generation, boustrophedon cell decomposition, swaths along a reference line.
- Headland laps with even spacing, leapfrog row order, min-radius Dubins turns and obstacle transits.
- Criss-cross (second pass at 90°) and obstacle avoidance.
