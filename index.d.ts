export type Position = [number, number];
export type Ring = Position[];

export interface PlanCoverageOptions {
  polygonCoords: Ring;
  obstacleRings?: Ring[];
  referenceLineCoords: Position[];
  headlandWidth: number;
  spacing: number;
  turningRadius: number;
  crissCross?: boolean;
}

export interface PlanCoverageResult {
  coordinates: Position[];
  driveFeature: unknown;
  rowCounts: number[];
  cellCount: number;
  skippedRows: number;
  lapCount: number;
  skippedLaps: number;
  [key: string]: unknown;
}

export function planCoverage(options: PlanCoverageOptions): PlanCoverageResult;

export interface MowerPathRequest {
  polygonCoords: Ring;
  referenceLineCoords: Position[];
  offsetDistance: number;
  spacing: number;
  turningRadius: number;
  crissCross?: boolean;
  obstacleAvoid?: boolean;
  obstaclePolygons?: Ring[];
  joinStartAndEnd?: boolean;
}

export interface MowerPathFeatureCollection {
  type: 'FeatureCollection';
  features: Array<{
    type: 'Feature';
    properties: Record<string, unknown>;
    geometry: { type: 'LineString'; coordinates: Position[] };
  }>;
}

export type MowerPathResponse =
  | { status: 200; result: MowerPathFeatureCollection; contentDisposition: string }
  | { status: number; error: string; message?: string };

export function computeF2cMowerPath(body: MowerPathRequest): MowerPathResponse;

export function closeMowerPath(coordinates: Position[], options: { turningRadius: number; gap?: number }): any;
export function closeMowerPathFallback(coordinates: Position[], options: { turningRadius: number; gap?: number; safeArea?: unknown }): any;

export const CONNECTOR_BOUNDARY_CLEARANCE_M: number;
export const CONNECTOR_OBSTACLE_CLEARANCE_M: number;
