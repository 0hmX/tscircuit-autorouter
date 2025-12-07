import { CapacitySegmentToPointSolver } from "lib/solvers/CapacityMeshSolver/CapacitySegmentToPointSolver"
import type { NodePortSegment } from "lib/types/capacity-edges-to-port-segments-types"
import type { CapacityMeshNode, Obstacle } from "lib/types"
import { mapLayerNameToZ } from "lib/utils/mapLayerNameToZ"

/**
 * Extends the base CapacitySegmentToPointSolver to incorporate obstacle avoidance
 * when assigning points to port segments. This solver calculates "safe" intervals
 * on a segment by clipping regions that would overlap with obstacles (plus a margin)
 * and then distributes connection points within these safe zones.
 */
export class CapacitySegmentToPointSolver2_ObstacleClipping extends CapacitySegmentToPointSolver {
  obstacles: Obstacle[]
  margin: number
  layerCount: number

  /**
   * Constructs a new CapacitySegmentToPointSolver2_ObstacleClipping instance.
   */
  constructor(params: {
    segments: NodePortSegment[]
    colorMap?: Record<string, string>
    nodes: CapacityMeshNode[]
    obstacles?: Obstacle[]
    margin?: number
    layerCount?: number
  }) {
    super({
      segments: params.segments,
      colorMap: params.colorMap,
      nodes: params.nodes,
    })
    this.obstacles = params.obstacles ?? []
    this.margin = params.margin ?? 0.15
    this.layerCount = params.layerCount ?? 2
  }

  /**
   * Projects an obstacle onto a given segment, determining the [0, 1] interval
   * along the segment that the obstacle (expanded by margin) blocks.
   * Considers Z-layer compatibility.
   */
  private projectObstacleOntoSegment(params: {
    seg: NodePortSegment
    obstacle: Obstacle
    margin: number
    epsilon?: number
  }): [number, number] | null {
    const { seg, obstacle, margin } = params
    const epsilon = params.epsilon ?? 1e-9

    // 1. Check Z-layer compatibility
    const obstacleZLayers =
      obstacle.zLayers ||
      obstacle.layers?.map((l) => mapLayerNameToZ(l, this.layerCount)) ||
      []
    const commonZ = seg.availableZ.filter((z) => obstacleZLayers.includes(z))
    if (commonZ.length === 0) {
      return null // No common layers, obstacle doesn't block this segment
    }

    // 2. Determine segment orientation
    const isHorizontal = Math.abs(seg.start.y - seg.end.y) < epsilon
    const isVertical = Math.abs(seg.start.x - seg.end.x) < epsilon

    if (!isHorizontal && !isVertical) {
      return null // Diagonal segments not handled yet
    }

    // 3. Calculate obstacle's effective bounds
    const obsMinX = obstacle.center.x - obstacle.width / 2 - margin
    const obsMaxX = obstacle.center.x + obstacle.width / 2 + margin
    const obsMinY = obstacle.center.y - obstacle.height / 2 - margin
    const obsMaxY = obstacle.center.y + obstacle.height / 2 + margin

    // 4. Get segment's actual length and direction for projection
    const segVecX = seg.end.x - seg.start.x
    const segVecY = seg.end.y - seg.start.y
    const segmentLength = Math.sqrt(segVecX * segVecX + segVecY * segVecY)

    if (segmentLength < epsilon) {
      return null // Degenerate segment
    }

    let tStart = 0
    let tEnd = 1

    if (isHorizontal) {
      const segY = seg.start.y
      // If obstacle (with margin) does not overlap with segment's Y-range, return null
      if (obsMaxY < segY - epsilon || obsMinY > segY + epsilon) return null

      // Project obstacle X range onto segment's X range
      const segMinX = Math.min(seg.start.x, seg.end.x)
      const segMaxX = Math.max(seg.start.x, seg.end.x)

      const projectedMinX = Math.max(segMinX, obsMinX)
      const projectedMaxX = Math.min(segMaxX, obsMaxX)

      if (projectedMinX >= projectedMaxX - epsilon) return null // No actual overlap

      // Convert projected X range to a [0, 1] interval along the segment
      tStart = (projectedMinX - seg.start.x) / segVecX
      tEnd = (projectedMaxX - seg.start.x) / segVecX

      // Ensure t values are sorted if segment direction is reversed
      if (segVecX < 0) [tStart, tEnd] = [tEnd, tStart]
    } else if (isVertical) {
      const segX = seg.start.x
      // If obstacle (with margin) does not overlap with segment's X-range, return null
      if (obsMaxX < segX - epsilon || obsMinX > segX + epsilon) return null

      // Project obstacle Y range onto segment's Y range
      const segMinY = Math.min(seg.start.y, seg.end.y)
      const segMaxY = Math.max(seg.start.y, seg.end.y)

      const projectedMinY = Math.max(segMinY, obsMinY)
      const projectedMaxY = Math.min(segMaxY, obsMaxY)

      if (projectedMinY >= projectedMaxY - epsilon) return null // No actual overlap

      // Convert projected Y range to a [0, 1] interval along the segment
      tStart = (projectedMinY - seg.start.y) / segVecY
      tEnd = (projectedMaxY - seg.start.y) / segVecY

      // Ensure t values are sorted if segment direction is reversed
      if (segVecY < 0) [tStart, tEnd] = [tEnd, tStart]
    }

    // Clamp t values to [0, 1]
    tStart = Math.max(0, tStart)
    tEnd = Math.min(1, tEnd)

    if (tStart >= tEnd - epsilon) return null // Interval too small

    return [tStart, tEnd]
  }

  /**
   * Subtracts a subtrahend interval from a list of sorted, non-overlapping intervals.
   * Returns a new list of intervals representing the remaining "free" space.
   *
   * @param params Object containing the intervals, subtrahend, and epsilon.
   * @param params.intervals The list of [start, end] intervals (relative to [0,1]).
   * @param params.subtrahend The [start, end] interval to subtract.
   * @param params.epsilon A small value for floating-point comparisons (optional).
   * @returns A new list of sorted, non-overlapping intervals.
   */
  private subtractIntervals(params: {
    intervals: [number, number][]
    subtrahend: [number, number]
    epsilon?: number
  }): [number, number][] {
    const { intervals, subtrahend } = params
    const epsilon = params.epsilon ?? 1e-9
    const result: [number, number][] = []
    const [subStart, subEnd] = subtrahend

    for (const [intStart, intEnd] of intervals) {
      // Case 1: Subtrahend completely before interval
      if (subEnd <= intStart + epsilon) {
        result.push([intStart, intEnd])
        continue
      }
      // Case 2: Subtrahend completely after interval
      if (subStart >= intEnd - epsilon) {
        result.push([intStart, intEnd])
        continue
      }

      // Case 3: Subtrahend completely covers interval
      if (subStart <= intStart + epsilon && subEnd >= intEnd - epsilon) {
        continue
      }

      // Case 4: Subtrahend splits interval
      if (subStart > intStart + epsilon && subEnd < intEnd - epsilon) {
        result.push([intStart, subStart])
        result.push([subEnd, intEnd])
        continue
      }

      // Case 5: Subtrahend overlaps left part
      if (subStart <= intStart + epsilon && subEnd < intEnd - epsilon) {
        result.push([subEnd, intEnd])
        continue
      }

      // Case 6: Subtrahend overlaps right part
      if (subStart > intStart + epsilon && subEnd >= intEnd - epsilon) {
        result.push([intStart, subStart])
        continue
      }
    }

    return result.filter(([start, end]) => end - start > epsilon)
  }

  /**
   * Calculates the "safe" [0, 1] intervals along a segment that are not
   * blocked by any obstacle (considering the configured margin).
   */
  private getSafeIntervals(params: {
    seg: NodePortSegment
  }): [number, number][] {
    const { seg } = params
    let currentIntervals: [number, number][] = [[0, 1]]

    for (const obstacle of this.obstacles) {
      const blockedInterval = this.projectObstacleOntoSegment({
        seg,
        obstacle,
        margin: this.margin,
      })

      if (blockedInterval) {
        currentIntervals = this.subtractIntervals({
          intervals: currentIntervals,
          subtrahend: blockedInterval,
        })
      }
    }

    return currentIntervals.sort((a, b) => a[0] - b[0])
  }

  /**
   * Distributes a given number of points evenly within a list of safe intervals.
   * Maps the relative [0,1] positions back to absolute (x,y,z) coordinates.
   */
  private pickPointsFromIntervals(params: {
    intervals: [number, number][]
    numPoints: number
    segStart: { x: number; y: number }
    segEnd: { x: number; y: number }
    z: number
    epsilon?: number
  }): { x: number; y: number; z: number }[] {
    const { intervals, numPoints, segStart, segEnd, z } = params
    const epsilon = params.epsilon ?? 1e-9

    if (numPoints === 0) return []

    const points: { x: number; y: number; z: number }[] = []
    const dx = segEnd.x - segStart.x
    const dy = segEnd.y - segStart.y

    let totalUsableLength = 0
    for (const [start, end] of intervals) {
      totalUsableLength += end - start
    }

    if (totalUsableLength < epsilon) {
      return []
    }

    const stepSize = totalUsableLength / (numPoints + 1)
    let currentUsableLengthCovered = 0
    let intervalIndex = 0

    for (let i = 1; i <= numPoints; i++) {
      const targetUsablePosition = i * stepSize

      while (intervalIndex < intervals.length) {
        const [intStart, intEnd] = intervals[intervalIndex]
        const intervalLength = intEnd - intStart

        if (
          currentUsableLengthCovered + intervalLength - epsilon >
          targetUsablePosition
        ) {
          const positionInInterval =
            targetUsablePosition - currentUsableLengthCovered
          const relativePositionInCurrentInterval =
            positionInInterval / intervalLength
          const relativePositionInSegment =
            intStart + relativePositionInCurrentInterval * intervalLength

          points.push({
            x: segStart.x + dx * relativePositionInSegment,
            y: segStart.y + dy * relativePositionInSegment,
            z,
          })
          break
        } else {
          currentUsableLengthCovered += intervalLength
          intervalIndex++
        }
      }
    }

    return points
  }

  /**
   * Performs one iteration step of the solver.
   * Overrides the base class's `_step` method to apply obstacle clipping logic.
   * For each unsolved segment, it calculates safe regions, distributes points,
   * and handles fallbacks if segments cannot be fully assigned.
   */
  _step() {
    let updated = false
    const unsolved = [...this.unsolvedSegments]

    // Iterate over unsolved segments
    for (const seg of unsolved) {
      const n = seg.connectionNames.length
      // Already processed? Skip if assignedPoints exists for all connections.
      if ("assignedPoints" in seg && seg.assignedPoints?.length === n) continue

      const safeIntervals = this.getSafeIntervals({ seg })

      if (safeIntervals.length === 0) {
        // If no safe intervals, the segment is completely blocked.
        continue
      }

      const pointsForSegment = this.pickPointsFromIntervals({
        intervals: safeIntervals,
        numPoints: n,
        segStart: seg.start,
        segEnd: seg.end,
        z: seg.availableZ[0],
      })

      if (n === 1) {
        if (pointsForSegment.length > 0) {
          ;(seg as any).assignedPoints = [
            {
              connectionName: seg.connectionNames[0],
              point: pointsForSegment[0],
            },
          ]
          this.unsolvedSegments.splice(this.unsolvedSegments.indexOf(seg), 1)
          this.solvedSegments.push(seg as any)
          updated = true
        }
      }
    }

    // Fallback for segments not updated in this iteration
    if (!updated && unsolved.length > 0) {
      // Choose the unsolved segment with the fewest connections
      let candidate = unsolved[0]
      for (const seg of unsolved) {
        if (seg.connectionNames.length < candidate.connectionNames.length) {
          candidate = seg
        }
      }

      const safeIntervalsForCandidate = this.getSafeIntervals({
        seg: candidate,
      })

      if (safeIntervalsForCandidate.length > 0) {
        const sortedConnections = [...candidate.connectionNames].sort()
        const points = this.pickPointsFromIntervals({
          intervals: safeIntervalsForCandidate,
          numPoints: sortedConnections.length,
          segStart: candidate.start,
          segEnd: candidate.end,
          z: candidate.availableZ[0],
        })

        if (points.length === sortedConnections.length) {
          ;(candidate as any).assignedPoints = sortedConnections.map(
            (conn, idx) => ({
              connectionName: conn,
              point: points[idx],
            }),
          )
          this.unsolvedSegments.splice(
            this.unsolvedSegments.indexOf(candidate),
            1,
          )
          this.solvedSegments.push(candidate as any)
          updated = true
        }
      }
    }

    if (this.unsolvedSegments.length === 0) {
      this.solved = true
    }
  }
}
