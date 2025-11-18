import { expect, test } from "bun:test"
import { AutoroutingPipelineSolver } from "lib"
import type { SimpleRouteJson } from "lib/types"
import { getLastStepSvg } from "../fixtures/getLastStepSvg"
import srj from "../../examples/internally-connected-pads/internally-connected-pads.json"

test("NetToPointPairsSolver with internally connected pads", () => {
  const solver = new AutoroutingPipelineSolver(srj as SimpleRouteJson)
  solver.solve()

  expect(getLastStepSvg(solver.visualize())).toMatchSvgSnapshot(
    import.meta.path,
  )
})
