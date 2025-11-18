import { AutoroutingPipelineDebugger } from "lib/testing/AutoroutingPipelineDebugger"
import srj from "./internally-connected-pads.json"
import { SimpleRouteJson } from "lib/types"

export default () => {
  return <AutoroutingPipelineDebugger srj={srj as SimpleRouteJson} />
}
