# Build and run rmw_iceoryx2. Invoke from the workspace root:
#   just -f src/rmw_iceoryx2/justfile <recipe>

import '.just/common.just'
import '.just/build.just'
import '.just/examples.just'
import '.just/benchmarks.just'
import '.just/ci.just'

_default:
    @just --justfile {{justfile()}} --list
