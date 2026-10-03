# rmw_iceoryx2 v?.?.?

## [vx.x.x](https://github.com/ekxide/rmw_iceoryx2/tree/vx.x.x)

[Full Changelog](https://github.com/ekxide/rmw_iceoryx2/compare/vx.x.x...vx.x.x)

### Features

<!--
    NOTE: Add new entries sorted by issue number to minimize the possibility of
    conflicts when merging.
-->

* Serialize/deserialized non-self-contained messages into `iceoryx2` payloads [#2](https://github.com/ekxide/rmw_iceoryx2/issues/2)
* Add support for QoS [#5](https://github.com/ekxide/rmw_iceoryx2/issues/2)
* Implement graph API for publish-subscribe topics [#6](https://github.com/ekxide/rmw_iceoryx2/issues/6)
* Trigger graph guard conditions when the graph changes [#6](https://github.com/ekxide/rmw_iceoryx2/issues/6)
* Add ROS 2 <-> iceoryx2 communication example [#44](https://github.com/ekxide/rmw_iceoryx2/issues/44)
* Add middleware overhead benchmarking application [#44](https://github.com/ekxide/rmw_iceoryx2/issues/44)
* Implement the missing publish-subscribe functions [#69](https://github.com/ekxide/rmw_iceoryx2/issues/69)
* Add benchmarks for the cost of rmw operations [#86](https://github.com/ekxide/rmw_iceoryx2/issues/86)

### Bugfixes

<!--
    NOTE: Add new entries sorted by issue number to minimize the possibility of
    conflicts when merging.
-->

* Fix memory leaks and loans kept after a failed take [#7](https://github.com/ekxide/rmw_iceoryx2/issues/7)
* Fix failing `gcc` build [#15](https://github.com/ekxide/rmw_iceoryx2/issues/15)
* Fix failing `gcc` and `clang` build on Ubuntu 22.04 [#29](https://github.com/ekxide/rmw_iceoryx2/issues/29)
* Fix non-triggered attachments not being set to `nullptr` on timeout [#36](https://github.com/ekxide/rmw_iceoryx2/issues/36)
* Delegate signal handling to `rcl` [#40](https://github.com/ekxide/rmw_iceoryx2/issues/40)
* Fix `rmw_wait` missing subscriptions and guard conditions that were ready before or during the wait [#51](https://github.com/ekxide/rmw_iceoryx2/issues/51)
* Export missing rmw functions [#55](https://github.com/ekxide/rmw_iceoryx2/issues/55)
* Publish and serialize messages with C typesupport [#57](https://github.com/ekxide/rmw_iceoryx2/issues/57)
* Skip topics that cannot be opened in the per-node graph queries [#62](https://github.com/ekxide/rmw_iceoryx2/issues/62)
* Report endpoints of unknown nodes with placeholder names [#63](https://github.com/ekxide/rmw_iceoryx2/issues/63)
* Propagate the errors of the endpoint info setters [#64](https://github.com/ekxide/rmw_iceoryx2/issues/64)
* Fix guard conditions failing after 255 per context [#82](https://github.com/ekxide/rmw_iceoryx2/issues/82)

### Refactoring

<!--
    NOTE: Add new entries sorted by issue number to minimize the possibility of
    conflicts when merging.
-->

* Organize code base to separate rmw api and implementation details [#16](https://github.com/ekxide/rmw_iceoryx2/issues/16)
* Bump `iceoryx2` dependency to v0.10.0 [#23](https://github.com/ekxide/rmw_iceoryx2/issues/23)
* Use https based url for git repos [#27](https://github.com/ekxide/rmw_iceoryx2/issues/27)
* Remove dependency on `iceoryx_hoofs` [#33](https://github.com/ekxide/rmw_iceoryx2/issues/27)

### Workflow

<!--
    NOTE: Add new entries sorted by issue number to minimize the possibility of
    conflicts when merging.
-->

* Add measurement for serialized messages to benchmark [#2](https://github.com/ekxide/rmw_iceoryx2/issues/15)
* Add CI for building and testing with `clang` [#12](https://github.com/ekxide/rmw_iceoryx2/issues/12)
* Use `vcs` to manage dependencies [#13](https://github.com/ekxide/rmw_iceoryx2/issues/13)
* Add CI for building and testing with `gcc` [#15](https://github.com/ekxide/rmw_iceoryx2/issues/15)
* Rename `main` branch to `rolling` [#38](https://github.com/ekxide/rmw_iceoryx2/issues/38)
* Add `just` scripts for building / running packages, demos and benchmark
* Build `performance_test` on Rolling [#88](https://github.com/ekxide/rmw_iceoryx2/issues/88)
  [#44](https://github.com/ekxide/rmw_iceoryx2/issues/44)
* Update Python dependencies for benchmark to apply security fixes [#47](https://github.com/ekxide/rmw_iceoryx2/issues/47)
* Make CI compatible with Ubuntu 26.04 [#49](https://github.com/ekxide/rmw_iceoryx2/issues/49)
* Properly specify direct package dependencies [#73](https://github.com/ekxide/rmw_iceoryx2/issues/73)

### New API features

<!--
    NOTE: Add new entries sorted by issue number to minimize the possibility of
    conflicts when merging.
-->

### API Breaking Changes

1. Example

   ```cpp
   // old
   auto fuu = Hello::is_it_me_you_re_looking_for();

   // new
   auto fuu = Hypnotoad::all_glory_to();
   ```
