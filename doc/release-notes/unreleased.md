# rmw_iceoryx2 v?.?.?

## [vx.x.x](https://github.com/ekxide/rmw_iceoryx2/tree/vx.x.x)

[Full Changelog](https://github.com/ekxide/rmw_iceoryx2/compare/vx.x.x...vx.x.x)

### Features

<!--
    NOTE: Add new entries sorted by issue number to minimize the possibility of
    conflicts when merging.
-->

* Serialize/deserialized non-self-contained messages into `iceoryx2` payloads [#2](https://github.com/ekxide/rmw_iceoryx2/issues/2)

### Bugfixes

<!--
    NOTE: Add new entries sorted by issue number to minimize the possibility of
    conflicts when merging.
-->

* Fix issue where messages containing fixed-size elements were not being identified as self-contained [#2](https://github.com/ekxide/rmw_iceoryx2/issues/2)
* Fix failing `gcc` build [#15](https://github.com/ekxide/rmw_iceoryx2/issues/15)
* Fix failing `gcc` and `clang` build on Ubuntu 22.04 [#29](https://github.com/ekxide/rmw_iceoryx2/issues/29)

### Refactoring

<!--
    NOTE: Add new entries sorted by issue number to minimize the possibility of
    conflicts when merging.
-->

* Organize code base to separate rmw api and implementation details [#16](https://github.com/ekxide/rmw_iceoryx2/issues/16)
* Use https based url for git repos [#27](https://github.com/ekxide/rmw_iceoryx2/issues/27)

### Workflow

<!--
    NOTE: Add new entries sorted by issue number to minimize the possibility of
    conflicts when merging.
-->

* Use `vcs` to manage dependencies [#13](https://github.com/ekxide/rmw_iceoryx2/issues/13)
* Add CI for building and testing with `clang` [#12](https://github.com/ekxide/rmw_iceoryx2/issues/12)
* Add CI for building and testing with `gcc` [#15](https://github.com/ekxide/rmw_iceoryx2/issues/15)

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
