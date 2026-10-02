# Contributing to rmw_iceoryx2

Contributions are welcome. By contributing, you agree that your contributions
are licensed under the terms of this repository, `Apache-2.0 OR MIT`
(see [LICENSE-APACHE](LICENSE-APACHE) and [LICENSE-MIT](LICENSE-MIT)).

## Workflow

1. Before you start to work, please create an issue first, or comment on an
   existing one so it can be assigned to you.
2. Fork the repository and create a branch in your fork with the prefix
   `rmw-iox2-<issue-number>`, e.g. `rmw-iox2-123-short-description`.
3. Every source code file requires this copyright header. In a new file, fill
   in the current year and the copyright holder of your contribution: yourself,
   or your organization.

   ```text
   // Copyright (c) <year> by <copyright holder> All rights reserved.
   //
   // This program and the accompanying materials are made available under the
   // terms of the Apache Software License 2.0 which is available at
   // https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
   // which is available at https://opensource.org/licenses/MIT.
   //
   // SPDX-License-Identifier: Apache-2.0 OR MIT
   ```

   When you make substantial changes to an existing file, you may add your
   own copyright line below the existing ones. This is optional. Never remove
   or change the existing copyright lines.

4. Every commit must have the prefix `[#<issue-number>]`, e.g.
   `[#123] Short description of the change`.
5. Format C and C++ code with `clang-format`, using
   [rmw_iceoryx2_cxx/.clang-format](rmw_iceoryx2_cxx/.clang-format).
6. Add tests for new behavior.
7. When the work is done, add your changes to the release notes in
   [doc/release-notes/unreleased.md](doc/release-notes/unreleased.md).
8. Create a pull request from your fork and fill in the pull request
   template.

The [README](README.md#setup) describes how to set up a workspace and build
the project.

## Review

* Every pull request needs the approval of a code owner before it is merged.
* The CI must pass. For first-time contributors, a maintainer has to approve
  the CI run.

## Reporting a Vulnerability

Please do not open a public issue for a security problem; see
[SECURITY.md](SECURITY.md).
