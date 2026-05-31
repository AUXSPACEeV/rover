# Contributing to the Rover Project

Thank you for your interest in contributing! This project is a ROS 2 application written in C++ for an open-source rover platform. Whether you're fixing a bug, adding a new feature, or improving documentation, we're glad to have your help.

Please read this guide fully before submitting your first contribution.

---

## Table of Contents

- [Getting Started](#getting-started)
- [Reporting Issues](#reporting-issues)
- [Branch & Pull Request Workflow](#branch--pull-request-workflow)
- [Commit Message Conventions](#commit-message-conventions)
- [Code Style & Linting](#code-style--linting)
- [Testing Requirements](#testing-requirements)
- [Review Process](#review-process)

---

## Getting Started

### Prerequisites

- Ubuntu Resolute (26.04) installed
- **ROS 2 Lyrical Luth** installed and sourced
- **colcon** build tool

We also provide a custom ISO and setup script which takes care of this. Please take look at the [Wiki](../../wiki).

### Building the project

```bash
# Clone the repository
git clone https://github.com/AUXSPACEeV/rover.git
cd rover

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

---

## Reporting Issues

We use GitHub Issues to track bugs, feature requests, and other tasks. Before opening a new issue, please search existing issues to avoid duplicates.

### Bug Reports

Use the **Bug Report** issue template and include:

- A clear, descriptive title
- **ROS 2 version** and **OS** (e.g., ROS 2 Lyrical on Ubuntu 26.04)
- Steps to reproduce the problem
- What you expected to happen vs. what actually happened
- Relevant log output (`ros2 run ... --ros-args --log-level debug`)
- Any hardware details if the bug is hardware-specific (e.g., motor controller, sensor model)

### Feature Requests

Use the **Feature Request** issue template and include:

- A description of the problem you're trying to solve
- Your proposed solution or idea
- Any alternatives you've considered
- Whether you're willing to implement it yourself

### Good to Know

- Issues tagged `good first issue` are beginner-friendly and well-scoped.
- Issues tagged `help wanted` are higher priority but may require deeper familiarity with the codebase.
- For questions or design discussions, prefer **GitHub Discussions** over Issues.

---

## Branch & Pull Request Workflow

### Branch Naming

Create branches from `main` using the following convention:

| Type | Pattern | Example |
|---|---|---|
| Feature | `feat/<short-description>` | `feat/lidar-obstacle-avoidance` |
| Bug fix | `fix/<short-description>` | `fix/imu-frame-transform` |
| Documentation | `docs/<short-description>` | `docs/update-contributing` |
| Refactor | `refactor/<short-description>` | `refactor/motor-driver-interface` |
| CI / tooling | `ci/<short-description>` | `ci/add-colcon-cache` |

Use lowercase and hyphens only. Keep names concise but descriptive.

### Opening a Pull Request

1. **Fork** the repository and create your branch from `main`.
2. Make your changes, following the code style and testing requirements below.
3. Ensure all tests pass locally (`colcon test`).
4. Push your branch and open a Pull Request against `main`.
5. Fill in the PR template completely — incomplete PRs may be closed without review.

### Pull Request Checklist

Before marking your PR as ready for review, confirm:

- [ ] Branch is up to date with `main` (rebase preferred over merge commits)
- [ ] Code follows the style guide and passes `clang-format` / `clang-tidy`
- [ ] New nodes and public APIs are documented with Doxygen comments
- [ ] Tests are added or updated for all changed behaviour
- [ ] All CI checks pass
- [ ] PR description explains *what* changed and *why*
- [ ] Linked to a relevant Issue (e.g., `Closes #42`)

### Keeping Your Branch Up to Date

```bash
git fetch origin
git rebase origin/main
```

Avoid merge commits in feature branches — use rebase to keep history clean.

---

## Commit Message Conventions

We follow the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) specification. This enables automated changelogs and makes history easy to navigate.

### Format

```
<type>(<scope>): <short summary>

[optional body]

[optional footer(s)]
```

### Types

| Type | When to use |
|---|---|
| `feat` | A new feature |
| `fix` | A bug fix |
| `docs` | Documentation only changes |
| `refactor` | Code change that neither fixes a bug nor adds a feature |
| `test` | Adding or updating tests |
| `ci` | CI/CD configuration changes |
| `chore` | Maintenance tasks (dependency updates, build scripts, etc.) |
| `perf` | Performance improvements |

### Scope (optional but encouraged)

Use the ROS 2 package name or subsystem as scope:

```
feat(navigation): add costmap inflation layer
fix(drivetrain): correct wheel velocity scaling
docs(imu_driver): add calibration procedure
```

### Rules

- Use the **imperative mood** in the summary: "add feature" not "added feature"
- Keep the summary line under **72 characters**
- Reference issues in the footer: `Closes #42` or `Refs #17`
- Mark breaking changes with `!` after the type or a `BREAKING CHANGE:` footer

### Examples

```
feat(arm_controller): add joint limit enforcement

Adds soft and hard joint limits to the arm controller node.
Limits are loaded from the URDF and enforced at the control loop level.

Closes #88
```

```
fix(camera_driver)!: remove deprecated image_transport API

BREAKING CHANGE: The `image_raw` topic is now published under
`camera/image_raw` to align with REP-2003.
```

---

## Code Style & Linting

All C++ code must conform to the **[ROS 2 C++ style guide](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)**, which is based on the Google C++ Style Guide with ROS-specific adaptations.

### Formatting with ament tools

We use the standard ROS 2 linting stack via `ament_lint`. The primary tools are:

- **`ament_clang_format`** — enforces code formatting
- **`ament_cpplint`** — checks style rules
- **`ament_uncrustify`** — additional formatting enforcement

Both `ament_clang_format` and `ament_uncrustify` support in-place reformatting:

```bash
ament_clang_format --reformat src/
ament_uncrustify --reformat src/
```

A `.clang-format` file is provided at the root of the repository. CI will fail on any file that does not pass the linter suite.

### Static Analysis

We also run `cppcheck` via `ament_cppcheck` and compile with `-Wall -Wextra -Wpedantic`. You can invoke these as part of the test suite:

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

### Key Style Rules

- Maximum line length is **100 characters**
- Indent with **2 spaces** — no tabs
- Class names in `UpperCamelCase`; functions and variables in `snake_case`
- Constants: ROS 2 does not enforce a single convention — match the style of surrounding code (`snake_case`, `UPPER_CASE`, or `PascalCase` are all found in the ecosystem)
- Global variables in `snake_case` prefixed with `g_` (e.g., `g_node_count`)
- ROS 2 node names, topic names, and parameter names in `snake_case`
- Always use braces after `if`, `else`, `for`, `while`, and `do`, even for single-line bodies
- Pointer and reference alignment: `char * c;` (space on both sides)
- Avoid raw pointers — prefer `std::shared_ptr`, `std::unique_ptr`, or references
- Avoid `using namespace` in header files
- Avoid Boost unless absolutely required
- All public APIs must have `///` or `/** */` Doxygen-style comments; use `//` for inline notes

### Headers and Includes

- Header files must use the `.hpp` extension
- Implementation files must use the `.cpp` extension

Order includes as follows, each group separated by a blank line:

1. Related header (the `.hpp` for the current `.cpp`)
2. C++ standard library headers
3. Third-party library headers (Eigen, OpenCV, etc.)
4. ROS 2 headers (`rclcpp`, `geometry_msgs`, etc.)
5. Project-internal headers

```cpp
#include "my_package/my_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "my_package/utils.hpp"
```

---

## Testing Requirements

All contributions that change behaviour must include tests. We use the standard ROS 2 testing stack: **GTest** for unit tests and **launch_testing** for integration tests.

### Unit Tests

- Place unit tests under `test/` in the relevant package
- Test file names must end in `_test.cpp`
- Aim for tests that are **fast** (< 1 s each), **isolated** (no ROS spin required), and **deterministic**
- Mock or stub external hardware interfaces and ROS topics where needed

```bash
# Run all tests
colcon test

# Run tests for a single package
colcon test --packages-select <package_name>

# View results
colcon test-result --verbose
```

### Integration / Launch Tests

For tests that require a running ROS graph (e.g., testing node communication or lifecycle behaviour), use `launch_testing`:

- Place launch tests under `test/` with names ending in `_launch_test.py`
- Keep integration tests to scenarios that cannot be meaningfully covered by unit tests
- Integration tests must not require physical hardware to pass — use simulated topics or mock drivers

### Coverage

- New features should aim for **>80% line coverage** on changed code
- Bug fixes must include at least one regression test that would have caught the bug

### What We Don't Accept

- PRs that reduce overall test coverage
- Tests that rely on timing (e.g., `sleep()`) rather than event-driven synchronisation
- Tests that require a physical rover or sensors to run in CI

---

## Review Process

- At least **one maintainer approval** is required before merging
- CI must be fully green (build, lint, and tests)
- Maintainers may request changes; please address all comments or start a discussion if you disagree
- Once approved and CI is green, a maintainer will merge using **squash and merge** to keep the main branch history clean
- Stale PRs (no activity for 30 days) may be closed — comment to reopen at any time

---

Thank you for contributing and helping make this rover better! 🤖
