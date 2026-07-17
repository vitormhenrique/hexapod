# Local and CI Quality Gates

The GitHub Actions workflow at `.github/workflows/quality.yml` runs on pushes
to `main`, pull requests, and manual dispatches. It intentionally uses only
hardware-free checks: native firmware tests, firmware builds, protocol vectors,
and Python loopback/UI tests. Hardware-in-the-loop evidence remains a manual
requirement and is never inferred from CI.

## Local Commands

Run the individual quality gates from the repository root:

```bash
just firmware-test
just firmware-build
just protocol-test
just protocol-lint
just protocol-typecheck
just companion-install
just companion-package
just companion-test
just jetson-test
just jetson-lint
just jetson-typecheck
```

`just test` runs all hardware-free test suites. `just check` additionally runs
the Ruff lint/type-check gates and compiles the normal OpenRB-150 image.

## CI Coverage

| Job | Checks |
| --- | --- |
| Firmware | Native Unity tests, normal OpenRB-150 build, output-disabled HIL build |
| Protocol | Python tests and golden vectors, Ruff, Pyright |
| Companion | Headless PySide6 test suite, Ruff, and source/wheel package build |
| Jetson | Shared-client loopback tests, Ruff, Pyright |

The CI firmware builds do not flash a board, enable DYNAMIXEL power, or claim
physical actuator behavior.