# Project Instructions for AI Agents

This file provides instructions and context for AI coding agents working on this project.

<!-- BEGIN BEADS INTEGRATION v:1 profile:minimal hash:ca08a54f -->
## Beads Issue Tracker

This project uses **bd (beads)** for issue tracking. Run `bd prime` to see full workflow context and commands.

### Quick Reference

```bash
bd ready              # Find available work
bd show <id>          # View issue details
bd update <id> --claim  # Claim work
bd close <id>         # Complete work
```

### Rules

- Use `bd` for ALL task tracking — do NOT use TodoWrite, TaskCreate, or markdown TODO lists
- Run `bd prime` for detailed command reference and session close protocol
- Use `bd remember` for persistent knowledge — do NOT use MEMORY.md files

## Session Completion

**When ending a work session**, you MUST complete ALL steps below. Work is NOT complete until `git push` succeeds.

**MANDATORY WORKFLOW:**

1. **File issues for remaining work** - Create issues for anything that needs follow-up
2. **Run quality gates** (if code changed) - Tests, linters, builds
3. **Update issue status** - Close finished work, update in-progress items
4. **PUSH TO REMOTE** - This is MANDATORY:
   ```bash
   git pull --rebase
   bd dolt push
   git push
   git status  # MUST show "up to date with origin"
   ```
5. **Clean up** - Clear stashes, prune remote branches
6. **Verify** - All changes committed AND pushed
7. **Hand off** - Provide context for next session

**CRITICAL RULES:**
- Work is NOT complete until `git push` succeeds
- NEVER stop before pushing - that leaves work stranded locally
- NEVER say "ready to push when you are" - YOU must push
- If push fails, resolve and retry until it succeeds
<!-- END BEADS INTEGRATION -->


## Build & Test

_Add your build and test commands here_

```bash
# Example:
# npm install
# npm test
```

## Architecture Overview

_Add a brief overview of your project architecture_

## Conventions & Patterns

_Add your project-specific conventions here_

## Current Config Hardware

Robot configuration is persisted by the SparkFun Qwiic OpenLog at `0x2A`
(`0x29` with its address jumper), not a 24LC32 EEPROM. `CONFIG.TXT` contains the
complete append-only config journal and `EVENTS.LOG` contains warnings, errors,
and retained crash records. The optional 1.3 inch Qwiic OLED is detected at
`0x3D` or `0x3C`; only `i2cTask` may update either device.

The OLED uses SparkFun's official `Qwiic1in3OLED` implementation from
SparkFun_Qwiic_OLED_Arduino_Library `v1.0.13`. Two detailed 5x7 views rotate
every 10 seconds and cover safety/fault, battery/RC, DXL, I2C/OpenLog, gait,
tuning/trim, and capture.

When OpenLog and SD are ready but `CONFIG.TXT` is missing or empty, firmware
creates and verifies its first record from the complete compiled default
`RobotConfig`. A non-empty invalid prefix is preserved, followed by a complete
default recovery record which must pass readback before config is persistent.
