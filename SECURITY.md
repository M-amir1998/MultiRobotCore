# Security policy

MultiRobotCore is a research and engineering library for multi-robot
coordination. It is **not certified for safety-critical deployment**: the
collision-avoidance and formation safety terms carry the guarantees stated in
their documentation and no more, and simulation results do not validate a
physical system.

## Supported versions

| Version | Supported |
|---|---|
| 0.1.x | yes |

## Reporting a vulnerability

Please report security issues privately through GitHub's
"Report a vulnerability" (security advisories) on this repository rather than
in a public issue. Include the affected version, a description, and steps or
inputs to reproduce. You can expect an acknowledgement within a week.

Relevant classes of issues include memory-safety defects (the CI runs
AddressSanitizer and UndefinedBehaviorSanitizer), unbounded resource use on
untrusted inputs (map files, configuration files, cost matrices) and
supply-chain concerns in the build (dependencies are pinned by SHA-256).
