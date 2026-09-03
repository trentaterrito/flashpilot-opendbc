# Sunnypilot MADS — development integration, runtime disabled

The C core is wired through the Ford adapter and native tests. There is no
production initializer or enabled vehicle feature. The state-machine bodies
remain unchanged. The reference-only heartbeat helper has internal linkage for
MISRA 8.7; the two linkage substitutions are documented in UPSTREAM.json and
reversed by tests before checking the original Git blobs.

Production uses an immediate host veto, not that helper's three-check delay.
Main/PCM/brake recovery cannot automatically grant independent authorization.
The superproject's FLASHPILOT_MADS_DESIGN.md lists unresolved Ford integrity,
cadence, hardware lifecycle, driver indication and release-packaging blockers.

This software is licensed under a custom license requiring permission for use.
This project uses software from Haibin Wen and SUNNYPILOT LLC and is licensed
under a custom license requiring permission for use. See LICENSE.md.
