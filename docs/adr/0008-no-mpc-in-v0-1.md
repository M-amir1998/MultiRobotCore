# ADR 0008 — Distributed MPC deferred from v0.1

**Status:** accepted

**Context.** Distributed MPC is the lowest-priority item of the project scope,
to be included only if quality permits, and must use a professional QP solver
(e.g. OSQP) rather than a hand-written one. A credible implementation needs a
centralised MPC baseline, a genuinely decomposed variant (e.g. sequential
distributed MPC exchanging predicted trajectories, Richards & How 2007),
linearised separation constraints, and validation of feasibility and
closed-loop safety.

**Decision.** v0.1 ships without MPC. The build already contains an opt-in
OSQP dependency hook (`MULTIROBOTCORE_WITH_OSQP`) for the follow-up.

**Consequences.** The status matrix lists MPC as not implemented, and no claim
of distributed MPC is made anywhere in the repository.
