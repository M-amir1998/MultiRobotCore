# ADR 0005 — Single-phase auction without epsilon scaling

**Status:** accepted

**Context.** The auction handles rectangular problems and infeasible pairs by
giving every robot a private "stay unassigned" object, which makes the
problem asymmetric (more objects than bidders). An implementation with
epsilon scaling violated the `n eps` optimality bound on random instances.

**Decision.** Use a single forward-auction phase. For asymmetric problems the
bound also requires objects that end unassigned to have zero price; a single
phase guarantees this, scaling phases that restart from non-zero prices do
not. Forward/reverse auctions (Bertsekas & Castañón 1992) would restore
scaling and are future work.

**Consequences.** The `n eps` bound holds and is tested on 200 random
instances. Without scaling the auction can need many rounds when robots
outnumber tasks; the benchmark reports this.
