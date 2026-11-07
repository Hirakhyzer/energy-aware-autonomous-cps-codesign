# Research questions, hypotheses, and contribution boundaries

**RQ1.** Can joint sensing, processor, and control adaptation reduce total modeled mission energy compared with fixed high-rate operation?

**RQ2.** Does cross-layer adaptation outperform independent sensing-only, computing-only, or control-only adaptation under identical battery and mission constraints?

**RQ3.** What is the energy–completion-time–tracking-error trade-off, including mode switching and processor decision overhead?

**RQ4.** How robust are feasible candidate rankings when process noise, movement load, battery reserve, and mission deadline change?

## Hypotheses to test (not established findings)

- H1: The joint policy uses less energy than fixed operation on at least some specified scenarios.
- H2: Joint adaptation improves over individual ablations when cross-layer interactions matter.
- H3: Constraint violations and mission failures can be explicitly attributed to estimation, compute, control, battery, or timing limits.
- H4: Calibrated hardware energy models improve the agreement between predicted and measured energy.

## Novelty caution

Energy-aware CPS co-design, DVFS, event-triggered sensing, and joint robotic computing/motion optimization are established fields. This implementation does **not** claim a novel method just because it combines them. A publication requires careful literature comparison, a genuinely differentiated method, and out-of-sample evidence. The present finite-grid baseline is an experimental starting point, not a publishable novelty claim.
