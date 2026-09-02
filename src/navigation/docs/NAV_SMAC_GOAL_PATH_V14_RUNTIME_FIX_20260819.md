# V14 — Runtime Smac Goal Path Fix

Latest runtime logs showed the `/goal_pose` bridge receiving free-space clicks,
but PlannerServer aborted before `/smac_plan` could be produced. One run reported
`Starting point in lethal space`; another reported `no valid path found` while
Smac Hybrid was configured as forward-only `DUBIN`. BtNavigator also failed
activation because BehaviorServer exposed only `wait` and `/spin` did not exist.

V14 changes the planning-only `/nav_map`, not the raw `/map` used by AMCL. Tiny
isolated mapping speckles are filtered and the known initial AGV footprint is
cleared from `/nav_map` to remove self-map artefacts at the planner start.

The primary and fallback global planners are both
`nav2_smac_planner/SmacPlannerHybrid`, both preserve the physical 2.0 m minimum
turning radius, and both use `REEDS_SHEPP`. The fallback relaxes search costs and
endpoint tolerance but does not change robot geometry.

The bridge explicitly computes the path first and publishes it on `/smac_plan`.
Only a successful path is forwarded to NavigateToPose, with a behavior tree that
uses the same planner profile.
