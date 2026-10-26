# Validation and limits

The [method and paper guide](method.md) describes the HyPR manuscript and its
static and dynamic experiments. The tests on this page do not reproduce those
experiments. In particular, SpaceAGORA's current research comparison tracker and
its random-seed mapping differ from the manuscript, so agreement with the paper's
aggregate results is not claimed.

When the search was moved out of SpaceAGORA into this package, the calculations
and their order of operations were kept, except that search code is now called
across the package boundary. The standalone serial search comes from the
robot-arm loop. RPO calls the core particle-update kernel and the shared swarm
policy, while its own outer schedule stays in the optional extension. The two
schedules are deliberately kept separate.

At extraction, four bounded RPO cases and two robot-arm cases, including the
random values drawn after each result, were compared with the original code. The
serialized comparison has the fingerprint
`9b25466fd812cf70330d3379a7b2071bcea0b93c3c364051959e698cdc3aed38`, and the
extracted code reproduced it byte for byte. That comparison covers those six
cases only; it is not an acceptance of full missions, of arbitrary thread counts,
of the native atmosphere or of the paper's claims.

Some known limits carry over unchanged: the RRT adaptive sampling limitation, the
retimer's fallback warnings, the shared optimizer dependencies and the finite
scope of the earlier physics tests. Moving the code into a package changed no
numerical acceptance threshold. Runs that stop on a wall-clock timeout can stop at
different points, so they are not bitwise repeatable.

## Arithmetic regression tests

`test/arithmetic_regressions.jl` compares exact `Float64` bit patterns, and the
random draws that follow, with values recorded from the search kernels at commit
`129abe677c6c26c83fc035eb07161871bcd50a69` under Julia 1.12.3. The fixture,
`test/fixtures/arithmetic_parent.toml`, covers serial search state, evaluation
order, culling and stopping, and the threaded particle update with one RNG per
particle. Both acceleration terms are nonzero. CI only reads the fixture; it never
regenerates it.

`python3 test/run_mutation_controls.py` runs the unchanged kernels as a positive
control and six deliberately altered kernels, each in its own process. The
alterations change evaluation order, culling before stopping, the final move,
floating-point operation order and which random draw goes to which term. Each
altered kernel must fail an arithmetic assertion; a package loading error does not
count. CI runs the controls on one and four threads. Exact bit comparisons hold
for this fixture and the supported runtime; they say nothing about other
platforms or other RNGs.

## Coverage of core and integration code

The coverage gate lists every Julia source file in `src/` and `ext/`, checks its
hash, and merges line counts from the core tests (one and four threads) with
SpaceAGORA's own RPO and robot-arm tests, run from the selected SpaceAGORA
checkout with HYPR loaded (`test/extension_coverage.jl`). Those tests also compare
the configured sampling wrappers with the shared kernels they call.

The gate requires 90% line coverage overall and 80% in every file with executable
lines. No file is excluded. `src/HYPR.jl` only wires the module together and has
no measured lines; it stays in the list without a percentage. Julia leaves a
function that was never compiled unmeasured, so the gate lists named functions,
anonymous `function` forms, arrow closures and `do` blocks, and rejects any whose
body has no measurement at all instead of dropping it from the total. For a
closure the gate inspects the body, not the line that creates or calls it; for a
named definition it includes the declaration line, because Julia can attribute a
short-form body to it. This is line coverage: it cannot tell apart two
expressions on one line, and it is neither branch coverage nor scientific
acceptance.

The configured RRT* wrapper always supplies a refinement callback, so its
fallback cost callback never runs in normal configured use.
`test/rrt_callback_contract.jl` checks, in a separate process, that the supplied
callback agrees with the configured polyline objective and then delegates to the
unchanged shared engine. This checks the callback contract only; it does not
create a fallback path for configured RRT* in production.

To measure coverage yourself, run these commands from the HYPR checkout you want
to measure. Pick a compatible SpaceAGORA checkout from the
[integration guide](integration.md), then prepare two separate environments: the
integration one, which `test/setup_integration.jl` points at this HYPR checkout,
and a core one that loads HYPR without SpaceAGORA.

```sh
julia --startup-file=no test/setup_integration.jl /path/to/SpaceAGORA.jl /path/to/integration-env
julia --startup-file=no -e 'using Pkg; Pkg.activate("/path/to/core-env"); Pkg.develop(path=pwd())'
python3 scripts/run_coverage.py --core-project /path/to/core-env \
    --integration-project /path/to/integration-env --output /path/to/coverage-output
```

The runner needs Python 3.11 or later and an empty output directory. It clears
old counters for this checkout, records each stage's exit status, stops at the
first failed stage, and keeps the raw counters next to the report. The report
lists every file, its counts and the thresholds. `python3 test/test_coverage_gate.py`
checks that the gate rejects low coverage, missing or stale counters, missing
source, unmeasured functions and failed stages.

## RPO objective compatibility

`test/custom_objective.jl` checks the [optional objective callback](integration.md#Optional-RPO-objective):
callback ranking, the straight-path branch, every refinement operation, result
metadata, malformed return values, and the default objective against HYPR 0.1.0
(`6b2af3b4129907d76f6cdc873267a57445c1334b`). The fixtures in
`test/fixtures/rpo_defaults_0_1_0.jl` hold eight legacy and manuscript cases with
path, cost, components, cost history, refinement status and the exact RNG
consumption. Floating-point results are compared with a relative and absolute
tolerance of 1e-12 to allow for differences between numerical libraries. The four
legacy cases are still checked against those fixtures. Manuscript mode has since
changed (see below), so its four cases are kept as a record of the earlier
implementation; the tests now check manuscript mode for seeded determinism,
preserved endpoints and a consistent objective.

`test/jakob_comparison.jl` runs the unmodified CPU comparison code from SpaceAGORA
commit `fa2e4a411c16c0587230b2020881a36065887b07`, after checking the hashes of
all its source files. Small clear cases with zero and one waypoints exercise both
the original and the retimed-fuel PSO routes and the coupled tracker that ends
each run. The acceleration limit of 0.00625 m/s² comes from that comparison's
configuration. The test checks compatibility; it does not reproduce the paper's
campaign or any reinforcement-learning training.

## Manuscript mode

Setting `hypr_mode=:manuscript` in SpaceAGORA's `RPOPSOConfig` selects the
formulation reconstructed from the final iSpaRo paper. In the extension's RPO
search this mode:

- moves the inertia and acceleration coefficients to the paper's absolute end
  values `(0.455, 1.05, 1.75)` along a cubic schedule over the first half of the
  iterations;
- culls every ten iterations, tries whole-particle replacements on stagnation,
  bounds re-exploration and evaluates the final particle update;
- imposes zero reference velocity at departure and arrival in the ordinary
  retimer.

Legacy mode and the standalone swarm kernel are unchanged. Two settings are
reconstruction choices, because the paper does not give them: whole-particle
trials are drawn uniformly, and the swarm grows by a factor of 1.5. Do not present
them as recovered original run settings. `test/final_isparo_swarm.jl` checks the
coefficient end values and the culling boundaries in the integration environment.

The full reconstruction of the paper experiment, with its saved inputs, is kept
in the private SpaceAGORARPOStudies repository; see SpaceAGORA's
[motion-planning guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/v0.7.1/docs/src/user/hypr_planning.md)
for what it covers and what has not been recovered from the original runs. None
of the tests here reproduce the paper's static and dynamic campaigns.
