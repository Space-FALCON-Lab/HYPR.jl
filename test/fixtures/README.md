# Arithmetic reference provenance

`arithmetic_parent.toml` records `Float64` bit patterns, the evaluation order and
the RNG draws that follow, taken from the extraction commit
`129abe677c6c26c83fc035eb07161871bcd50a69`, before the package's input validation
was added. The file also records the source hash and the Julia version. Tests only
read it. The inputs are in `../arithmetic_cases.jl`.

Both movement coefficients are nonzero. The tests can tell apart the serial
evaluation order, stopping before culling, the final unevaluated move, how
floating-point expressions are associated, and which random draw multiplies each
term of the particle update. Checking the RNG draws after each call also catches
changes that clamping or stopping would otherwise hide.

`../run_mutation_controls.py` applies six mutations, each in its own process,
without editing the source. Each mutation must fail an arithmetic assertion, not
a loading error, and the unchanged control must pass. Change these values only as
part of a reviewed change to the numerical policy; never regenerate them from the
implementation under test.
