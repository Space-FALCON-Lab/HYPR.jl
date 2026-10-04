# Arithmetic reference provenance

`arithmetic_parent.toml` records Float64 bit patterns, evaluation order and subsequent
RNG draws from the independently accepted extraction commit
`129abe677c6c26c83fc035eb07161871bcd50a69`, before package 1 validation changes.
The source hash and Julia runtime are embedded in the file. Normal test execution
only reads it. The fixture inputs live in `../arithmetic_cases.jl`.

Both movement coefficients are nonzero. Tests distinguish serial evaluation order,
stopping before culling, the final unevaluated movement, floating-point expression
association and which random draw multiplies each particle-update term. Checks of
post-call RNG draws also detect changes hidden by clamping or stopping.

`run_mutation_controls.py` applies the six preserved review mutations in separate
processes without editing source. An expected rejection must be an arithmetic
assertion failure, not a loader error. The unchanged control must pass. Updating
these values requires a separately reviewed numerical-policy change; they are not
regenerated from whichever implementation is under test.
