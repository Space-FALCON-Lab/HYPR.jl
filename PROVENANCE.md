# Provenance and licensing

Extracted from Space-FALCON Lab's SpaceAGORA.jl, commit
`9be384a20dfe28594a35341eed18e6fe945f1049`, specifically
`packages/SpaceAGORAHYPR/src/`. The original MIT license and copyright are retained
in LICENSE. This repository starts from that extraction and does not include the
earlier development history, which stays in SpaceAGORA.jl.

Most domain files are unchanged. The shared swarm policy now lives in the core,
with forwarding functions in the extension. The robot-arm iteration loop now calls
the standalone core search, and RPO calls the core movement kernel. The seeded
comparison described in the [validation guide](docs/src/validation.md) checks that
these changes kept the results.

The [method and citation guide](docs/src/method.md#citation) gives the reference
for the HyPR paper by Jakob Gorisek-Gazze and Giusy Falcone. Cite the paper for
the method and record the exact software revisions for an experiment. Citing it
does not mean this package reproduces the paper's complete campaign.
