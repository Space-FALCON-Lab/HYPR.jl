# Provenance and licensing

Extracted from Space-FALCON Lab's SpaceAGORA.jl, commit
`9be384a20dfe28594a35341eed18e6fe945f1049`, specifically
`packages/SpaceAGORAHYPR/src/`. The original MIT license and copyright are retained
in LICENSE. Git history and the extraction receipt identify the source; this new
repository does not pretend to contain the original development history.

Most domain implementation files are unchanged. The shared swarm policy has one
owner in the core and compatibility forwarders in the extension. The robot-arm
iteration loop now calls the standalone core search; RPO calls its core movement
kernel. The source comparison and retained seeded results identify these changes.

The [method and citation guide](docs/src/method.md#citation) records the confirmed
HyPR manuscript by Jakob Gorisek-Gazze and Giusy Falcone. Cite that paper for the
method and record the exact software revisions for an experiment. The citation
does not imply that this package reproduces the complete paper campaign.
