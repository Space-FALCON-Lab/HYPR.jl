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

This package candidate adds no authorship claims or unverified publication links.
A formal manuscript citation should be added only from an approved bibliographic
record, rather than inferred from filenames or an agent summary.
