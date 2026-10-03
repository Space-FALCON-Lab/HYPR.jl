# Validation and limits

The extraction preserves source calculations and operation order, with a targeted
exception: search code is called through the new package boundary. The standalone
serial search is extracted from the robot-arm loop. RPO calls the core particle
update kernel and shared swarm policy; its specialized outer schedule remains in
the optional extension. These variants are not silently unified.

The accepted comparison consists of four bounded RPO cases and two robot-arm
cases, including the random values consumed after each result. Its serialized
fingerprint is `9b25466fd812cf70330d3379a7b2071bcea0b93c3c364051959e698cdc3aed38`.
The candidate reproduces that fingerprint byte-for-byte. Larger integration and
independent review are separately recorded; this hash is not full mission,
performance, arbitrary-thread, native atmosphere or paper-claim acceptance.

Preserve the known RRT adaptive sampling limitation, retimer fallback warnings,
shared optimizer dependencies and the finite scope of previous physics tests.
No numerical acceptance threshold is changed by packaging. Wall-clock timeout
runs can stop at different points and are not claimed bitwise repeatable.
