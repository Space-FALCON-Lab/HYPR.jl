# HYPR.jl

HYPR owns the search implementation and its optional SpaceAGORA adapters. The
core exposes particle-swarm search with explicit objective, culling and stopping
policies. It can run without SpaceAGORA. The optional extension retains the
existing spacecraft RPO and robot-arm specializations when SpaceAGORA is loaded.

**Version:** 0.1.1, the optional-objective prerelease, for Julia 1.12. Scientific acceptance
is limited to the tests and matched results described in the validation guide.
Install from the versioned Git source; the package is not registered in General.

```sh
julia --project=hypr-demo -e 'using Pkg; Pkg.add(url="https://github.com/Space-FALCON-Lab/HYPR.jl.git", rev="v0.1.1")'
```

The standalone core needs no SpaceAGORA checkout. Configured RPO and robot-arm
planning requires the compatible SpaceAGORA installation described below.

## Run the standalone example

From a checkout of this repository:

```sh
julia --project=. -e 'using Pkg; Pkg.instantiate()'
julia --project=. examples/bounded_search.jl
julia --project=. -e 'using Pkg; Pkg.test()'
```

The example minimizes a dimensionless quadratic. It demonstrates the API, not a
new spacecraft maneuver, robot-arm accuracy envelope or paper reproduction.

## Read the documentation

- [Getting started](docs/src/index.md)
- [Search API and ownership](docs/src/api.md)
- [SpaceAGORA integration](docs/src/integration.md)
- [Validation and limitations](docs/src/validation.md)
- [Migration and compatibility](docs/src/migration.md)

Build the standalone site with:

```sh
julia --project=docs -e 'using Pkg; Pkg.develop(path="."); Pkg.instantiate()'
julia --project=docs docs/make.jl
```

See [PROVENANCE.md](PROVENANCE.md) for source attribution and the retained MIT
license. Use the release commit as the software citation until a verified
manuscript/DOI citation is supplied; no bibliographic identity is invented here.
