# HYPR.jl

HYPR searches for motion paths for spacecraft rendezvous and robot arms in
SpaceAGORA. It improves candidate paths with particle-swarm optimization;
SpaceAGORA supplies the geometry, physical models and simulation. The numerical
search library also works on its own with an objective supplied by the caller.

The spacecraft method is described in *HyPR: Safety- and Fuel-Aware Motion
Planning and Replanning for Autonomous SmallSat Servicing*. The package's
validation covers selected search and integration cases. Reproducing the paper's
complete static and dynamic campaigns remains separate work.

## Install

HYPR **0.4.0** is a prerelease for **Julia 1.12** and is not registered in General.
For the standalone numerical search, install the versioned source:

```sh
julia --project=hypr-demo -e 'using Pkg; Pkg.add(url="https://github.com/Space-FALCON-Lab/HYPR.jl.git", rev="v0.4.0")'
```

For spacecraft or robot-arm planning, use the
[SpaceAGORA setup instructions](docs/src/integration.md#install-with-spaceagora).
They install the compatible packages together in a separate environment.

## Try a first example

To explore the standalone API, clone this repository and run the small search:

```sh
git clone https://github.com/Space-FALCON-Lab/HYPR.jl.git
cd HYPR.jl
julia --project=. -e 'using Pkg; Pkg.instantiate()'
julia --project=. examples/bounded_search.jl
```

It prints the best position and cost for a dimensionless quadratic. For a
physical planning example, follow the [robot-arm walkthrough](docs/src/robot_arm.md):
define an arm and target, create a timed joint reference, and inspect its endpoint
error. For spacecraft rendezvous, start with SpaceAGORA's
[HYPR planning guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/CORE_V050_SHA/docs/src/user/hypr_planning.md).

## Documentation

| Question | Guide |
| --- | --- |
| What does HYPR do? | [Overview and first steps](docs/src/index.md) |
| How do I plan an arm motion? | [Robot-arm walkthrough](docs/src/robot_arm.md) |
| How does it connect to SpaceAGORA? | [Installation and integration](docs/src/integration.md) |
| Which versions go together? | [Compatibility and source records](docs/src/integration.md#compatibility-and-source-records) |
| How do I supply a search objective? | [Search API](docs/src/api.md) |
| What has been validated? | [Validation and limitations](docs/src/validation.md) |
| How does the software relate to the paper? | [Method and citation](docs/src/method.md) |
| How do I update an older installation? | [Migration](docs/src/migration.md) |

Build the documentation locally with:

```sh
julia --project=docs -e 'using Pkg; Pkg.develop(path="."); Pkg.instantiate()'
julia --project=docs docs/make.jl
```

Run the standalone tests with `julia --project=. -e 'using Pkg; Pkg.test()'`.
The [validation guide](docs/src/validation.md) describes the separate adapter and
arithmetic checks and the limits of their evidence.

## Citation and credits

For the method, cite Jakob Gorisek-Gazze and Giusy Falcone, “HyPR: Safety- and
Fuel-Aware Motion Planning and Replanning for Autonomous SmallSat Servicing,”
accepted to iSpaRo 2026. The [method page](docs/src/method.md#citation) provides
the reference and BibTeX. Record the exact HYPR and SpaceAGORA revisions alongside
experimental results. [PROVENANCE.md](PROVENANCE.md) describes the extraction from
Space-FALCON Lab's SpaceAGORA and the retained [MIT license](LICENSE).
