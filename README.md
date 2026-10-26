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

HYPR 0.5.0 is a prerelease for Julia 1.12 and is not registered in General.
For the standalone numerical search, install the release tag:

```sh
julia --project=hypr-demo -e 'using Pkg; Pkg.add(url="https://github.com/Space-FALCON-Lab/HYPR.jl.git", rev="v0.5.0")'
```

Then start Julia with `julia --project=hypr-demo` and run `using HYPR`.

For spacecraft or robot-arm planning, prepare a HYPR–SpaceAGORA environment;
see [Install with SpaceAGORA](docs/src/integration.md#install-with-spaceagora).
SpaceAGORA's setup command installs the HYPR revision your SpaceAGORA release
pins. For SpaceAGORA 0.7 that is not the `v0.5.0` tag, which admits only
SpaceAGORA 0.6.

## Try a first example

To try the standalone API from a clone of this repository:

```sh
git clone https://github.com/Space-FALCON-Lab/HYPR.jl.git
cd HYPR.jl
julia --project=. -e 'using Pkg; Pkg.instantiate()'
julia --project=. examples/bounded_search.jl
```

It minimizes a dimensionless quadratic and prints the best position and its cost.
For a physical example, the [robot-arm walkthrough](docs/src/robot_arm.md)
defines an arm and a target, plans a timed joint reference and checks its endpoint
error. For spacecraft rendezvous, start with SpaceAGORA's
[HYPR planning guide](https://github.com/Space-FALCON-Lab/SpaceAGORA.jl/blob/v0.7.1/docs/src/user/hypr_planning.md).

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

## Work on the package

From the HYPR checkout, build the documentation with:

```sh
julia --project=docs -e 'using Pkg; Pkg.develop(path="."); Pkg.instantiate()'
julia --project=docs docs/make.jl
```

Run the standalone tests with `julia --project=. -e 'using Pkg; Pkg.test()'`.
The [validation guide](docs/src/validation.md) describes the SpaceAGORA
integration, arithmetic and coverage checks and what each of them shows.

## Citation and credits

For the method, cite Jakob Gorisek-Gazze and Giusy Falcone, “HyPR: Safety- and
Fuel-Aware Motion Planning and Replanning for Autonomous SmallSat Servicing,”
accepted to iSpaRo 2026. The [method page](docs/src/method.md#citation) has the
reference and a BibTeX entry. With any results, record the exact HYPR and
SpaceAGORA revisions you used. [PROVENANCE.md](PROVENANCE.md) describes how the
package was extracted from SpaceAGORA; the original [MIT license](LICENSE) is kept.
