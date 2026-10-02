# SimpleSyntheticFlightPaths.jl

Julia library for generating synthetic UAV flight paths and sampling them with realistic, speed-varying spacing.

It builds 3D "lawnmower" survey paths from wiggly line segments joined by smooth Bézier bridges, then samples points on them the way a real UAV would: the temporal sample rate is fixed, but the vehicle's speed varies, so the spacing between samples fluctuates around a nominal value. Every sample lies exactly on the path, and all randomness can be seeded for reproducible output.

## Installation

The package is not registered. Install it directly from a local checkout or from GitHub:

```julia
using Pkg
Pkg.develop(path="path/to/SimpleSyntheticFlightPaths")
# or
Pkg.add(url="https://github.com/<your-username>/SimpleSyntheticFlightPaths.jl")
```

Requires Julia 1.9 or newer.

## Quick start

```julia
using SimpleSyntheticFlightPaths, Random

# Sweep between (0, 0) and (100, 50), climbing from z = 10 to z = 30 in steps of 5.
path = UAVplaneflight(0.0, 100.0, 0.0, 50.0, 10.0, 30.0, 5.0,
                      [2.0, 1.0, 0.5],   # wiggle magnitudes on the line segments
                      [1.0, 0.5];        # wiggle magnitudes on the bridges
                      usedeterministicseed=true)

length(path)             # total arc length
position(path, 12.3)     # 3D point 12.3 units along the path

# Sample with a nominal spacing of 1.0 and a fixed seed
pts = sampleUAVPath(path, 1.0; rng=Xoshiro(42))
```

`pts` is a `Vector{SVector{3,Float64}}` starting at the path start.

## Plotting

Plotting is not part of the package, but the output works directly with [Makie.jl](https://docs.makie.org):

```julia
using GLMakie  # or CairoMakie for static output

fig = Figure()
ax = Axis3(fig[1, 1]; aspect=:data)
scatter!(ax, pts; markersize=6)
lines!(ax, pts; linewidth=1, color=(:gray, 0.5))
fig
```

## How it works

**Path construction.** `UAVplaneflight` creates one `UAVLineSegment` per altitude level, alternating direction on each pass. Consecutive lines are joined by a `UAVBridgeSegment`, a cubic Bézier curve with G1 (tangent) continuity, so the path climbs between levels without sharp corners. Each segment carries a random *wiggle spectrum*: a sum of oscillations with frequency proportional to the index, shaped by `sin(πt)^0.7` so the perturbation vanishes at segment ends.

**Sampling.** `sampleUAVPath` advances along the path by `ds * (1 + deviation)` per sample, where `deviation` follows an AR(1) process. Higher `speedcorrelation` gives smoother, more physical speed changes, and `0` gives independent jitter. A minimum step guarantees forward progress. Each sample is evaluated through `position(path, x)`, so no interpolation is involved and every point is exactly on the path.

## API

| Function / type | Description |
|---|---|
| `UAVplaneflight(startx, endx, starty, endy, startz, endz, Δz, wig, wigbridge; usedeterministicseed=false)` | Generate a full lawnmower flight path |
| `sampleUAVPath(path, ds; jitter=0.3, speedcorrelation=0.9, rng, includeend=false)` | Sample points along a path with varying spacing |
| `UAVPath(segments)` | A sequence of segments with cumulative lengths |
| `UAVLineSegment(start, stop, wigglevectorspectrum)` | Straight, wiggly segment |
| `UAVBridgeSegment(segmentA, segmentB, wigglevectorspectrum; tension=5.0, steps=200)` | Smooth Bézier connection between two segments |
| `length(path)` | Total arc length (extends `Base.length`) |
| `position(path, x)` | Point at arc length `x` (extends `Base.position`) |
| `randomwigglevectorspectrum(mags; rng)` | Random wiggle vectors for given magnitudes |
| `get_cubic_bezier_point(p0, p1, p2, p3, t)` | Evaluate a cubic Bézier curve |
| `bridge_3d_curves(p_a, t_a, p_b, t_b; tension, steps)` | Discretized G1 bridge between two curve ends |

### `sampleUAVPath` options

| Keyword | Default | Meaning |
|---|---|---|
| `jitter` | `0.3` | Relative standard deviation of the step size (about ±30 % of `ds`) |
| `speedcorrelation` | `0.9` | Correlation of speed fluctuation between consecutive samples, in `[0, 1)` |
| `rng` | `Random.default_rng()` | Random number generator, e.g. `Xoshiro(1234)` for reproducibility |
| `includeend` | `false` | Append the exact path end point as the last sample |

## Notes and limitations

- `ds` is the spacing in the path's arc-length parametrization, so it is "roughly" the Euclidean distance between consecutive samples. The wiggle adds length that `length(path)` does not account for, and the Bézier length is discretized.
- Wiggle offsets are applied on top of the straight or Bézier base curve, so very large wiggle magnitudes relative to segment length can distort the path noticeably.

## Testing

```julia
using Pkg
Pkg.test("SimpleSyntheticFlightPaths")
```

## License

Add your license here (e.g. MIT).
