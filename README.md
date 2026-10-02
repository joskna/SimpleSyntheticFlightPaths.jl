# SimpleSyntheticFlightPaths.jl
Julia library for generating synthetic UAV flight paths and sampling them with realistic, speed-varying spacing.

SimpleSyntheticFlightPaths.jl generates synthetic 3D UAV flight paths, such as lawnmower survey patterns with wiggly line segments joined by smooth Bézier bridges. It can also sample points on those paths with randomly varying spacing, mimicking a vehicle whose speed fluctuates while the sample rate stays constant. All samples lie exactly on the path, and results are reproducible via seeded RNGs.

julia
using SimpleSyntheticFlightPaths, Random

path = UAVplaneflight(0.0, 100.0, 0.0, 50.0, 10.0, 30.0, 5.0,
                      [2.0, 1.0, 0.5], [1.0, 0.5]; usedeterministicseed=true)
pts = sampleUAVPath(path, 1.0; rng=Xoshiro(42))
