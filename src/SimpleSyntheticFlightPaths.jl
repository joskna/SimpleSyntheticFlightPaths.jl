"""
    SimpleSyntheticFlightPaths

Generate synthetic UAV flight paths (wiggly line segments joined by smooth Bézier
bridges) and sample points on them with realistically varying spacing.

Main entry points: [`UAVplaneflight`](@ref), [`UAVPath`](@ref), [`sampleUAVPath`](@ref).
`length(path)` and `position(path, x)` extend `Base.length` / `Base.position`.
"""
module SimpleSyntheticFlightPaths

using LinearAlgebra
using Random
using StaticArrays

import Base: length, position

export UAVPathSegment, UAVLineSegment, UAVBridgeSegment, UAVPath,
    UAVplaneflight, sampleUAVPath,
    randomwigglevectorspectrum, get_cubic_bezier_point, bridge_3d_curves

include("geometry.jl")
include("segments.jl")
include("path.jl")
include("sampling.jl")

end # module