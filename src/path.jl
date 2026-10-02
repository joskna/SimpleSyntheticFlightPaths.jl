"""
    UAVPath(segments::Vector{UAVPathSegment})

A sequence of segments with precomputed cumulative arc lengths for fast lookup.
"""
struct UAVPath
    segments::Vector{UAVPathSegment}
    lengths::Vector{Float64}
end

function UAVPath(segments::Vector{UAVPathSegment})
    return UAVPath(segments, cumsum(Float64[length(s) for s in segments]))
end

"""Total arc length of the path."""
length(path::UAVPath) = path.lengths[end]

"""
    position(path::UAVPath, x)

3D position at total arc-length `x` from the start. Throws if `x` is outside `[0, length(path)]`.
"""
function position(path::UAVPath, x::Real)
    (x < 0 || x > path.lengths[end]) && error("Position out of range")
    i = searchsortedfirst(path.lengths, x)
    offset = i == 1 ? x : x - path.lengths[i-1]
    return position(path.segments[i], offset)
end

"""
    UAVplaneflight(startx, endx, starty, endy, startz, endz, Δz, wig, wigbridge;
                   usedeterministicseed=false)

"Lawnmower" path sweeping between `(startx, starty)` and `(endx, endy)` at altitudes
`startz:Δz:endz`, alternating direction each pass. Line segments use the `wig` magnitude
spectrum, joined by smooth bridges using `wigbridge`. With `usedeterministicseed=true`
the RNG seed is fixed (1234) for reproducible output.
"""
function UAVplaneflight(startx, endx, starty, endy, startz, endz, Δz, wig, wigbridge;
                        usedeterministicseed=false)
    rng = usedeterministicseed ? Xoshiro(1234) : Xoshiro()

    linesegmentcount = floor(Int, (endz - startz) / Δz) + 1
    bridgecount = linesegmentcount - 1

    segments = Vector{UAVPathSegment}(undef, linesegmentcount + bridgecount)
    forwarddirection = true

    for k in 1:linesegmentcount
        wiggle = randomwigglevectorspectrum(wig, rng=rng)
        z = startz + (k - 1) * Δz
        startpoint = forwarddirection ? [startx, starty, z] : [endx, endy, z]
        endpoint   = forwarddirection ? [endx, endy, z]     : [startx, starty, z]
        segments[2k-1] = UAVLineSegment(startpoint, endpoint, wiggle)
        forwarddirection = !forwarddirection
    end

    for k in 1:bridgecount
        wigglebridge = randomwigglevectorspectrum(wigbridge, rng=rng)
        segments[2k] = UAVBridgeSegment(segments[2k-1], segments[2k+1], wigglebridge)
    end

    return UAVPath(segments)
end
