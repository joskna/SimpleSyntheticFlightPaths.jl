"""Abstract supertype of all path segments. Subtypes implement `length` and `position`."""
abstract type UAVPathSegment end

# ---------------------------------------------------------------- line segment

"""
    UAVLineSegment(start, stop, wigglevectorspectrum)

Straight segment from `start` to `stop` (any 3-element indexable) with a wiggle offset.
"""
struct UAVLineSegment <: UAVPathSegment
    startpoint::SVector{3,Float64}
    endpoint::SVector{3,Float64}
    diffvec::SVector{3,Float64}
    wigglevectorspectrum::Vector{SVector{3,Float64}}
end

function UAVLineSegment(start::SVector{3,Float64}, stop::SVector{3,Float64},
                        wigglevectorspectrum::Vector{SVector{3,Float64}})
    return UAVLineSegment(start, stop, stop - start, wigglevectorspectrum)
end

function UAVLineSegment(start, stop, wigglevectorspectrum::Vector{SVector{3,Float64}})
    return UAVLineSegment(SVector{3,Float64}(start), SVector{3,Float64}(stop), wigglevectorspectrum)
end

"""Euclidean length of the straight segment (without wiggle)."""
length(segment::UAVLineSegment) = norm(segment.diffvec)

"""
    position(segment::UAVLineSegment, x)

Position at distance `x` from the start, including the wiggle offset.
"""
function position(segment::UAVLineSegment, x::Real)
    t = x / length(segment)
    pos = ComplexF64.(segment.startpoint + t * segment.diffvec)
    return addoffsetfromwiggle(pos, t, segment.wigglevectorspectrum)
end

# ---------------------------------------------------------------- bridge segment

"""
    UAVBridgeSegment(segmentA, segmentB, wigglevectorspectrum; tension=5.0, steps=200)

Cubic Bézier bridge from the end of `segmentA` to the start of `segmentB` with G1
(tangent) continuity. Tangents are estimated by sampling near each endpoint; control
points are scaled by `tension` and the endpoint distance. The arc length is approximated
with `steps` straight sub-segments.
"""
struct UAVBridgeSegment <: UAVPathSegment
    startpoint::SVector{3,Float64}
    endpoint::SVector{3,Float64}
    controlA::SVector{3,Float64}
    controlB::SVector{3,Float64}
    wigglevectorspectrum::Vector{SVector{3,Float64}}
    len::Float64
end

function UAVBridgeSegment(segmentA, segmentB, wigglevectorspectrum; tension=5.0, steps::Integer=200)
    l = length(segmentA) - min(0.1, 0.01 * length(segmentA))
    t1 = segmentA.endpoint - position(segmentA, l)
    t1 = t1 / norm(t1)

    l = min(0.1, 0.01 * length(segmentB))
    t2 = position(segmentB, l) - segmentB.startpoint
    t2 = t2 / norm(t2)

    p_a = segmentA.endpoint
    p_b = segmentB.startpoint

    k = norm(p_b - p_a) * 0.33 * tension
    c_a = SVector{3}(p_a + t1 .* k)
    c_b = SVector{3}(p_b - t2 .* k)

    len = 0.0
    prev = SVector{3,Float64}(p_a)
    for i in 1:steps
        cur = SVector{3,Float64}(get_cubic_bezier_point(p_a, c_a, c_b, p_b, i / steps))
        len += norm(cur - prev)
        prev = cur
    end
    return UAVBridgeSegment(p_a, p_b, c_a, c_b, wigglevectorspectrum, len)
end

"""Precomputed (discretized) arc length of the bridge."""
length(segment::UAVBridgeSegment) = segment.len

"""
    position(segment::UAVBridgeSegment, x)

Position at arc-length `x` from the start on the Bézier curve, plus wiggle.
"""
function position(segment::UAVBridgeSegment, x::Real)
    t = x / length(segment)
    pos = ComplexF64.(get_cubic_bezier_point(segment.startpoint, segment.controlA,
                                             segment.controlB, segment.endpoint, t))
    return addoffsetfromwiggle(pos, t, segment.wigglevectorspectrum)
end
