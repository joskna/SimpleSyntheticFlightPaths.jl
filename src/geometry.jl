"""
    randomwigglevectorspectrum(wigglemags; rng=Random.default_rng())

For each magnitude in `wigglemags`, a random 3D direction vector (Gaussian components)
scaled by that magnitude. Seeds the pseudo-random perturbation along a segment.
"""
function randomwigglevectorspectrum(wigglemags; rng=Random.default_rng())
    return [mag * SVector{3}(randn(rng, Float64, 3)) for mag in wigglemags]
end

"""
    addoffsetfromwiggle(pos, t, wigglevectorspectrum)

Adds oscillating wiggle offsets to `pos` at normalized parameter `t ∈ [0, 1]`. Entry `k`
contributes a rotating complex-phase offset of frequency proportional to `k`, shaped by
`sin(pi * t)^0.7` so the wiggle vanishes at both segment endpoints. Returns the real part.
"""
function addoffsetfromwiggle(pos, t, wigglevectorspectrum)
    for (k, wiggle) in enumerate(wigglevectorspectrum)
        pos += wiggle * cispi(0.5 * k * t) * complex(sin(pi * t))^0.7
    end
    return real.(pos)
end

"""
    get_cubic_bezier_point(p0, p1, p2, p3, t)

Point on a cubic Bézier curve for `t ∈ [0, 1]` (start `p0`, controls `p1`/`p2`, end `p3`).
"""
function get_cubic_bezier_point(p0, p1, p2, p3, t)
    mt = 1.0 - t
    return (mt^3) .* p0 + (3.0 * mt^2 * t) .* p1 + (3.0 * mt * t^2) .* p2 + (t^3) .* p3
end

"""
    bridge_3d_curves(p_a, t_a, p_b, t_b; tension, steps=20)

Discretized G1-continuous cubic Bézier bridge between two curve ends.
`t_a`, `t_b` are normalized tangents at `p_a`, `p_b`; `tension` scales the curvature.
"""
function bridge_3d_curves(p_a, t_a, p_b, t_b; tension, steps=20)
    k = norm(p_b - p_a) * 0.33 * tension
    c_a = p_a + (t_a .* k)
    c_b = p_b - (t_b .* k)
    return [get_cubic_bezier_point(p_a, c_a, c_b, p_b, i / steps) for i in 0:steps]
end
