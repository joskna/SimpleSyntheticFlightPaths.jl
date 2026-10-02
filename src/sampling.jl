"""
    sampleUAVPath(path::UAVPath, ds::Real; jitter=0.3, speedcorrelation=0.9,
                  rng=Random.default_rng(), includeend=false)

Samples points along `path` with arc-length spacing roughly `ds` but varying randomly,
mimicking a UAV whose speed fluctuates while the temporal sample rate stays fixed.
Every sample is evaluated through `position(path, x)`, so all lie exactly on the path.

# Keyword arguments
- `jitter`: relative standard deviation of the step size (0.3 → about ±30 % of `ds`).
- `speedcorrelation ∈ [0, 1)`: correlation of speed fluctuation between consecutive
  samples (`0` = independent, near `1` = slowly varying speed).
- `rng`: random number generator (e.g. `Xoshiro(1234)` for reproducibility).
- `includeend`: append the exact end point of the path as the last sample.

Returns a `Vector{SVector{3,Float64}}` starting at the path start.
"""
function sampleUAVPath(path::UAVPath, ds::Real;
                       jitter::Real=0.3,
                       speedcorrelation::Real=0.9,
                       rng=Random.default_rng(),
                       includeend::Bool=false)
    ds > 0 || throw(ArgumentError("ds must be positive"))
    0 <= speedcorrelation < 1 || throw(ArgumentError("speedcorrelation must be in [0, 1)"))
    jitter >= 0 || throw(ArgumentError("jitter must be non-negative"))

    total = length(path)
    samples = SVector{3,Float64}[]
    sizehint!(samples, ceil(Int, total / ds) + 2)

    # AR(1) relative speed deviation with stationary standard deviation `jitter`.
    ρ = Float64(speedcorrelation)
    innovationscale = jitter * sqrt(1 - ρ^2)
    deviation = jitter * randn(rng)
    minstep = 0.05 * ds  # never stop or move backwards

    x = 0.0
    push!(samples, SVector{3,Float64}(position(path, x)))

    while true
        deviation = ρ * deviation + innovationscale * randn(rng)
        x += max(ds * (1 + deviation), minstep)
        x > total && break
        push!(samples, SVector{3,Float64}(position(path, x)))
    end

    includeend && push!(samples, SVector{3,Float64}(position(path, total)))

    return samples
end
