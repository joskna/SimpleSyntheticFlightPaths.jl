using Test
using Random
using StaticArrays
using SimpleSyntheticFlightPaths

@testset "SimpleSyntheticFlightPaths" begin
    path = UAVplaneflight(0.0, 100.0, 0.0, 50.0, 10.0, 30.0, 10.0,
        [2.0, 1.0, 0.5], [1.0, 0.5]; usedeterministicseed=true)

    @test length(path) > 0
    @test Base.length(path.segments) == 5      # 3 lines + 2 bridges

    @testset "position" begin
        @test position(path, 0.0) ≈ SVector(0.0, 0.0, 10.0)
        @test_throws ErrorException position(path, length(path) + 1)
    end

    @testset "sampling" begin
        pts = sampleUAVPath(path, 1.0; rng=Xoshiro(42))
        @test pts isa Vector{SVector{3,Float64}}
        @test pts[1] ≈ position(path, 0.0)

        # no jitter -> uniform spacing in arc length
        uniform = sampleUAVPath(path, 1.0; jitter=0.0)
        @test Base.length(uniform) == floor(Int, length(path)) + 1

        # reproducible with the same RNG seed
        @test sampleUAVPath(path, 1.0; rng=Xoshiro(1)) == sampleUAVPath(path, 1.0; rng=Xoshiro(1))

        @test sampleUAVPath(path, 1.0; includeend=true)[end] ≈ position(path, length(path))
        @test_throws ArgumentError sampleUAVPath(path, -1.0)
    end
end
