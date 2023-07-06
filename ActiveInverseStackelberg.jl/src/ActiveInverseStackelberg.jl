module ActiveInverseStackelberg

using LinearAlgebra: I, kron, diagm, pinv, norm
using QuadGK: quadgk
using Combinatorics: combinations
using JuMP
using MosekTools
using Ipopt
using CairoMakie: Figure, Axis, lines!, scatter!, DataAspect, Colorbar
using RosSockets
import JSON
using Rotations: QuatRotation

include("types.jl")
include("params.jl")
include("dynamics.jl")
include("costs.jl")
include("dynamic_programming.jl")
include("solve.jl")
include("run.jl")
include("plot_utils.jl")

include("communication.jl")
include("spline.jl")

end # module ActiveInverseStackelberg
