module ActiveInverseStackelberg

using LinearAlgebra: I, kron, diagm, pinv, norm
using QuadGK: quadgk
using Combinatorics: combinations
using JuMP
using MosekTools

include("params.jl")
include("dynamics.jl")
include("costs.jl")
include("dynamic_programming.jl")
include("solve.jl")
include("run.jl")

end # module ActiveInverseStackelberg
