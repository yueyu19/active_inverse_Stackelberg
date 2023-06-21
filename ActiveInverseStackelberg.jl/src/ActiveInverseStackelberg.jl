module ActiveInverseStackelberg

using LinearAlgebra: I, kron, diagm
using QuadGK: quadgk

include("params.jl")
include("dynamics.jl")
include("costs.jl")
include("solve.jl")
include("run.jl")

end # module ActiveInverseStackelberg
