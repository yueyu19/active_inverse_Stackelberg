module ActiveInverseStackelberg

using LinearAlgebra: I, kron, diagm, pinv
using QuadGK: quadgk

include("params.jl")
include("dynamics.jl")
include("costs.jl")
include("dynamic_programming.jl")
include("solve.jl")
include("run.jl")

end # module ActiveInverseStackelberg
