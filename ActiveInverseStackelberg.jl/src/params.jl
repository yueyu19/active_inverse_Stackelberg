Base.@kwdef struct Parameters
    d::Int      # number of subsystems in leader's system
    dt::Real    # discretization step size
end

"""Creates the parameters for the double-integrator experiment"""
di_params() = Parameters(;
    d = 3,
    dt = 0.2
)