Base.@kwdef struct Parameters
    d::Int          # number of subsystems in leader's system
    dt::Real        # discretization step size
    maxccpiter::Int # max # of rand seed in CCP
    maxrad::Int     # max box radius for reference state
    ccp_maxiter::Int    # max iteration of CCP
    ccp_eps::Real   # stopping criterion for CCP
end

"""Creates the parameters for the double-integrator experiment"""
di_params() = Parameters(;
    d = 3,
    dt = 0.2,
    maxccpiter = 10,
    maxrad = 5,
    ccp_maxiter = 10,
    ccp_eps = 1e-4
)