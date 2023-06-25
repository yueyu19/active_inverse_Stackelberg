Base.@kwdef struct Parameters
    d::Int          # number of subsystems in leader's system
    dt::Real        # discretization step size
    T::Real         # time horizon (total duration of the game)
    maxccpiter::Int # max # of rand seed in CCP
    maxrad::Int     # max box radius for reference state
    ccp_maxiter::Int    # max iteration of CCP
    ccp_eps::Real   # stopping criterion for CCP
end

struct Dynamics
    Ac0::Matrix{<:Real}
    Bc0::Matrix{<:Real}
    Al::Matrix{<:Real}
    Bl::Matrix{<:Real}
    Af::Matrix{<:Real}
    Bf::Matrix{<:Real}
end

struct Cost
    Ql::Array{<:Real, 2}    # leader cost matrices
    Rl::Array{<:Real, 2}
    Qf::Array{<:Real, 3}    # follower cost matrices
    Rf::Array{<:Real, 3}
end

Base.@kwdef struct ActiveInverseStackelbergProblem
    parameters::Parameters
    dynamics::Dynamics
    cost::Cost
end