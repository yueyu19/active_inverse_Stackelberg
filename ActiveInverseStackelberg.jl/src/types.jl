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

struct InitialConditions
    x0::Vector{<:Real}    # initial of leader state
    xi0::Vector{<:Real}   # initial follower state mean
end

Base.@kwdef struct ActiveInverseStackelbergProblem
    parameters::Parameters
    dynamics::Dynamics
    cost::Cost
    initial_conditions::InitialConditions
end

Base.@kwdef struct ActiveInverseStackelbergSolution
    ql_opt::Array{<:Real, 2}
    xl_opt::Array{<:Real, 2}
    qf_opt::Array{<:Real, 3}
    xi_opt::Array{<:Real, 3}
    Lambda::Array{<:Real, 4}
    Pf::Array{<:Real, 4}
end

struct TurtlebotConnection
    feedback::Connection
    rollout::Connection
    ts::Connection
    coeffs_x::Connection
    coeffs_y::Connection
end

struct Connections
    leader_tbs::Vector{TurtlebotConnection}
    follower_tb::TurtlebotConnection
    timing::Connection
end

struct SplineSegment
    coeffs_x::Vector{<:Real}
    coeffs_y::Vector{<:Real}
    t0::Real
    tf::Real
end

struct Spline
    ts::Vector{<:Real}
    all_x_coeffs::Matrix{<:Real}
    all_y_coeffs::Matrix{<:Real}
end

struct RolloutData
    ts::Vector{<:Real}
    xs::Matrix{<:Real}
    xds::Matrix{<:Real}
    us::Matrix{<:Real}
end