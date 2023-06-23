struct Dynamics
    Ac0::Matrix{<:Real}
    Bc0::Matrix{<:Real}
    Al::Matrix{<:Real}
    Bl::Matrix{<:Real}
    Af::Matrix{<:Real}
    Bf::Matrix{<:Real}
end

"""Creates the dynamics for p.d leader subsystems and 1 follower, all with
identical LTI dynamics specified by Ac0 and Bc0."""
function Dynamics(Ac0, Bc0, p::Parameters)
    # FOH discretization (LTI systems only)
    A0 = exp(p.dt*Ac0)
    integral, _ = quadgk(t -> exp(t*Ac0), 0, p.dt)
    B0 = integral*Bc0
    
    Al = kron(diagm(ones(p.d)), A0)
    Bl = kron(diagm(ones(p.d)), B0)

    # we are assuming the follower has the same dynamics as the leader
    Af = A0
    Bf = B0

    return Dynamics(Ac0, Bc0, Al, Bl, Af, Bf)
end

"""Creates the dynamics the double-integrator experiment"""
function di_dynamics(p::Parameters)
    # double-integrator agent dynamics
    Ac0 = [
        0   0   1   0
        0   0   0   1
        0   0   0   0
        0   0   0   0
    ]
    Bc0 = [
        0   0
        0   0
        1   0
        0   1
    ]
    return Dynamics(Ac0, Bc0, p)
end