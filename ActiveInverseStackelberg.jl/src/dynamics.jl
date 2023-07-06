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

"""Creates the dynamics the double-integrator turtlebot experiment"""
function di_tb_dynamics(p::Parameters)
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

function dynprop(El, Fl, Ef, Ff, Ql, Qf, L, x0, xi0, w)

    tau = size(El,3)
    n2, n1, d = size(L)
    
    xl = zeros(n1, tau+1);
    ql = zeros(n1, tau+1);
    qf = zeros(n2, tau+1, d);
    xi = zeros(n2, tau+1, d);
    
    xl[:,1] = x0 # initial condition for leader
    
    for k = 1:d
        # initial condition for follower under hypo k
        xi[:,1,k] = xi0
    end
    
    # final condition for predicted follower co-state
    ql[:,tau+1] = -Ql*w
    
    for t = tau:-1:1
        # propagate leader's co-state
        ql[:,t] = El[:,:,t]'*ql[:,t+1] - Ql*w
    end
    
    for t = 1:tau
        # propogate leader's state
        xl[:,t+1] = El[:,:,t]*xl[:,t] - Fl[:,:,t]*ql[:,t+1]
    end
    
    
    for k = 1:d
        # final condition for follower co-state under hypo k
        qf[:,tau+1,k] = -Qf[:,:,k]*L[:,:,k]*xl[:,tau+1]
    end
    
    for t = tau:-1:1
        for k = 1:d
            # propagate follower co-state under hypothesis k
            qf[:,t,k] = Ef[:,:,t,k]'*qf[:,t+1,k] - Qf[:,:,k]*L[:,:,k]*xl[:,t]
        end
    end
    
    for t = 1:tau
        for k = 1:d
            # propogate follower's state
            xi[:,t+1,k] = Ef[:,:,t,k]*xi[:,t,k] - Ff[:,:,t,k]*qf[:,t+1,k]
        end
    end
    
    return ql, xl, qf, xi
end