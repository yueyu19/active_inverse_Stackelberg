function dynamic_programming(
        dyn::Dynamics,
        cost::Cost,
        p::Parameters
    )
    # extract dimensions
    nl, ml = size(dyn.Bl)
    nf, mf = size(dyn.Bf)

    tau = Integer(round(p.T/p.dt))     # length of trajectory

    # memory dyn.Allocation for leader matrices
    Pl = zeros(nl, nl, tau+1)
    El = zeros(nl, nl, tau)
    Fl = zeros(nl, nl, tau)

    # initidyn.Alization for leader's closed-loop matrices
    Pl[:,:,end] = cost.Ql

    # Dynamic programming for leader's closed-loop matrices
    for t = tau:-1:1
        Fl[:,:,t] = dyn.Bl*pinv(cost.Rl+dyn.Bl'*Pl[:,:,t+1]*dyn.Bl)*dyn.Bl'
        El[:,:,t] = dyn.Al - Fl[:,:,t]*Pl[:,:,t+1]*dyn.Al
        Pl[:,:,t] = cost.Ql + dyn.Al'*Pl[:,:,t+1]*El[:,:,t]
    end

    # memory allocation for follower matrices
    Pf = zeros(nf, nf, tau+1, p.d)
    Ef = zeros(nf, nf, tau, p.d)
    Ff = zeros(nf, nf, tau, p.d)
    Lambda = zeros(nf, nf, tau+1, p.d)
    Lambdainv = zeros(nf, nf, tau+1, p.d)

    # initialization for follower's closed-loop matrices
    for k = 1:p.d
        Pf[:,:,end,k] = cost.Qf[:,:,k]
        Lambda[:,:,1,k] = 1e-2*diagm(ones(nf)) # follower's initial state variance
        Lambdainv[:,:,1,k] = pinv(Lambda[:,:,1,k])
    end

    # Dynamic programming for follower's closed-loop matrices
    for t = tau:-1:1
        for k = 1:p.d
            Ff[:,:,t,k] = (dyn.Bf*pinv(cost.Rf[:,:,k]
                            + dyn.Bf'*Pf[:,:,t+1,k]*dyn.Bf)*dyn.Bf')
            Ef[:,:,t,k] = dyn.Af - Ff[:,:,t,k]*Pf[:,:,t+1,k]*dyn.Af
            Pf[:,:,t,k] = cost.Qf[:,:,k] + dyn.Af'*Pf[:,:,t+1,k]*Ef[:,:,t,k]
        end
    end

    for t = 1:tau
        for k = 1:p.d
            # propogate the follower's state var under hypo k
            Lambda[:,:,t+1,k] = Ef[:,:,t,k]*Lambda[:,:,t,k]*Ef[:,:,t,k]' + Ff[:,:,t,k]
            Lambdainvtemp = pinv(Lambda[:,:,t+1,k])
            Lambdainv[:,:,t+1,k] = 0.5*(Lambdainvtemp + Lambdainvtemp')
        end
    end

    # output matrix that maps leader's state to follower's reference state, one
    # for each hypothesis
    L = zeros(nf, nl, p.d)

    for k = 1:p.d
        L[:, (k-1)*nf+1:k*nf, k] = diagm(ones(nf))
    end

    return Pl, El, Fl, Pf, Ef, Ff, Lambda, Lambdainv, L
end