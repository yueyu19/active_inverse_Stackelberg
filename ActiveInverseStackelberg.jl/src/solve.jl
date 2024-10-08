function solve(problem::ActiveInverseStackelbergProblem)
    dyn = problem.dynamics
    cost = problem.cost
    p = problem.parameters
    
    Pl, El, Fl, Pf, Ef, Ff, Lambda, Lambdainv, L = dynamic_programming(dyn, cost, p)

    n0 = size(dyn.Ac0,1)
    nl, ml = size(dyn.Bl)
    nf, mf = size(dyn.Bf)
    tau = Integer(round(p.T/p.dt))            # length of trajectory
    setD = collect(combinations(1:p.d, 2))  # set of hypo pairs
    Dn = size(setD, 1)
    d = p.d

    x0 = problem.initial_conditions.x0[:,:] # initilization of leader's state
    xi0 = problem.initial_conditions.xi0    # initialization of the follower's state mean
    u_opt = zeros(ml, tau)      # initialize optimal inputs for leader
    objval_opt = Inf            # initialize optimal value in CCP
    objval = Inf                
    w_opt = Inf

    # random initialization of CCP (convex-concave procedure)
    for ccp_rand = 1:p.maxccpiter 
        
        w = x0 + p.maxrad*(2*rand(nl, 1) .- 1)   # random reference point for leader
        
        # propagate the leader & follower's states & co-states
        ql_hat, xl_hat, qf_hat, xi_hat = dynprop(
            El, Fl, Ef, Ff, cost.Ql, cost.Qf, L, x0, xi0, w
        )

        objval_hat = Inf # initialize optimal value for CCP

        # CCP iteration
        for ccp_iter = 1:p.ccp_maxiter 
            model = Model(MosekTools.Optimizer)
            set_silent(model)

            @variables(model, begin
                ql[1:nl, 1:tau+1]       # leader's costate
                xl[1:nl, 1:tau+1]       # leader's state
                w[1:nl]                 # leader's reference state
                qf[1:nf, 1:tau+1, 1:d]  # hypothesis agent co-state
                xi[1:nf, 1:tau+1, 1:d]  # hypothesis agent state
                eta[1:nf, 1:tau+1, 1:Dn]    # auxiliary variable for quadratic constraints
                S[1:tau+1, 1:Dn]        # slack variable
                ups                     # upper bound for sum of quadratics
            end)

            @constraints(model, begin
                xl[:,1] .== x0              # initial condition for leader's state
                ql[:,end] .== -cost.Ql*w    # final condition for leader's co-state
            end)

            for t = 1:tau
                @constraints(model, begin
                    # leader state dynamics
                    xl[:,t+1] == El[:,:,t]*xl[:,t] - Fl[:,:,t]*ql[:,t+1]

                    # leader co-state dynamics
                    ql[:,t] == El[:,:,t]'*ql[:,t+1] - cost.Ql*w
                end)
            end

            # reference state constraints
            for k = 1:d
                num = (k-1)*n0
                @constraints(model, begin
                    (w[num+1] - x0[num+1])^2 <= p.maxrad^2
                    (w[num+2] - x0[num+2])^2 <= p.maxrad^2
                end)
                @constraint(model, w[num+3:num+4] == [0, 0])
            end

            for k = 1:d
                @constraints(model, begin
                    # final condition for k-th follower co-state
                    qf[:,tau+1,k] == -cost.Qf[:,:,k]*L[:,:,k]*xl[:,tau+1]

                    # initial condition for k-th follower state
                    xi[:,1,k] == xi0; 
                end)

                for t = 1:tau
                    @constraints(model, begin
                        # k-th follower's co-state dynamics
                        qf[:,t,k] == Ef[:,:,t,k]'*qf[:,t+1,k] - cost.Qf[:,:,k]*L[:,:,k]*xl[:,t]
                        
                        # k-th follower's state dynamics
                        xi[:,t+1,k] == Ef[:,:,t,k]*xi[:,t,k] - Ff[:,:,t,k]*qf[:,t+1,k]
                    end)
                end
            end

            for ind = 1:Dn
                for t = 1:tau+1
                    k1 = setD[ind][1]
                    k2 = setD[ind][2]
                    # upper bound quadratics
                    @constraint(model, 
                        eta[:,t,ind] == (sqrt(Lambdainv[:,:,t,k1] + Lambdainv[:,:,t,k2])
                                        *(xi[:,t,k1] - xi[:,t,k2])))
                    @constraint(model, eta[:,t,ind]'*eta[:,t,ind] <= S[t,ind])
                end
                @constraint(model, sum(sum(S)) - sum(S[:, ind]) <= ups)
            end

            objf1 = 0;
            for t = 1:tau+1
                for k1 = 1:d-1
                    for k2 = k1+1:d
                        objf1 += ((xi_hat[:,t,k1] - xi_hat[:,t,k2])'
                            *(Lambdainv[:,:,t,k1]+Lambdainv[:,:,t,k2])*(xi[:,t,k1] - xi[:,t,k2]))
                    end
                end
            end

            @objective(model, Min, 
                ups - objf1
            )

            optimize!(model)

            objval1 = vol(value.(xi), Lambdainv)
            objval = max(value.(ups)) - objval1

            println("Sample $(ccp_rand), CCP iter $(ccp_iter), Value $(-objval)")

            if objval_hat - objval < p.ccp_eps
                break
            else
                objval_hat = objval
            end
        end
        if objval < objval_opt
            w_opt = value.(w)
            objval_opt = objval
        end
    end
    
    ql_opt, xl_opt, qf_opt, xi_opt = dynprop(El, Fl, Ef, Ff, cost.Ql, cost.Qf, L, x0, xi0, w_opt)
    
    return ActiveInverseStackelbergSolution(;
        ql_opt = ql_opt,
        xl_opt = xl_opt,
        qf_opt = qf_opt,
        xi_opt = xi_opt,
        Lambda = Lambda,
        Pf = Pf
    )
end

function vol(xi, W)
    (_, tau1, N) = size(xi)

    f = 0
    for t = 1:tau1
        for k1 = 1:N
            for k2 = k1+1:N
                f += (xi[:,t,k1] - xi[:,t,k2])'*(W[:,:,t,k1] + W[:,:,t,k2])*(xi[:,t,k1] - xi[:,t,k2])
            end
        end
    end
    return f
end

function follower_trajectory(
    prob::ActiveInverseStackelbergProblem,
    sol::ActiveInverseStackelbergSolution
    ; i::Integer = 1 # used to select which leader is followed
)
    xi_opt = sol.xi_opt
    qf_opt = sol.qf_opt
    Lambda = sol.Lambda
    Af = prob.dynamics.Af
    Bf = prob.dynamics.Bf
    Pf = sol.Pf
    Rf = prob.cost.Rf
    
    nf = size(prob.dynamics.Bf,1)
    tau = size(sol.xi_opt, 2)
    
    xf = zeros(nf, tau)
    xf[:,1] = rand(MvNormal(xi_opt[:,1,i], Lambda[:,:,1,i]))
    for t = 1:tau-1
        Sigma = pinv(Rf[:,:,i] + Bf'*Pf[:,:,t+1,i]*Bf)
        mu = -Sigma*Bf'*(Pf[:,:,t+1,i]*Af*xf[:,t] + qf_opt[:,t+1,i])
        xf[:,t+1] = Af*xf[:,t] + Bf*rand(MvNormal(mu, Sigma))
    end
    return xf
end