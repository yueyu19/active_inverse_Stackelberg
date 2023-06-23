function solve(;
    dyn::Dynamics,
    cost::Cost,
    p::Parameters
)
    Pl, El, Fl, Pf, Ef, Ff, Lambda, Lambdainv, L = dynamic_programming(dyn, cost, p)

    n0 = size(dyn.Ac0,1)
    nl, ml = size(dyn.Bl)
    nf, mf = size(dyn.Bf)
    tau = Integer(round(1/p.dt))            # length of trajectory
    setD = collect(combinations(1:p.d, 2))  # set of hypo pairs

    x0 = kron(ones(p.d, 1), zeros(n0))      # initilization of leader's state
    xi0 = zeros(nf)             # initialization of the follower's state mean

    u_opt = zeros(ml, tau)      # initialize optimal inputs for leader
    objval_opt = Inf            # initialize optimal value in CCP
end