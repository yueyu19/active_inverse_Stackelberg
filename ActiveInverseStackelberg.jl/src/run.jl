function run_di()
    p = di_params()
    dyn = di_dynamics(p)
    cost = di_cost(p, dyn)

    x0 = zeros(p.d*size(dyn.Ac0,1))     # initial of leader state  
    xi0 = zeros(size(dyn.Bf,1))         # initial follower state mean
    initial_conditions = InitialConditions(x0, xi0)

    inv_stackelberg_problem = ActiveInverseStackelbergProblem(;
        parameters = p, 
        dynamics = dyn,
        cost = cost,
        initial_conditions = initial_conditions
    )
    xl_opt, xi_opt = solve(inv_stackelberg_problem)
    plot_trajectories(inv_stackelberg_problem, xl_opt, xi_opt)
end