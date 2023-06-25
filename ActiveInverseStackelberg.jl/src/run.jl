function run_di()
    p = di_params()
    dyn = di_dynamics(p)
    cost = di_cost(p, dyn)
    problem = ActiveInverseStackelbergProblem(;
        parameters = p, 
        dynamics = dyn,
        cost = cost
    )
    xl_opt, xi_opt = solve(problem)
    plot_trajectories(problem, xl_opt, xi_opt)
end