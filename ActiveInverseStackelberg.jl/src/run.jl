function run_di()
    p = di_params()
    dyn = di_dynamics(p)
    cost = di_cost(p, dyn)

    x0 = zeros(p.d*size(dyn.Ac0,1))     # initial leader states  
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

function run_di_tb()
    p = di_tb_params()
    dyn = di_tb_dynamics(p)
    cost = di_tb_cost(p, dyn)

    connections = open_tb_connections()

    x0 = Vector{Real}()     # initial leader states
    # small initial velocity for each leader so that they initially "point" the
    # same way as the actual robot
    v = 0.01
    for i in 1:p.d
        s = state(connections.tbs[i])
        x = s[1]
        y = s[2]
        θ = s[4]
        xdot = v*cos(θ)
        ydot = v*sin(θ)
        append!(x0, [x, y, xdot, ydot])
    end
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

    splines = make_splines(inv_stackelberg_problem, xl_opt)
    plot_splines(splines)

    close_tb_connections(connections)
end