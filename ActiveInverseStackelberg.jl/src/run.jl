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
    sol = solve(inv_stackelberg_problem)
    xf = follower_trajectory(inv_stackelberg_problem, sol, i=1)
    plot_trajectories(inv_stackelberg_problem, sol, xf=xf)
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
        s = state(connections.leader_tbs[i])
        x = s[1]
        y = s[2]
        θ = s[4]
        xdot = v*cos(θ)
        ydot = v*sin(θ)
        append!(x0, [x, y, xdot, ydot])
    end
    
    # initial follower state mean
    xi0 = zeros(size(dyn.Bf,1))         
    s_follower = state(connections.follower_tb)
    xi0[1] = s_follower[1]
    xi0[2] = s_follower[2]
    xi0[3] = v*cos(s_follower[4])
    xi0[4] = v*sin(s_follower[4])
    
    initial_conditions = InitialConditions(x0, xi0)

    inv_stackelberg_problem = ActiveInverseStackelbergProblem(;
        parameters = p, 
        dynamics = dyn,
        cost = cost,
        initial_conditions = initial_conditions
    )
    sol = solve(inv_stackelberg_problem)
    xf = follower_trajectory(inv_stackelberg_problem, sol, i=1)
    plot_trajectories(inv_stackelberg_problem, sol, xf=xf)

    leader_splines = make_splines(inv_stackelberg_problem, sol)
    follower_spline = make_spline(inv_stackelberg_problem, xf)
    fig, ax = plot_splines(leader_splines, follower_spline)

    send_leader_splines(connections, leader_splines)
    send_follower_spline(connections, follower_spline)
    sleep(0.5)
    start_robots(connections)
    t = 0.0
    while t < p.T
        sleep(0.5)
        t = time_elapsed(connections)
        @info "Running experiemnt. Time elapsed: $t"
    end
    stop_robots(connections)
    sleep(1.0)
    leader_rs, follwer_r = all_rollout_data(connections)
    close_tb_connections(connections)

    plot_rollouts(fig, ax, leader_rs, follwer_r)
end