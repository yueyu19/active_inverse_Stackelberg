function plot_trajectories(prob::ActiveInverseStackelbergProblem, xl_opt, xi_opt)
    n0 = size(prob.dynamics.Ac0,1)
    
    fig = Figure(resolution = (800, 800))
    ax = Axis(fig[1,1], xlabel = "x", ylabel = "y")
    for k = 1:prob.parameters.d
        num = (k-1)*n0
        lines!(ax, xl_opt[num+1,:], xl_opt[num+2,:], label = "Leader $k")
    end
    display(fig)
end