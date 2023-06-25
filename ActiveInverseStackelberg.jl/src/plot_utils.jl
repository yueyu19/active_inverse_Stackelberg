function plot_trajectories(prob::ActiveInverseStackelbergProblem, xl_opt, xi_opt)
    n0 = size(prob.dynamics.Ac0,1)
    
    fig = Figure(resolution = (800, 800))
    ax = Axis(fig[1,1], title="Position1", xlabel="x", ylabel="y")
    for k = 1:prob.parameters.d
        num = (k-1)*n0
        lines!(ax, xl_opt[num+1,:], xl_opt[num+2,:], label = "Leader $k")
        scatter!(ax, xl_opt[num+1,:], xl_opt[num+2,:], label = "Leader $k")
    end
    display(fig)

    fig2 = Figure(resolution = (800, 800))
    ax2 = Axis(fig2[1,1], title="Position2", xlabel="x", ylabel="y")
    for k = 1:prob.parameters.d
        lines!(ax2, xi_opt[1,:,k], xi_opt[2,:,k])
        scatter!(ax2, xi_opt[1,end,k], xi_opt[2,end,k])
        scatter!(ax2, xi_opt[1,1,k], xi_opt[2,1,k])
    end
    display(fig2)
end