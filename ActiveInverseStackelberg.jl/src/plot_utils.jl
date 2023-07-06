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

function plot_splines(
    splines::Vector{Spline}
)
    
    fig = Figure(resolution=(1150, 850))
    ax = Axis(fig[1, 1], aspect=DataAspect(), title ="Trajectories", 
                xlabel="x (m)", ylabel="y (m)", 
                xticks=-3:0.5:3, yticks=-3:0.5:3)
    colormaps = [:Blues_9, :Greens_9, :Reds_9, :Purples_9, :Oranges_9, :Greys_9]

    for (j, spl) in enumerate(splines)
        t = range(0, stop=spl.ts[end], length=3000)
        T = length(t)-1
        xs = zeros(T)
        ys = zeros(T)
        xdots = zeros(T)
        ydots = zeros(T)
        vs = zeros(T)
        for i in 1:T
            xs[i], ys[i], xdots[i], ydots[i] = evaluate(spl, t[i])
            vs[i] = norm([xdots[i], ydots[i]])
        end
        
        s = scatter!(ax, xs, ys, color=vs, markersize=6,
                        colormap=colormaps[j], colorrange = (0.0, 0.30))
        Colorbar(fig[1, 1+j], s, label="Planned Velocity (m/s), Leader $(j)")
        println("Leader $(j) planned max velocity: $(maximum(vs)) m/s")
    end
    display(fig)
    return fig, ax
end

function plot_rollouts(fig, ax, rs::Vector{RolloutData})
    for r in rs
        scatter!(ax, r.xs[1,:], r.xs[2,:], color=:black, markersize=3)
    end
    display(fig)
end