function run_di()
    p = di_params()
    dyn = di_dynamics(p)
    cost = di_cost(p, dyn)
    solve(dyn=dyn, cost=cost, p=p)
end