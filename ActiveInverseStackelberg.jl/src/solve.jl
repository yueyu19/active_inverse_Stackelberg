function solve(;
    dyn::Dynamics,
    cost::Cost,
    p::Parameters
)
    Pl, El, Fl, Pf, Ef, Ff, Lambda, Lambdainv, L = dynamic_programming(dyn, cost, p)
    
end