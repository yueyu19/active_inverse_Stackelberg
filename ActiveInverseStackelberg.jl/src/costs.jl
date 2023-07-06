"""Creates the cost matrices the double-integrator experiment"""
function di_cost(p::Parameters, dyn::Dynamics)
    d = p.d
    ml = size(dyn.Bl,2)
    nf, mf = size(dyn.Bf)
    
    Ql = kron(diagm(ones(d)), diagm([1, 1, 0, 0]))
    Rl = 1e-1*diagm(ones(ml))
    
    Qf = zeros(nf, nf, d)
    Rf = zeros(mf, mf, d)
    for k = 1:d
        Qf[:, :, k] =  diagm([1; 1; 0; 0])
        Rf[:, :, k] = 1e-2*diagm(ones(mf))
    end
    return Cost(Ql, Rl, Qf, Rf)
end

"""Creates the cost matrices the double-integrator turtlebot experiment"""
function di_tb_cost(p::Parameters, dyn::Dynamics)
    d = p.d
    ml = size(dyn.Bl,2)
    nf, mf = size(dyn.Bf)
    
    Ql = kron(diagm(ones(d)), diagm([1, 1, 0, 0]))
    Rl = 1e-1*diagm(ones(ml))
    
    Qf = zeros(nf, nf, d)
    Rf = zeros(mf, mf, d)
    for k = 1:d
        Qf[:, :, k] =  diagm([1; 1; 0; 0])
        Rf[:, :, k] = 1e-2*diagm(ones(mf))
    end
    return Cost(Ql, Rl, Qf, Rf)
end