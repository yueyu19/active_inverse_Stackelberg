struct Cost
    Ql::Array{<:Real, 2}    # leader cost matrices
    Rl::Array{<:Real, 2}
    Qf::Array{<:Real, 3}    # follower cost matrices
    Rf::Array{<:Real, 3}
end

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