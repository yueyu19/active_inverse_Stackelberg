struct Cost
    Ql::Matrix{<:Real}  # leader cost matrices
    Rl::Matrix{<:Real}
    Qf::Matrix{<:Real}  # follower cost matrices
    Rf::Matrix{<:Real}
end

function di_costs(d, ml, mf, nf)
    Ql = kron(diagm(ones(d)), diagm([1, 1, 0, 0]))
    rl = 1e-1*I
    
    Qf = zeros(nf, nf, d)
    Rf = zeros(mf, mf, d)
    for k = 1:d
        Qf[:, :, k] =  diagm([1; 1; 0; 0])
        Rf[:, :, k] = 1e-2*I
    end
    return Cost(Ql, rl, Qf, Rf)
end