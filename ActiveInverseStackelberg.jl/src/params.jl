"""Creates the parameters for the double-integrator experiment"""
di_params() = Parameters(;
    d = 3,
    dt = 0.2,
    T = 2,
    maxccpiter = 10,
    maxrad = 10,
    ccp_maxiter = 10,
    ccp_eps = 1e-4
)