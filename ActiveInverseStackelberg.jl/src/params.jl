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

"""Creates the parameters for the double-integrator turtlebot experiment"""
di_tb_params() = Parameters(;
    d = 3,
    dt = 2.0,
    T = 30,
    maxccpiter = 10,
    maxrad = 10,
    ccp_maxiter = 10,
    ccp_eps = 1e-4
)