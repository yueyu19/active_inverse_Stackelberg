"""Cubic spline segment between `s0` and `sf` at the times `t0` and `tf`"""
function SplineSegment(t0::Real, tf::Real, s0::Vector{<:Real}, sf::Vector{<:Real})
    x0 = s0[1]
    xf = sf[1]
    xdot_0 = s0[3]
    xdot_f = sf[3]

    y0 = s0[2]
    yf = sf[2]
    ydot_0 = s0[4]
    ydot_f = sf[4]

    A = [
            t0^3    t0^2    t0  1
            tf^3    tf^2    tf  1
            3*t0^2  2*t0    1   0
            3*tf^2  2*tf    1   0
        ]

    coeffs_x = A \ [x0, xf, xdot_0, xdot_f]
    coeffs_y = A \ [y0, yf, ydot_0, ydot_f]

    return SplineSegment(
        coeffs_x,
        coeffs_y,
        t0,
        tf
    )
end

"""For each leader, constructs splines between states in `xl`"""
function make_splines(
    prob::ActiveInverseStackelbergProblem,
    sol::ActiveInverseStackelbergSolution
)
    xl = sol.xl_opt
    dt = prob.parameters.dt
    tau = size(xl, 2) # number of time steps
    
    splines = Vector{Spline}(undef, prob.parameters.d)
    for j in 1:prob.parameters.d
        ts = zeros(tau)
        all_x_coeffs = zeros(tau, 4)
        all_y_coeffs = zeros(tau, 4)
        t = 0.0
        for i in 1:tau-1
            num = (j-1)*4
            s = xl[num+1:num+4, i]
            s_next = xl[num+1:num+4, i+1]
            ss = SplineSegment(t, t+dt, s, s_next)
            all_x_coeffs[i,:] = ss.coeffs_x
            all_y_coeffs[i,:] = ss.coeffs_y
            ts[i+1] = ss.tf
            t += dt
        end
        splines[j] = Spline(ts, all_x_coeffs, all_y_coeffs)
    end
    return splines
end

"""Evaluate the spline at time t and return [x, y, xdot, ydot]"""
function evaluate(spl::Spline, t)
    i = searchsortedlast(spl.ts, t)
    if i == 0
        i = 1
    end
    coeffs_x = spl.all_x_coeffs[i,:]
    coeffs_y = spl.all_y_coeffs[i,:]
    return [
        coeffs_x[1]*t^3 + coeffs_x[2]*t^2 + coeffs_x[3]*t + coeffs_x[4],
        coeffs_y[1]*t^3 + coeffs_y[2]*t^2 + coeffs_y[3]*t + coeffs_y[4],
        3*coeffs_x[1]*t^2 + 2*coeffs_x[2]*t + coeffs_x[3],
        3*coeffs_y[1]*t^2 + 2*coeffs_y[2]*t + coeffs_y[3]
    ]
end