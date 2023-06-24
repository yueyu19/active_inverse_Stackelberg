clear;clc;close all

% double-integrator agent dynamics

Ac0 = [0,     0,      1,       0;
    0,     0,      0,       1;
    0,     0,      0,       0;
    0,     0,      0        0];
Bc0 = [0, 0;
    0, 0;
    1, 0;
    0, 1];

% continuous time dynamics d/dt(x) = Ac*x+Bc*u

% state: [position; velocity];
% control: acceleration;

[n0, m0] = size(Bc0); % extract dimension

dt = 0.2; % discretization step size

A0 = expm(dt*Ac0); % FOH discretization
B0 = integral(@(t) A0*t, 0, dt, 'ArrayValued', true)*Bc0;

tau = round(2/dt); % length of trajectory

% leader dynamics
d = 6; % number of subsystems in leader's system

Al = kron(eye(d), A0); % the joint dynamics of d double-integrator agents
Bl = kron(eye(d), B0);
[nl, ml] = size(Bl); % leader's dimension

% follower dynamics

Af = A0; % one double-integrator
Bf = B0;
[nf, mf] = size(Bf); % follower's dimension

% leader cost matrices
Ql = kron(eye(d), diag([1; 1; 0; 0]));
Rl = 1e-1*eye(ml, ml);

% follower cost matrices
Qf = zeros(nf, nf, d);
Rf = zeros(mf, mf, d);

for k = 1:d
    Qf(:, :, k) = diag([1; 1; 0; 0]); % follower's cost matrices under hypothesis n
    Rf(:, :, k) = 1e-2*eye(mf, mf);
end

% memory allocation for leader matrices
Pl = zeros(nl, nl, tau+1);
El = zeros(nl, nl, tau);
Fl = zeros(nl, nl, tau);

% initialization for leader's closed-loop matrices
Pl(:, :, end) = Ql;

% Dynamic programming for leader's closed-loop matrices
for t = tau:-1:1
    Fl(:, :, t) = Bl*pinv(Rl+Bl'*Pl(:, :, t+1)*Bl)*Bl';
    El(:, :, t) = Al - Fl(:, :, t)*Pl(:, :, t+1)*Al;
    Pl(:, :, t) = Ql + Al'*Pl(:, :, t+1)*El(:, :, t);
end

% memory allocation for follower matrices
Pf = zeros(nf, nf, tau+1, d);
Ef = zeros(nf, nf, tau, d);
Ff = zeros(nf, nf, tau, d);
Lambda = zeros(nf, nf, tau, d);
Lambdainv = zeros(nf, nf, tau+1, d);


% initialization for follower's closed-loop matrices
for k = 1:d
    Pf(:, :, end, k) = Qf(:, :, k);
    Lambda(:, :, 1, k) = 1e-2*eye(nf); % follower's initial state variance
    Lambdainv(:, :, 1, k) = pinv(Lambda(:, :, 1, k));
end

% Dynamic programming for follower's closed-loop matrices
for t = tau:-1:1
    for k = 1:d
        Ff(:, :, t, k) = Bf*pinv(Rf(:, :, k)+Bf'*Pf(:, :, t+1, k)*Bf)*Bf';
        Ef(:, :, t, k) = Af - Ff(:, :, t, k)*Pf(:, :, t+1, k)*Af;
        Pf(:, :, t, k) = Qf(:, :, k) + Af'*Pf(:, :, t+1, k)*Ef(:, :, t, k);
    end
end

for t = 1:tau
    for k = 1:d
        Lambda(:, :, t+1, k) = Ef(:, :, t, k)*Lambda(:, :, t, k)*Ef(:, :, t, k)' + Ff(:, :, t, k); % propogate the follower's state var under hypo k
        Lambdainv(:, :, t+1, k) = pinv(Lambda(:, :, t+1, k));
    end
end


L = zeros(nf, nl, d); % output matrix that maps leader's state to follower's reference state, one for each hypothesis

for k = 1:d
    L(:, (k-1)*nf+1:k*nf, k) = eye(nf); 
end

%%
% system parameters

maxccpiter = 10; % max # of rand seed in CCP
maxrad = 10; % max box radius for reference state

setD = nchoosek(1:d, 2); % set of hypo pairs


x0 = kron(ones(d, 1), [0; 0; 0; 0]); % initilization of leader's state
xi0 = [0; 0; 0; 0]; % initialization of the follower's state mean

u_opt = zeros(ml, tau); % initialize optimal inputs for leader
objval_opt = Inf; % initialize optimal value in CCP


for ccp_rand = 1:maxccpiter % random initialization of CCP (convex-concave procedure)

    w = x0 + maxrad*(2*rand(nl, 1)-1); % random reference point for leader

    [ql_hat, xl_hat, qf_hat, xi_hat] = dynprop(El, Fl, Ef, Ff, Ql, Qf, L, x0, xi0, w); % propagate the leader & follower's states & co-states

    objval_hat = Inf; % initialize optimal value for CCP

    ccp_maxiter = 10; % max iteration of CCP
    ccp_eps = 1e-4; % stopping criterion for CCP
    Dn = size(setD, 1); % size 

    for ccp_iter = 1:ccp_maxiter % CCP iteration

        yalmip('clear')

        ql = sdpvar(nl, tau+1, 'full'); % leader's costate
        xl = sdpvar(nl, tau+1, 'full'); % leader's state
        w = sdpvar(nl, 1, 'full'); % leader's reference state
        qf = sdpvar(nf, tau+1, d,'full'); % hypothesis agent co-state
        xi = sdpvar(nf, tau+1, d, 'full'); % hypothesis agent state
        S = sdpvar(tau+1, Dn, 'full'); % slack variable
        ups = sdpvar(1, 1, 'full'); % upper bound for sum of quadratics

        constr = [xl(:, 1) == x0, ...    % initial condition for leader's state
            ql(:, tau+1) == -Ql*w]; % final condition for leader's co-state

        for t = 1:tau
            constr = [constr, xl(:, t+1) == El(:, :, t)*xl(:, t) - Fl(:, :, t)*ql(:, t+1)]; % leader state dynamics
            constr = [constr, ql(:, t) == El(:, :, t)'*ql(:, t+1) - Ql*w]; % leader co-state dynamics
        end

        for k = 1:d
            num = (k-1)*n0;
            constr = [constr, norm(w(num+1:num+2) - x0(num+1:num+2), Inf) <= maxrad, w(num+3:num+4) == [0; 0]]; % reference state constraints
        end

        for k = 1:d
            constr = [constr, qf(:, tau+1, k) == -Qf(:, :, k)*L(:, :, k)*xl(:, tau+1), ... % final condition for k-th follower co-state
                xi(:, 1, k) == xi0]; % initial condition for k-th follower state
            for t = 1:tau
                constr = [constr, qf(:, t, k) == Ef(:, :, t, k)'*qf(:, t+1, k)-Qf(:, :, k)*L(:, :, k)*xl(:, t)]; % k-th follower's co-state dynamics
                constr = [constr, xi(:, t+1, k) == Ef(:, :, t, k)*xi(:, t, k) - Ff(:, :, t, k)*qf(:, t+1, k)]; % k-th follower's state dynamics
            end
        end

        for ind = 1:Dn
            for t = 1:tau+1
                k1 = setD(ind, 1);
                k2 = setD(ind, 2);
                constr = [constr, (xi(:, t, k1) - xi(:, t, k2))'*(Lambdainv(:, :, t, k1)+Lambdainv(:, :, t, k2))*(xi(:, t, k1) - xi(:, t, k2)) <= S(t, ind)];
                % upper bound quadratics
            end
            constr = [constr, sum(sum(S)) - sum(S(:, ind)) <= ups];  % upper bound sum of quadratics
   
        end

        objf1 = 0;

        for t = 1:tau+1
            for k1 = 1:d-1
                for k2 = k1+1:d
                    objf1 = objf1 + (xi_hat(:, t, k1) - xi_hat(:, t, k2))'*(Lambdainv(:, :, t, k1)+Lambdainv(:, :, t, k2))*(xi(:, t, k1) - xi(:, t, k2));

                end
            end
        end

        options = sdpsettings('verbose',0,'solver','mosek');

        solution = optimize(constr, ups - objf1, options);
       
        objval1 = vol(value(xi), Lambdainv);
        objval = max(value(ups)) - objval1; 


        fprintf('Sample %d, CCP iter %d, Value %4.2f \n', ccp_rand, ccp_iter, -objval)

        if objval_hat-objval < ccp_eps
            break
        else
            objval_hat = objval;
        end

    end

    if objval < objval_opt
        w_opt = value(w);
        objval_opt = objval;
    end
end

[ql_opt, xl_opt, qf_opt, xi_opt] = dynprop(El, Fl, Ef, Ff, Ql, Qf, L, x0, xi0, w_opt);

%%
figure('Position',[0,0,800,800])
hold on

for k = 1:d
    num = (k-1)*n0;
    plot(xl_opt(num+1, :), xl_opt(num+2, :), '-square', 'LineWidth', 3)
    scatter(xl_opt(num+1, end), xl_opt(num+2, end), 300, 'k>', 'filled')
end
scatter(xl_opt(1, 1), xl_opt(2, 1), 300, 'ko', 'filled')
hold off

figure('Position',[0,0,800,800])
hold on
for k = 1:d
    plot(xi_opt(1, :, k), xi_opt(2, :, k), '--o', 'LineWidth', 3)
    scatter(xi_opt(1, end, k), xi_opt(2, end, k), 300, 'k>', 'filled')
    scatter(xi_opt(1, 1, k), xi_opt(2, 1, k), 300, 'ko', 'filled')
end

hold off
% set(gca,'Yticklabel',[]) 
% set(gca,'Xticklabel',[])
%axis equal

