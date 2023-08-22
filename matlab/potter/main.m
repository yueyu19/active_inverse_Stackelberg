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
B0 = integral(@(t) expm(t*Ac0), 0, dt, 'ArrayValued', true)*Bc0;

tau = round(1/dt); % length of trajectory

% leader dynamics
d = 5; % number of subsystems for leader

Al = kron(eye(d), A0); % the joint dynamics of d double-integrator agents
Bl = kron(eye(d), B0);
[nl, ml] = size(Bl); % leader's dimension
Omegl = 1e-3*eye(nl); % leader disturbance covariance


% follower dynamics

Af = A0; % one double-integrator
Bf = B0;
[nf, mf] = size(Bf); % follower's dimension
Omegf = 1e-3*eye(nf); % leader disturbance covariance

% leader cost matrices
Ql = kron(eye(d), diag([1; 1; 0; 0]));
Rl = 1e0*eye(ml, ml);

% follower cost matrices
Qf = zeros(nf, nf, d);
Rf = zeros(mf, mf, d);

for k = 1:d
    Qf(:, :, k) = eye(nf); % follower's cost matrices under hypothesis n
    Rf(:, :, k) = 1e-1*eye(mf, mf);
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
        Lambda(:, :, t+1, k) = Ef(:, :, t, k)*Lambda(:, :, t, k)*Ef(:, :, t, k)' + Ff(:, :, t, k) + Omegf; % propogate the follower's state var under hypo k
        Lambdainv(:, :, t+1, k) = pinv(Lambda(:, :, t+1, k));
    end
end


M = zeros(n0, nl, d); % output matrix that maps leader's state to follower's reference state, one for each hypothesis

for k = 1:d
    M(:, (k-1)*n0+1:k*n0, k) = eye(n0);
end

setD = nchoosek(1:d, 2); % set of hypo pairs

%%
% system parameters

% RH setup

RHep = 10; % # of epsidoes of RH game play
MCnum = 1; % number of Monte Carlo for RH games

xl = zeros(nl, RHep+1, MCnum); % leader state traj history
xf = zeros(nf, RHep+1, MCnum); % follower state traj history
xfm = zeros(nf, tau+1, d, RHep, MCnum);

% CCP setup
maxccpinit = 1; % max # of rand seed in CCP

maxu = 2; % max box radius for reference state
ccpmaxiter = 1e2; % max iteration of CCP
ccp_eps = 1e-4; % stopping criterion for CCP
Dn = size(setD, 1); % size of set D
hypo = 1; % choose the hypothesis that governs the follower's dynamics 

for MCiter = 1: MCnum
    for sys_t = 1:RHep
        xl0_hat = xl(:, sys_t, MCiter); % leader's current state
        xf0_hat = xf(:, sys_t, MCiter); % follower's current state

        % initilize CCP

        objval_opt = Inf; % initialize optimal value in CCP

        for ccp_init = 1:maxccpinit % random initialization of CCP (convex-concave procedure)

            if sys_t > 1 
                ul_ccp0 = ul_opt;
            else

                ul_ccp0 = zeros(ml, tau);

                for k = 1:d
                    num = (k-1)*m0;
                    thet = rand*2*pi;
                    ul_ccp0(num+1:num+2, :) =  maxu*[cos(thet); sin(thet)]*ones(1, tau); % reference state constraints
                end
  
            end

            [~, ~, xi_hat] = dynprop(Al, Bl, Ef, Ff, Qf, M, Omegl, xl0_hat, xf0_hat, ul_ccp0); % propagate the leader & follower's states & co-states

            objval_hat = Inf; % initialize optimal value for CCP

            for ccp_iter = 1:ccpmaxiter % CCP iteration

                yalmip('clear')

                ul = sdpvar(ml, tau, 'full'); % leader's costate
                eta = sdpvar(nl, tau+1, 'full'); % leader's state
                qf = sdpvar(nf, tau+1, d,'full'); % hypothesis agent co-state
                xi = sdpvar(nf, tau+1, d, 'full'); % hypothesis agent state
                zeta = sdpvar(nf, tau, Dn, 'full'); % auxiliary variable for quadratic constraints
                S = sdpvar(tau, Dn, 'full'); % slack variable
                ups = sdpvar(1, 1, 'full'); % upper bound for sum of quadratics

                constr = [eta(:, 1) == xl0_hat];    % initial condition for leader's state

                for t = 1:tau
                    constr = [constr, eta(:, t+1) == Al*eta(:, t) + Bl*ul(:, t)]; % leader state dynamics
                end


                for k = 1:d
                    num = (k-1)*m0;

                    for t = 1:tau
                        constr = [constr, norm(ul(num+1:num+m0, t), Inf) <= maxu]; % leader input constraints
                    end

                end


                for k = 1:d
                    constr = [constr, qf(:, tau+1, k) == -Qf(:, :, k)*M(:, :, k)*eta(:, tau+1), ... % final condition for k-th follower co-state
                        xi(:, 1, k) == xf0_hat]; % initial condition for k-th follower state
                    for t = 1:tau
                        constr = [constr, qf(:, t, k) == Ef(:, :, t, k)'*qf(:, t+1, k)-Qf(:, :, k)*M(:, :, k)*eta(:, t)]; % k-th follower's co-state dynamics
                        constr = [constr, xi(:, t+1, k) == Ef(:, :, t, k)*xi(:, t, k) - Ff(:, :, t, k)*qf(:, t+1, k)]; % k-th follower's state dynamics
                    end
                end

                for ind = 1:Dn
                    for t = 1:tau
                        k1 = setD(ind, 1);
                        k2 = setD(ind, 2);
                        W0 = sqrtm(Lambdainv(:, :, t+1, k1)+Lambdainv(:, :, t+1, k2));
                        constr = [constr, zeta(:, t, ind) == W0*(xi(:, t+1, k1) - xi(:, t+1, k2))];
                        constr = [constr, zeta(:, t, ind)'*zeta(:, t, ind) <= S(t, ind)]; % upper bound quadratics
                    end
                    constr = [constr, sum(sum(S)) - sum(S(:, ind)) <= ups];  % upper bound sum of quadratics

                end


                lvol = 0;
                lcost = 0;

                for t = 1:tau-1
                    lcost = lcost + (ul(:, t+1) - ul(:, t))'*Rl*(ul(:, t+1) - ul(:, t));
                end

                for t = 1:tau+1

                    for k1 = 1:d-1
                        for k2 = k1+1:d
                            lvol = lvol + 2*(xi_hat(:, t, k1) - xi_hat(:, t, k2))'*(Lambdainv(:, :, t, k1)+Lambdainv(:, :, t, k2))*(xi(:, t, k1) - xi(:, t, k2));

                        end
                    end
                end

                options = sdpsettings('verbose',0,'solver','mosek');


                solution = optimize(constr, lcost + ups - lvol, options);

                mindist = vol(value(xi), Lambdainv)-value(ups);

                objval =  value(lcost) - mindist;


                fprintf('MC iter %d, Syst Time %d, CCP Init %d, CCP Iter %d, Min Diff %4.2f, Input Rate %4.2f\n', ...
                    MCiter, sys_t, ccp_init, ccp_iter, mindist, value(lcost))

                if (objval_hat-objval)/abs(objval) < ccp_eps
                    break
                else
                    objval_hat = objval;
                    xi_hat = value(xi);
                end

            end

            if objval < objval_opt
                ul_opt = value(ul);
                objval_opt = objval;
            end

% %             if ccp_init == 1 && sys_t > 1
% %                 break % if CCP is warm-started, no need to try other random initialization
% %             end
        end

        [xl_opt, qf_opt, xi_opt] = dynprop(Al, Bl, Ef, Ff, Qf, M, Omegl, xl0_hat, xf0_hat, ul_opt); % propagate the leader & follower's states & co-states
        xfm(:, :, :, sys_t, MCiter) = xi_opt; 


        % simulate leader's dynamics

        xl(:, sys_t+1, MCiter) = xl_opt(:, 2);

        % simulate the follower's synamics
       
        Sigma = pinv(Rf(:, :, hypo)+Bf'*Pf(:, :, 2, hypo)*Bf);
        mu = -Sigma*Bf'*(Pf(:, :, 2, hypo)*Af*xf(:, sys_t, MCiter) + qf_opt(:, 2, hypo));
        xf(:, sys_t+1, MCiter) = Af*xf(:, sys_t, MCiter) + Bf*mvnrnd(mu, Sigma)'...
                                 +mvnrnd(zeros(nf, 1), Omegf)'; % simulate next state of follower


    end
end



%%

figure('Position',[0,0,800,800])
hold on

for k = 1:d
    num = (k-1)*n0;
    plot(xl(num+1, :, 1), xl(num+2, :, 1), '-square', 'LineWidth', 3)
end
plot(xf(1, :, 1), xf(2, :, 1), '-ko', 'LineWidth', 3)

hold off
grid on

xl_pos = kron(eye(d), diag([1 1 0 0]))*xl(:, :, 1);
% 
ub = min(1+max(abs(xl_pos(:))), 1.2*max(abs(xl_pos(:))));
xlim([-ub ub])
ylim([-ub ub])

yalmip('clear')
%
filename = ['RHGdata1_', num2str(d), '.mat'];

save(filename)

