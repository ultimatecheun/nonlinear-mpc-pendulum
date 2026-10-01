% MATLAB program for Noninear MPC: Simple pendulum system 

clc;
clear all;
close all

global Ts Nc Np NT m n x0

%% Define Variables and Parameters
% We try to define the final time for simulation
Tfinal = 2; % Time in seconds (s)

% Set integration step-size parameter
h = 0.05; % Also in seconds (s)

% Create a column vector for the simulation time period
Tsim = [0:h:Tfinal]';

load('u0.mat');
% Define a variable "Nsim" which will be the number of steps in the simulation
Nsim = length(Tsim);

% Now, set the optimization options alongside the tolerance on the decision
% vector "TolX" and also for the function values "TolFun"
% NB: These tolerances are used to determine optimization algorithm's
% convergence
% options = optimoptions(@fmincon,'Algorithm','interior-point','Display','iter',...
%     'MaxFunctionEvaluations',90000,'FiniteDifferenceStepSize',1e-3);

Ts = h;                     % Step size for time discretization a.k.a Sampling time
Nc = 8;
Np = Nc;                    % Total length of control horizon = prediction horizon
NT = Nsim;                  % NT is needed for the whole "for-loop" execution
m = 1;                      % m is number of control decision variables to be discretized
n = 2;                      % n is number of design decision variables, or states

x0 = [0;0];                 % Initial state - 2 elements, x1 and x2
x = zeros(n,NT+1);          % Spans simulation time frame, for control trajectory
x(:,1) = x0;


% Initialize control values (for input trajectory)
% say Zero (0) in this case
U = zeros(NT,m);            % Large basket to be storing applied control inputs
                            % This is the torque, control signal in "Nm" (Newton meters)
                                  
% Define the upper and lower bounds for the decision variables "control trajectory"
umax = 50*ones(Nsim,m);     % Also in "Nm"
umin = -50*ones(Nsim,m);


Uk = u0(1:Nc);         % Corresponds to size of control horizon
zk = [Uk];


Tctrlstart = Tsim(1);
Tctrlend = Tsim(Nc);

Tpredstart = Tsim(1);
Tpredend = Tsim(Np);

b = 0; % Initializing counter for MPC iterations

tic                         % Trying to monitor simulation runtime
% simulating system with MPC
for k = 1:NT-Nc             % Intend to extrapolate last set of values
   
   b = b+1;                 % MATLAB indexing starts from 1 to be updated for each iteration
   
   fun = @(z)obj_fun(z,h,Tctrlstart,Tctrlend,Tpredstart,Tpredend,x,b);
   nonlcon = @(z)nlcon(z,h,Tctrlstart,Tctrlend,Tpredstart,Tpredend,x,b);
   lb = umin(1:length(Uk)); % Pay attention to bounds if solving for multiple control vars
   ub = umax(1:length(Uk)); % Pay attention to bounds if solving for multiple control vars
   
   opts = optimoptions(  @fmincon,...
                         'Algorithm',                   'sqp',... % or 'interior-point'
                         'UseParallel',                 true,... % to use Parallel computing or NOT
                         'Display',                     'iter',...
                         'FiniteDifferenceType',        'forward',...
                         'ScaleProblem',                'none',...
                         'MaxFunctionEvaluations',      90000,...
                         'MaxIterations',               500,...
                         'StepTolerance',               1e-5,...
                         'ConstraintTolerance',         1e-5,...
                         'OptimalityTolerance',         1e-5,...
                         'FiniteDifferenceStepSize',    1e-3);    % NB: The effect of step size
                     
   z = fmincon(fun,zk,[],[],[],[],lb,ub,nonlcon,opts);
   
   U(k) = z(1);             % Grab first element and apply to system
   % zk = z;                % Restore guess for next optimization problem
   
   x1 = x(1,b);
   x2 = x(2,b);
   opt = simset('solver','ode5','SrcWorkspace','Current','FixedStep',h);
   simApply = sim('SingleLinkManipulator', [Tsim(k) Tsim(k+1)], opt, [Tsim(k) U(k)]);
   
   % Now, go on ahead to update states in x
   x(1,b+1) = simApply.x1(end);
   x(2,b+1) = simApply.x2(end);
   
   
   if 1+k < NT
       Tctrlstart = Tsim(1+k);  % Shift control horizon
   else
       Tctrlstart = Tsim(NT);   % Implement Receding Horizon
   end
   
   if Nc+k < NT
       Tctrlend = Tsim(Nc+k);   % Shift control horizon
   else
       Tctrlend = Tsim(NT);     % Implement Receding Horizon
   end
   

   if 1+k < NT
       Tpredstart = Tsim(1+k);  % Shift prediction horizon
   else
       Tpredstart = Tsim(NT);   % Implement Receding Horizon
   end
                                % Prediction always starts from simulation start time, most times zero (0) NOT SURE!!!
   
   if Np+k < NT
       Tpredend = Tsim(Np+k);   % Shift prediction horizon
   else
       Tpredend = Tsim(NT);     % Implement Moving Horizon
   end
       
end    

runTime = toc

% Perform extrapolation to obtain remaining unknown values for optimal
% control trajectory
X = interp1(Tsim(1:NT-Nc),U(1:NT-Nc),Tsim,'pchip');


% Define input vector for simulation that includes time
TU = [Tsim X];

% Set the simulink optimization options
opt = simset('solver','ode5','SrcWorkspace','Current','FixedStep',h);

% With input trajectory defined in TU, perform simulation of optimal
% response of the single-link manipulator between tStart and Tfinal

x1 = 0;     % Restore states to initial value and observe control trajectory
x2 = 0;
simOut = sim('SingleLinkManipulator', [0 Tfinal], opt, TU);
% Weighting matrix D will be used to compute the Objective function
w = ones(1,length(simOut.x1(end)));
w(length(w)) = 100;
D = diag(w);
    
% Calculate the objective function value
% Let's avoid using a 'for' loop and encourage computational efficiency
% by computing in vectorized form
mass = 3;
fval = 0.2*trapz(simOut.tout, (simOut.u_optim).^2)/300 - 0.8*mass + (simOut.x1(end)-4)^2; 


% Save results
Output.optimal.time           = Tsim;
Output.optimal.controls.u     = X;
Output.optimal.objective      = fval;

Output.optimal.simOut         = simOut;
Output.Run_time               = runTime;

save('Det_Output','Output')

%% Visulaizing the results obtained
subplot(3,1,1)
plot(Tsim,simOut.x1,'r','LineWidth',1.5)
xlabel('time(s)')
ylabel('x_1')
grid on
title('Optimal states and control signal')

subplot(3,1,2)
plot(Tsim,simOut.x2,'b','LineWidth',1.5)
xlabel('time(s)')
ylabel('x_2')
grid on

subplot(3,1,3)
plot(Tsim,X,'m','LineWidth',1.5)
xlabel('time(s)')
ylabel('u')
grid on