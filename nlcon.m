function [c,ceq] = nlcon(z,h,Tctrlstart,Tctrlend,Tpredstart,Tpredend,x,b) % Computing equality and inequality constraints
global Ts Nc Np NT m n

x1 = x(1,b);
x2 = x(2,b);
   
% Set the simulink optimization options
opt = simset('solver','ode5','SrcWorkspace','Current','FixedStep',h);
    
% Create a column vector for the simulation time period
T = [Tctrlstart:h:Tctrlend]';
    
% Define input vector for simulation that includes time
X = z;

% Define input vector for simulation that includes time
TU = [T X];
    
% With input trajectory defined in TU (control horizon), perform simulation of optimal
% response of the single-link manipulator between tStart and Tfinal (entire
% prediction horizon)
simOut = sim('SingleLinkManipulator', [Tpredstart Tpredend], opt, TU);
    
% Define an array vector of all the inequality constraint functions g(X)

% c = [simOut.x2-5; simOut.x1(end)-4.5; -simOut.x1(end)+3.5];
c = [simOut.x2-5];

% Define an array vector of all the equality constraint functions h(X)
% Here,there are no equality constraints.
ceq = [];
end