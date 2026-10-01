function f = obj_fun(z,h,Tctrlstart,Tctrlend,Tpredstart,Tpredend,x,b) % Computing the objective function
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
    
mass = 3;

% Weighting matrix D will be used to compute the Objective function
% NB: Weighting matrix not used in this problem
% w = ones(1,length(x1(end)));
% w(length(w)) = 100;
% D = diag(w);
% Calculate the objective function value
% Let's avoid using a 'for' loop and encourage computational efficiency
% by computing in vectorized form
   

f = 0.2*trapz(simOut.tout, (simOut.u_optim).^2)/300 - 0.8*mass + (simOut.x1(end)-4)^2; % Maximize the mass by subtracting from the objective function
                                    % Also remember to scale the objective function
                                        
                                    % objective                                                             
end