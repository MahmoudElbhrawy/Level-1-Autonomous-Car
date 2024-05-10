%*******************************************************************
%*** Solving Differentially-Steered Vehicle kinematics equations ***
%*******************************************************************

% Clearing workspace, closing figures, and clearing command window
close all
clear
clc

% Parameters of the vehicle
r = 3.25; % wheel radius
l = 8;    % distance between wheel and center

% Wheel velocities
fhiRDot = 5; % right wheel velocity
fhiLDot = 5;    % left wheel velocity

% Initial position and orientation of the vehicle
x0 = 0;
y0 = 0;
theta0 = pi/2;

% Time span for simulation
tspan = [0 5];

% Calculating translational and rotational velocities
xrdot = (r/2) * (fhiRDot + fhiLDot);
yrdot = 0;
omegadot = (r/(2 * l)) * (fhiRDot - fhiLDot);

% Symbolic representation of position and orientation
syms x(t) y(t) theta(t)

% Defining kinematic differential equations
ode1 = diff(x) == cos(theta) * xrdot;
ode2 = diff(y) == sin(theta) * xrdot;
ode3 = diff(theta) == omegadot;
odes = [ode1; ode2; ode3];

% Initial conditions
cond1 = x(0) == x0;
cond2 = y(0) == y0;
cond3 = theta(0) == theta0;
conds = [cond1; cond2; cond3];

% Solving ODEs
S = dsolve(odes,conds);

% Displaying and plotting results
disp('x(t)=')
disp(S.x)
disp('y(t)=')
disp(S.y)
disp('theta(t)=')
disp(S.theta)

% Plotting x, y, and theta against time
fplot(S.x, tspan, 'b', 'linewidth', 2),  hold on
fplot(S.y, tspan, 'r', 'linewidth', 2)
fplot(S.theta, tspan, 'g', 'linewidth', 2),  grid on
legend('x', 'y', 'theta', 'Location', 'best')
title('x, y, and theta with respect to time')

% Plotting the XY trajectory
figure
fplot(S.x, S.y, tspan, 'linewidth', 2), grid on
title('XY trajectory')
