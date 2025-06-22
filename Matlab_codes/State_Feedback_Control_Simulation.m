clc;
close all;

%% Constants
dt = 0.001;
MAX_VOLTAGE = 12;
MAX_PWM = 254;
SAMPLE_TIME = 0.02;
FREQUENCY = 0.1;

% State Constraints
MAX_STATE1 = 3;             % Current state limit
MAX_STATE2 = 90;            % Velocity state limit

%% Motor Parameters
motor_params = load('motor_parameters.mat');
R = motor_params.R_est;  
L = motor_params.L_est;
ke = motor_params.Ke_est;
kt = motor_params.Kt_est;
b = motor_params.b_est;
J = motor_params.J_est;

% % Given parameters
% R = 19.300;       % Ohms
% L = 0.8747;       % Henry
% J = 2.2849e-6;    % kg·m²
% b = 1.2936e-4;    % N·m/rpm
% ke = 0.1005;      % V/rpm
% kt = 0.1006;      % N·m/A

%% State-Space Representation
A = [-R/L,  -ke/L;
     kt/J, -b/J];
B = [1/L; 0];
C = [0, 1];  % Only velocity is measurable
D = 0;

sys = ss(A, B, C, D);

%% Controller Design
pole1 = -95 + 95*1i;
pole2 = -95 - 95*1i;
K = place(A,B,[pole1 pole2]);
N = -1 / (C * inv(A - B*K) * B);  % Reference Gain

%% Simulation Setup
t = 0:SAMPLE_TIME:50;
% Constant reference velocity (step)
% r = 35* ones(size(t));
% Generate a variable reference velocity
r = 30 + 5 * sin(2 * pi * FREQUENCY * t);

%% Simulation Variables
x = [0; 0];
x_hist = zeros(size(A,1), length(t));
u_hist = zeros(1, length(t));

%% Simulation Loop
for i = 1:length(t)-1
    % Control Input Calculation
    u_volt = -K * x + N * r(i);
    u_pwm = u_volt*MAX_PWM/MAX_VOLTAGE;
    u_arduino = 0.0200*u_pwm - 0.0381;
    u_drive = 0.2282*u_arduino.^3 - 2.6181*u_arduino.^2 + 10.3600*u_arduino - 4.5332;
    u = u_drive;

    % State Update
    x_dot = A * x + B * u - [0;0]; % u in [volt]   
    x = x + x_dot * dt;
    x(1) = max(0, min(x(1), MAX_STATE1));
    x(2) = max(0, min(x(2), MAX_STATE2)); 
    x = [x(1);x(2)];

    % Data Storage
    x_hist(:, i) = x;
    u_hist(i) = u;
end

%% Plotting
figure(1);
subplot(3,1,1);
plot(t, r, 'k--', 'LineWidth', 1.5); 
hold on;
plot(t, x_hist(2,:), 'b', 'LineWidth', 1.5);
axis([0 15 0 42.6])

legend('$\it{y}_{ref}$','$\it{y(t)}$: Closed-loop Output','Interpreter','latex',...
    'Location','southeast','NumColumns',1,'FontSize',12);

xlabel('position','FontSize',12,'FontWeight','bold','Color','k', 'Interpreter','latex','String','\boldmath{Time(s)}');
ylabel('position','FontSize',16,'FontWeight','bold','Color','k', 'Interpreter','latex','String',['$\boldmath\it{w(rpm)}','$']);

subplot(3,1,2);
plot(t, x_hist(1,:), 'b', 'LineWidth', 1.5);
xlabel('position','FontSize',12,'FontWeight','bold','Color','k', 'Interpreter','latex','String','\boldmath{Time(s)}');
ylabel('position','FontSize',16,'FontWeight','bold','Color','k', 'Interpreter','latex','String',['$\boldmath\it{i(Amper)}','$']);
axis([0 15 0 0.1])

subplot(3,1,3);
plot(t, u_hist, 'b', 'LineWidth', 1.5);
axis([0 15 0 12])
xlabel('position','FontSize',12,'FontWeight','bold','Color','k', 'Interpreter','latex','String','\boldmath{Time(s)}');
ylabel('position','FontSize',16,'FontWeight','bold','Color','k', 'Interpreter','latex','String',['$\boldmath\it{u(Volt)}','$']);

set(gcf, 'Position', [100 100 700 500]);
set(gcf, 'Color', 'w');