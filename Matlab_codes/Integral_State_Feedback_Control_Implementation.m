%% Scripte name: Integral_State_Feedback_Control_Implementation.m
%% Project: Real-Time Velocity Control of DC Motor via State-Feedback
%% Author: Autis-Tech

%% Version:
%   1.0.0 (Initial Release)
% Key Features:
%   - Real-time plotting of reference vs measured velocity
%   - State observer for current and velocity estimation
%   - Comprehensive data logging and performance analysis
%   - Configurable reference signal generation

%% Description:
%   This MATLAB script implements bidirectional serial communication with Arduino board for closed-loop motor control.
%   The controller generates reference velocity signals and receives real-time motor
%   performance data from Arduino, completing the control loop.

%% Functionality:
%   - Generates velocity reference signals (sine wave profile)
%   - Receives from Arduino:
%     1. Filtered motor velocity in RPM (filtredVelocityRPM)
%     2. Motor voltage in volts (motorVoltage)
%   - Implements state-space control algorithm with integral action
%   - Sends PWM control signals to Arduino (controlSignalPWM)

%% Hardware Requirements:
%   - Arduino MEGA 2560 (running companion sketch)
%   - Compatible motor driver (H-bridge)
%   - Encoder for velocity feedback
%   - Voltage divider circuit for voltage sensing
%   - DC motor with appropriate power supply

%% MATLAB Setup:
%   1. Identify correct COM port:
%      a) Check Arduino IDE → Tools → Port
%      b) Or Windows Device Manager → Ports (COM & LPT)
%   2. Verify serial connection:
%      >> s = serialport('COMX', 9600);

%% Usage:
%   1. Upload companion sketch to Arduino MEGA
%   2. Run this MATLAB control script
%   3. Monitor real-time control performance
%   4. Analyze logged data post-experiment

%% Notes:
%   - Ensure matching baud rates (9600 by default)
%   - Verify serial buffer sizes for data integrity
%   - Calibrate voltage sensing parameters for accurate readings
%   - Adjust safety limits according to motor specifications

%% LICENSE
% █████╗ ██╗   ██╗████████╗██╗███████╗  ████████╗███████╗ ██████╗██╗  ██╗
% ██╔══██╗██║   ██║╚══██╔══╝██║██╔════╝  ╚══██╔══╝██╔════╝██╔════╝██║  ██║
% ███████║██║   ██║   ██║   ██║███████╗     ██║   █████╗  ██║     ███████║
% ██╔══██║██║   ██║   ██║   ██║╚════██║     ██║   ██╔══╝  ██║     ██╔══██║
% ██║  ██║╚██████╔╝   ██║   ██║███████║     ██║   ███████╗╚██████╗██║  ██║
% ╚═╝  ╚═╝ ╚═════╝    ╚═╝   ╚═╝╚══════╝     ╚═╝   ╚══════╝ ╚═════╝╚═╝  ╚═╝
% 
% Autis-Tech control firmware for Real-Time Velocity Control of DC Motors via State-Feedback
%
% Copyright (C) 2025 Autis-Tech
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, in version 3.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.



%% Configuration Parameters
% ========================
% Serial Communication Setup
%   1. Identify correct COM port:
%      a) Check Arduino IDE → Tools → Port
%      b) Or Windows Device Manager → Ports (COM & LPT)
%   2. Verify serial connection:
%      >> s = serialport('COMX', 9600);


COM_PORT = "COM6";          % Serial port identifier
BAUD_RATE = 9600;           % Communication baud rate

% Timing Parameters
SAMPLE_TIME = 0.02;         % Control loop sampling time (s)
TOTAL_TIME = 50;            % Experiment duration (s)
dt = 0.001;                 % Integration time step (s)

% Reference Signal
BASE_RPM = 30;              % Base RPM value
RPM_AMPLITUDE = 5;          % Sine wave amplitude
SIGNAL_FREQ = 0.1;          % Reference signal frequency (Hz)      

% State Constraints
MAX_STATE1 = 3;             % Current state limit
MAX_STATE2 = 90;            % Velocity state limit

% Control Constraints
MAX_CONTROL_MAGNITUDE = 130;
MAX_PWM = 254;              % Maximum PWM value
MAX_VOLTAGE = 12;

%% Initialize arrays to store velocity and target speed for plotting
timeStamps = [];
ReferenceVelocities = [];
FeedbackVelocities = [];
estimatedCurrentAs = [];
controlSignalPWMs = [];

% Time parameters
samplingTime = SAMPLE_TIME;
totalTime = TOTAL_TIME;
t = 0:samplingTime:totalTime;

% Initialize variables
previousVelocityErrorRPM = 0;
integratedVelocityError = 0;
deltaT = samplingTime;



clc;
close all;
clear arduino;

%% Define the COM port and baud rate
port = COM_PORT;
baudRate = BAUD_RATE;

% Create the serial port object using serialport function
arduino = serialport(port, baudRate);


% Create a figure for plotting
figure('Position', [0, 10, 790, 920]);
hold on;
xlabel('Time (s)');
ylabel('Motor Velocity (RPM)');
title('Target Velocity Vs Measured Velocity');
grid on;

% Initialize plot handles
ReferenceVelocitiesPlot = plot(nan, nan, 'k--', 'LineWidth', 2, 'DisplayName', 'W desired [RPM]');
FeedbackVelocitiesPlot = plot(nan, nan, 'b', 'LineWidth', 2, 'DisplayName', 'W measured [RPM]');
legend('show', 'Location', 'north');

%% Define DC Motor Parameters (Estimated)
motor_params = load('motor_parameters.mat');
R = motor_params.R_est;
L = motor_params.L_est;
ke = motor_params.Ke_est;
kt = motor_params.Kt_est;
b = motor_params.b_est;
J = motor_params.J_est;

x = [0;0];

%% State-Space Representation
A = [-R/L,  -ke/L;
      kt/J, -b/J];
B = [1/L; 0];
C = [0, 1];
D = 0;

syms ki kw lambda
K = [ki, kw];
A_cl = A - B*K;
char_eq = det(lambda*eye(2) - A_cl);
% Convert to decimal (floating point) with 5 significant digits
char_eq_vpa = vpa(char_eq, 5);
disp('The characteristic equation is:')
disp(char_eq_vpa)

% Augmented matrices
A_aug = [A, zeros(size(A,1),1); -C, 0];
B_aug = [B; 0];

% Controller Tuning
pole1 = -90 + 90*1i;
pole2 = -90 - 90*1i;
poleIntegralTerm = -2.7;

% Compute feedback gains
K_aug = place(A_aug, B_aug, [pole1 pole2 poleIntegralTerm]);
K = K_aug(1:end-1);
k_int = K_aug(end);

%% constant reference velocity (step)
% referenceVelocityRPM = 35;

% Control loop
for k = 1:length(t)
    % Generate a variable reference velocity
    referenceVelocityRPM = BASE_RPM + RPM_AMPLITUDE * sin(2 * pi * SIGNAL_FREQ * t(k));
    
    %% Read a line of data from Arduino
    data = readline(arduino);
    values = split(data, ',');

    if numel(values) == 2
        filtredVelocityRPM = str2double(values(1));
        motorVoltage = str2double(values(2));
    else
        error('Invalid data format: Expected 2 values (filtredVelocityRPM, motorVoltage).');
    end


    %% Current Estimation: Model-Based Luenberger Observer
    % Estimates armature current without physical current sensor    % Tune observer poles (faster than controller)
    obs_poles = [-100 -110];  
    L = place(A', C', obs_poles)';
    % Observer update
    x_dot = A*x + B*motorVoltage + L*(filtredVelocityRPM - C*x);
    % Update State using Euler Integration
    x(1) = x(1) + x_dot(1) * dt;
    x(2) = x(2) + x_dot(2) * dt;
    x(1) = max(0, min(x(1), MAX_STATE1));
    estimatedCurrentA = x(1);

    % compute velocity tracking error
    velocityErrorRPM = referenceVelocityRPM - filtredVelocityRPM;
    
    % Integral
    integratedVelocityError = integratedVelocityError + velocityErrorRPM * deltaT;
    
    % Generate the State-feedback control in VOLT
    u_State_Feedback_Volt = (-K * [estimatedCurrentA; filtredVelocityRPM] - k_int * integratedVelocityError );

    % Convert control unit from VOLT to PWM
    controlSignalPWM = u_State_Feedback_Volt * MAX_PWM / MAX_VOLTAGE;
    controlSignalPWM = max(0, min(controlSignalPWM, MAX_CONTROL_MAGNITUDE));

    %% Send controlSignalPWM to Arduino
    writeline(arduino, string(controlSignalPWM));

    %% Update data arrays
    currentTime = t(k);
    timeStamps = [timeStamps, currentTime];
    ReferenceVelocities = [ReferenceVelocities, referenceVelocityRPM];
    FeedbackVelocities = [FeedbackVelocities, filtredVelocityRPM];
    estimatedCurrentAs = [estimatedCurrentAs, estimatedCurrentA];
    controlSignalPWMs = [controlSignalPWMs, controlSignalPWM];

    %% Update plot data
    set(ReferenceVelocitiesPlot, 'XData', timeStamps, 'YData', ReferenceVelocities);   
    set(FeedbackVelocitiesPlot, 'XData', timeStamps, 'YData', FeedbackVelocities);
    
    %% Wait for the next sampling period
    previousVelocityErrorRPM = velocityErrorRPM;
    pause(samplingTime);
end

% Clean up
clear arduino;
disp("Serial port cleared.");


figure(2);
subplot(3, 1, 1);
plot(timeStamps, ReferenceVelocities, 'k--','LineWidth',2'); % Sensor 1 in red
hold on; 
% grid on;
plot(timeStamps,FeedbackVelocities  , 'b','LineWidth',2); % Sensor 1 in red
xlabel('position','FontSize',12,'FontWeight','bold','Color','k', 'Interpreter','latex','String','\boldmath{Time(s)}');
ylabel('position','FontSize',16,'FontWeight','bold','Color','k', 'Interpreter','latex','String',['$\boldmath\it{w(rpm)}','$']);
% title('position','FontSize',12,'FontWeight','bold','Color','k', 'Interpreter','latex','String','\boldmath{Output Vs refrence}');
axis([0  max(timeStamps) -5  80])

subplot(3, 1, 2);
plot(timeStamps,estimatedCurrentAs , 'b', 'LineWidth', 2);
hold on; 
xlabel('position','FontSize',12,'FontWeight','bold','Color','k', 'Interpreter','latex','String','\boldmath{Time(s)}');
ylabel('position','FontSize',16,'FontWeight','bold','Color','k', 'Interpreter','latex','String',['$\boldmath\it{i(A)}','$']);
% title('position','FontSize',12,'FontWeight','bold','Color','k', 'Interpreter','latex','String','\boldmath{Current}');
axis([0  max(timeStamps) 0  0.1])

subplot(3, 1, 3);
plot(timeStamps,controlSignalPWMs , 'b', 'LineWidth', 2);
hold on; 
xlabel('position','FontSize',12,'FontWeight','bold','Color','k', 'Interpreter','latex','String','\boldmath{Time(s)}');
ylabel('position','FontSize',16,'FontWeight','bold','Color','k', 'Interpreter','latex','String',['$\boldmath\it{v(volt)}','$']);
% title('position','FontSize',12,'FontWeight','bold','Color','k', 'Interpreter','latex','String','\boldmath{Control signal (pwm)}');
axis([0  max(timeStamps) 0  200])
set(gcf, 'Position', [100   100  700 500]); % [Position   Position  x-length   y-length]
set(gcf, 'Color', 'w');
% export_fig figure1.jpg -m 8.7