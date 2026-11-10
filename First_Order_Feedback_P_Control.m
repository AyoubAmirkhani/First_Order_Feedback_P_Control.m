% First_Order_Feedback_P_Control.m
% Experiment 2: First-order system with unity feedback and proportional control
% Requires MATLAB Control System Toolbox.
%
% Model:
%   G(s) = 1 / (tau*s + 1)
%   C(s) = Kp
%   T(s) = C(s)G(s) / (1 + C(s)G(s))
%
% Edit tau, Kp_values, and simulation settings to match the laboratory setup.

clear; clc; close all;

%% 1) System and simulation settings
tau = 3.3e-3;                         % Open-loop time constant (seconds)
Kp_values = [0.1 0.5 1 2 5 10];      % Proportional gains to compare
reference_amplitude = 1;             % Unit-step amplitude
t_end = 20e-3;                        % Simulation duration (seconds)
dt = 10e-6;                           % Simulation time step (seconds)
t = 0:dt:t_end;

if tau <= 0
    error('The time constant tau must be positive.');
end
if any(Kp_values <= 0)
    error('All proportional gains Kp_values must be positive.');
end

%% 2) Open-loop plant and closed-loop systems
s = tf('s');
G = 1 / (tau*s + 1);                 % First-order plant
T = cell(size(Kp_values));

Kp_column = Kp_values(:);
DCGain = zeros(size(Kp_column));
SteadyStateError = zeros(size(Kp_column));
ClosedLoopTimeConstant_ms = zeros(size(Kp_column));
TheoreticalSettlingTime_ms = zeros(size(Kp_column));
MeasuredSettlingTime_ms = zeros(size(Kp_column));
MeasuredRiseTime_ms = zeros(size(Kp_column));

for k = 1:numel(Kp_values)
    Kp = Kp_values(k);
    C = Kp;
    T{k} = feedback(C*G, 1);         % Unity negative feedback

    DCGain(k) = dcgain(T{k});
    SteadyStateError(k) = abs(reference_amplitude * (1 - DCGain(k)));
    tau_cl = tau / (1 + Kp);
    ClosedLoopTimeConstant_ms(k) = 1e3 * tau_cl;
    TheoreticalSettlingTime_ms(k) = 1e3 * (-log(0.02) * tau_cl);

    info = stepinfo(T{k}, 'SettlingTimeThreshold', 0.02);
    MeasuredSettlingTime_ms(k) = 1e3 * info.SettlingTime;
    MeasuredRiseTime_ms(k) = 1e3 * info.RiseTime;
end

%% 3) Display results
Results = table(Kp_column, DCGain, SteadyStateError, ...
    ClosedLoopTimeConstant_ms, TheoreticalSettlingTime_ms, ...
    MeasuredRiseTime_ms, MeasuredSettlingTime_ms);
disp('Experiment 2: First-order system with proportional feedback');
disp(Results);

%% 4) Compare closed-loop step responses
figure('Name','Closed-loop step responses','Color','w');
hold on; grid on;
for k = 1:numel(Kp_values)
    [y, tout] = step(reference_amplitude*T{k}, t);
    plot(tout, squeeze(y), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('K_p = %g', Kp_values(k)));
end
yline(reference_amplitude, '--k', 'Reference');
xlabel('Time (s)');
ylabel('Output');
title('Effect of proportional gain on the closed-loop step response');
legend('Location','southeast');
hold off;

%% 5) Compare tracking error
figure('Name','Tracking error','Color','w');
hold on; grid on;
for k = 1:numel(Kp_values)
    [y, tout] = step(reference_amplitude*T{k}, t);
    e = reference_amplitude - squeeze(y);
    plot(tout, e, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('K_p = %g', Kp_values(k)));
end
xlabel('Time (s)');
ylabel('Error e(t)');
title('Tracking error for different proportional gains');
legend('Location','northeast');
hold off;

%% 6) Bode plot for a selected proportional gain
selected_index = min(3, numel(Kp_values));
figure('Name','Closed-loop Bode response','Color','w');
bode(T{selected_index});
grid on;
title(sprintf('Closed-loop Bode response (K_p = %g)', ...
    Kp_values(selected_index)));

%% 7) Print the analytical model
fprintf('\nOpen-loop plant: G(s) = 1 / (%.6g*s + 1)\n', tau);
fprintf('Unity-feedback closed-loop model: T(s) = Kp / (tau*s + 1 + Kp)\n');
fprintf('For a unit-step input: e_ss = 1/(1+Kp)\n');
fprintf('Closed-loop time constant: tau_cl = tau/(1+Kp)\n');
