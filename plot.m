% Read CSV file
data = readmatrix('data.csv'); % Input data location here

% Extract and shift signals
ref = data(:,1) - 180;      % Shift by 180°
output = data(:,2) - 182;   % Shift by 182°
Ts = 0.02;                  % Sampling period = 20 ms
t = (0:length(ref)-1) * Ts; % Time axis (seconds)

% Plot
figure;
plot(t, ref, 'LineWidth', 1.5, 'Color', [1 0.7 0]); hold on;
plot(t, output, 'LineWidth', 1.5, 'Color', [1 0.4 0]);
xlabel('Time (s)');
ylabel('Angle (°)');
title('Reference vs Output (shifted) — Time Axis (Ts = 20 ms)');
legend('Reference (-180° shift)', 'Output (-182° shift)', 'Location', 'best');
grid on;
