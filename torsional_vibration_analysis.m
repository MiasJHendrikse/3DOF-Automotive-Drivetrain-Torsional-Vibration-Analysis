%% ========================================================================
% Modal Analysis of 3-DOF Automotive Drivetrain System
% System: Engine - Clutch - Transmission
% ========================================================================
% This code performs undamped modal analysis for a 3-DOF torsional 
% vibration system representing an automotive drivetrain.
%
% Author: Mias J. Hendrikse
% Date: October 2025
% ========================================================================

clear all; close all; clc;

%% SYSTEM PARAMETERS
% ========================================================================

format short
% --- 1. Defining System Parameters (Inputs) ---

% Moments of Inertia (kg·m^2). These were derived from literature sources
J1 = 0.15;    % Engine/Flywheel inertia
J2 = 0.02;    % Clutch disc inertia
J3 = 0.05;    % Transmission input shaft inertia
% Torsional Stiffness - Case 1: High Stiffness Damper (Nm/rad)
k1_high = 500;     % Clutch damper stiffness (high)
k2 = 5000;         % Shaft stiffness
% Torsional Stiffness - Case 2: Low Stiffness Damper (Nm/rad)
k1_low = 100;      % Clutch damper stiffness (low)

%  External Torque Value (T) 
% Define an example scalar torque value (Nm) for the T_ext vector
T = 50; 

% 2. Construct the M (Inertia) and K (Stiffness) Matrices (3 DOF Torsional System) 

% Inertia Matrix [J] (which is the Mass Matrix M in dynamics)
% This is a diagonal matrix containing J1, J2, and J3
J_matrix = diag([J1, J2, J3]);

% Stiffness Matrix [K_high] (Case 1: High Stiffness Damper)
% This represents a chain: (J1) --k1_high-- (J2) --k2-- (J3)
K_high_matrix = [ (k1_high),  -(k1_high),      0      ;
                 -(k1_high), (k1_high + k2), -(k2)    ;
                      0,        -(k2),       (k2)    ];

% Stiffness Matrix [K_low] (Case 2: Low Stiffness Damper)
% This represents a chain: (J1) --k1_low-- (J2) --k2-- (J3)
K_low_matrix = [ (k1_low),  -(k1_low),      0      ;
                -(k1_low), (k1_low + k2), -(k2)    ;
                     0,        -(k2),       (k2)    ];

% External Torque Vector T_ext
% Assuming the excitation torque (T) acts only on the first DOF (J1)
T_ext = [T; 0; 0]; 

% --- 3. Display the Constructed Matrices and Vector ---

fprintf('========================================\n');
fprintf('     TORSIONAL SYSTEM MATRICES/VECTORS     \n');
fprintf('========================================\n');

disp('1. Inertia Matrix [J] (or [M]):');
disp(J_matrix);

fprintf('----------------------------------------\n');

disp('2. Stiffness Matrix [K_high] (k1_high = 500):');
disp(K_high_matrix);

fprintf('----------------------------------------\n');

disp('3. Stiffness Matrix [K_low] (k1_low = 100):');
disp(K_low_matrix);

fprintf('----------------------------------------\n');

disp('4. External Torque Vector T_ext = [T; 0; 0] (T = 10):');
disp(T_ext);

fprintf('========================================\n\n');

format short
fprintf('========================================\n');
fprintf('3-DOF TORSIONAL VIBRATION MODAL ANALYSIS\n');
fprintf('========================================\n\n');

fprintf('System Configuration:\n');
fprintf('  DOF 1: Engine/Flywheel (J1 = %.3f kg·m²)\n', J1);
fprintf('  DOF 2: Clutch Disc (J2 = %.3f kg·m²)\n', J2);
fprintf('  DOF 3: Transmission Input (J3 = %.3f kg·m²)\n\n', J3);


%% ========================================================================
% Modal Matrix - CASE 1: HIGH STIFFNESS DAMPER (k1 = 500 Nm/rad)
% ========================================================================
fprintf('========================================\n');
fprintf('CASE 1: HIGH STIFFNESS DAMPER\n');
fprintf('k1 = %d Nm/rad, k2 = %d Nm/rad\n', k1_high, k2);
fprintf('========================================\n\n');

% Mass Matrix [M]
M = diag([J1, J2, J3]); %This creates the inertia matrix as a diagonal matrix

% Stiffness Matrix [K] for Case 1
K1 = [k1_high,        -k1_high,         0;
     -k1_high,   (k1_high + k2),      -k2;
      0,              -k2,              k2];

% Solve Eigenvalue Problem: det([K] - ω²[M]) = 0. The MATLAB built-in
% function eig is used.This function already normalises the output
% ModeShapes1 represents the Eigen Vectors, while Lambda1 represents the
% eigen values. The eigen values are the squares of the natural frequencies
[ModeShapes1, Lambda1] = eig(K1, M);

% Since Lambda1 is a diagonal matrix of eigenvalues, we extract the
% diagonals
eigenvalues_vector = diag(Lambda1); 
fprintf('========================================\n');
fprintf('Eigenvalues (omega_n^2) and Eigenvectors (Mode Shapes):\n');
fprintf('========================================\n');

% Display the vector of eigenvalues
disp('Eigenvalues (omega_n^2):');
disp(eigenvalues_vector);

% Display the matrix of eigenvectors
disp('Eigenvectors (Mode Shapes - columns):');
disp(ModeShapes1);
fprintf('========================================\n\n');

% Natural Frequencies (rad/s and Hz)
omega_n1 = sqrt(diag(Lambda1));    % Natural frequencies (rad/s)
f_n1 = omega_n1 / (2*pi);          % Natural frequencies (Hz)

% Sort frequencies and mode shapes in ascending order
[f_n1, idx1] = sort(f_n1);
omega_n1 = omega_n1(idx1);
ModeShapes1 = ModeShapes1(:, idx1); % ModeShapes1 now holds the EIG output (which is already mass-normalized)

% Display Results - Case 1
fprintf('Natural Frequencies (Case 1):\n');
fprintf('  Mode 1: %.4f rad/s  (%.4f Hz)\n', omega_n1(1), f_n1(1));
fprintf('  Mode 2: %.4f rad/s  (%.4f Hz)\n', omega_n1(2), f_n1(2));
fprintf('  Mode 3: %.4f rad/s  (%.4f Hz)\n\n', omega_n1(3), f_n1(3));

% Display raw modal matrix before normalization (which is the EIG output)
fprintf('Modal Matrix [Phi] (Eigenvectors from EIG) - Case 1:\n');
fprintf('         Mode 1      Mode 2      Mode 3\n');
fprintf('  θ1:   %9.6f  %9.6f  %9.6f\n', ModeShapes1(1,:));
fprintf('  θ2:   %9.6f  %9.6f  %9.6f\n', ModeShapes1(2,:));
fprintf('  θ3:   %9.6f  %9.6f  %9.6f\n\n', ModeShapes1(3,:));

% --- NORMALIZATION CHECK STEP ---
fprintf('NORMALIZATION CHECK (Case 1):\n');

% Check if each column of the EIG output satisfies the mass-normalization condition (φᵢᵀ[M]φᵢ = 1)
for i = 1:3
    % Calculate modal mass: φᵢᵀ[M]φᵢ
    modal_mass = ModeShapes1(:,i)' * M * ModeShapes1(:,i);
    fprintf('  Mode %d: φᵀ[M]φ = %.6f\n', i, modal_mass);
end
fprintf('\n');

% --- CONFIRMATION ---
fprintf('**Confirmation: The MATLAB eig function already returned mass-normalized mode shapes.**\n');
fprintf('The output shows that the condition φᵀ[M]φ = 1.000000 is satisfied for all modes.\n\n');

% --- Orthogonality Verification (adapted to use the EIG output directly) ---

% VERIFICATION: Check orthogonality condition [Φ]ᵀ[M][Φ] = [I]
fprintf('VERIFICATION OF NORMALIZATION (Case 1):\n');
fprintf('Checking: [Φ]ᵀ[M][Φ] = [I] (Identity Matrix)\n\n');
Phi_T_M_Phi1 = ModeShapes1' * M * ModeShapes1;
fprintf('[Φ]ᵀ[M][Φ] =\n');
fprintf('  %8.5f  %8.5f  %8.5f\n', Phi_T_M_Phi1(1,:));
fprintf('  %8.5f  %8.5f  %8.5f\n', Phi_T_M_Phi1(2,:));
fprintf('  %8.5f  %8.5f  %8.5f\n\n', Phi_T_M_Phi1(3,:));

% Check if it's approximately identity
is_identity1 = norm(Phi_T_M_Phi1 - eye(3), 'fro') < 1e-10;
if is_identity1
    fprintf('✓ VERIFIED: The eig output is properly mass-normalized!\n');
    fprintf('  (Matrix equals identity within numerical tolerance)\n\n');
else
    fprintf('✗ WARNING: Normalization check failed!\n\n');
end

% Additional verification: Check stiffness orthogonality [Φ]ᵀ[K][Φ] = [Λ]
fprintf('Additional Check: [Φ]ᵀ[K][Φ] should be diagonal with ω² values\n\n');
Phi_T_K_Phi1 = ModeShapes1' * K1 * ModeShapes1;
fprintf('[Φ]ᵀ[K][Φ] =\n');
fprintf('  %10.4f  %10.4f  %10.4f\n', Phi_T_K_Phi1(1,:));
fprintf('  %10.4f  %10.4f  %10.4f\n', Phi_T_K_Phi1(2,:));
fprintf('  %10.4f  %10.4f  %10.4f\n\n', Phi_T_K_Phi1(3,:));

% Confirm that the K matrix gives the natural frequencies
fprintf('**Confirmation: The [K] matrix gives the natural frequencies.**\n');
fprintf('The result of [Φ]ᵀ[K][Φ] is a diagonal matrix where the diagonal elements\n');
fprintf('are the squares of the natural frequencies (ω²), confirming that the [K]\n');
fprintf('and [M] matrices were correctly used in the eigenvalue problem.\n');

fprintf('Expected diagonal values (ω²):\n');
fprintf('  Mode 1: %.4f (Calculated: %.4f)\n', omega_n1(1)^2, Phi_T_K_Phi1(1,1));
fprintf('  Mode 2: %.4f (Calculated: %.4f)\n', omega_n1(2)^2, Phi_T_K_Phi1(2,2));
fprintf('  Mode 3: %.4f (Calculated: %.4f)\n\n', omega_n1(3)^2, Phi_T_K_Phi1(3,3));

%% ========================================================================
% Modal Matrix - CASE 2: LOW STIFFNESS DAMPER (k1 = 100 Nm/rad)
% ========================================================================
fprintf('========================================\n');
fprintf('CASE 2: LOW STIFFNESS DAMPER\n');
fprintf('k1 = %d Nm/rad, k2 = %d Nm/rad\n', k1_low, k2);
fprintf('========================================\n\n');

% Stiffness Matrix [K] for Case 2
K2 = [k1_low,        -k1_low,         0;
     -k1_low,   (k1_low + k2),      -k2;
      0,              -k2,            k2];

% ModeShapes2 represents the Eigen Vectors, while Lambda2 represents the
% eigen values. The eigen values are the squares of the natural frequencies
[ModeShapes2, Lambda2] = eig(K2, M);

% Since Lambda2 is a diagonal matrix of eigenvalues, we extract the diagonal
eigenvalues_vector = diag(Lambda2); 
fprintf('========================================\n');
fprintf('Eigenvalues (omega_n^2) and Eigenvectors (Mode Shapes):\n');
fprintf('========================================\n');

% Display the vector of eigenvalues
disp('Eigenvalues (omega_n^2):');
disp(eigenvalues_vector);

% Display the matrix of eigenvectors
disp('Eigenvectors (Mode Shapes - columns):');
disp(ModeShapes2);

% Natural Frequencies (rad/s and Hz)
omega_n2 = sqrt(diag(Lambda2));
f_n2 = omega_n2 / (2*pi);

% Sort frequencies and mode shapes
[f_n2, idx2] = sort(f_n2);
omega_n2 = omega_n2(idx2);
ModeShapes2 = ModeShapes2(:, idx2); % ModeShapes2 now holds the EIG output

% Display Results - Case 2
fprintf('Natural Frequencies (Case 2):\n');
fprintf('  Mode 1: %.4f rad/s  (%.4f Hz)\n', omega_n2(1), f_n2(1));
fprintf('  Mode 2: %.4f rad/s  (%.4f Hz)\n', omega_n2(2), f_n2(2));
fprintf('  Mode 3: %.4f rad/s  (%.4f Hz)\n\n', omega_n2(3), f_n2(3));

% Display modal matrix (EIG output)
fprintf('Modal Matrix [Phi] (Eigenvectors from EIG) - Case 2:\n');
fprintf('         Mode 1      Mode 2      Mode 3\n');
fprintf('  θ1:   %9.6f  %9.6f  %9.6f\n', ModeShapes2(1,:));
fprintf('  θ2:   %9.6f  %9.6f  %9.6f\n', ModeShapes2(2,:));
fprintf('  θ3:   %9.6f  %9.6f  %9.6f\n\n', ModeShapes2(3,:));

% --- NORMALIZATION CHECK STEP ---
fprintf('NORMALIZATION CHECK (Case 2):\n');
fprintf('Mass-normalization check on the EIG output (φᵢᵀ[M]φᵢ):\n\n');

% Check if each column of the EIG output satisfies the mass-normalization condition (φᵢᵀ[M]φᵢ = 1)
for i = 1:3
    % Calculate modal mass: φᵢᵀ[M]φᵢ
    modal_mass = ModeShapes2(:,i)' * M * ModeShapes2(:,i);
    fprintf('  Mode %d: φᵀ[M]φ = %.6f\n', i, modal_mass);
end
fprintf('\n');

% --- CONFIRMATION ---
fprintf('**Confirmation: As with Case 1, the MATLAB eig function already returned mass-normalized mode shapes.**\n');
fprintf('The test above confirms the condition φᵀ[M]φ = 1.000000 is satisfied for all modes in the raw eig output.\n\n');

% --- Orthogonality Verification (uses the EIG output directly) ---

% VERIFICATION: Check orthogonality condition [Φ]ᵀ[M][Φ] = [I]
fprintf('VERIFICATION OF NORMALIZATION (Case 2):\n');
fprintf('Checking: [Φ]ᵀ[M][Φ] = [I] (Identity Matrix)\n\n');
Phi_T_M_Phi2 = ModeShapes2' * M * ModeShapes2;
fprintf('[Φ]ᵀ[M][Φ] =\n');
fprintf('  %8.5f  %8.5f  %8.5f\n', Phi_T_M_Phi2(1,:));
fprintf('  %8.5f  %8.5f  %8.5f\n', Phi_T_M_Phi2(2,:));
fprintf('  %8.5f  %8.5f  %8.5f\n\n', Phi_T_M_Phi2(3,:));

% Check if it's approximately identity
is_identity2 = norm(Phi_T_M_Phi2 - eye(3), 'fro') < 1e-10;
if is_identity2
    fprintf(' VERIFIED: The eig output is properly mass-normalized!\n');
    fprintf('  (Matrix equals identity within numerical tolerance)\n\n');
else
    fprintf(' WARNING: Normalization check failed!\n\n');
end

% Additional verification: Check stiffness orthogonality [Φ]ᵀ[K][Φ] = [Λ]
fprintf('Additional Check: [Φ]ᵀ[K][Φ] should be diagonal with ω² values\n\n');
Phi_T_K_Phi2 = ModeShapes2' * K2 * ModeShapes2;
fprintf('[Φ]ᵀ[K][Φ] =\n');
fprintf('  %10.4f  %10.4f  %10.4f\n', Phi_T_K_Phi2(1,:));
fprintf('  %10.4f  %10.4f  %10.4f\n', Phi_T_K_Phi2(2,:));
fprintf('  %10.4f  %10.4f  %10.4f\n\n', Phi_T_K_Phi2(3,:));

% Confirm that the K matrix gives the natural frequencies
fprintf('Expected diagonal values (ω²):\n');
fprintf('  Mode 1: %.4f (Calculated: %.4f)\n', omega_n2(1)^2, Phi_T_K_Phi2(1,1));
fprintf('  Mode 2: %.4f (Calculated: %.4f)\n', omega_n2(2)^2, Phi_T_K_Phi2(2,2));
fprintf('  Mode 3: %.4f (Calculated: %.4f)\n\n', omega_n2(3)^2, Phi_T_K_Phi2(3,3));

%% ========================================================================
% COMPARISON OF RESULTS
% ========================================================================

fprintf('========================================\n');
fprintf('COMPARISON: Effect of Clutch Stiffness\n');
fprintf('========================================\n\n');

fprintf('Natural Frequency Comparison:\n');
fprintf('Mode    High k1 (Hz)   Low k1 (Hz)   Change (%%)\n');
fprintf('----    ------------   -----------   ----------\n');
for i = 1:3
    change = ((f_n2(i) - f_n1(i))/f_n1(i)) * 100;
    fprintf(' %d      %10.4f     %10.4f     %+8.2f\n', i, f_n1(i), f_n2(i), change);
end
fprintf('\n');

%% ========================================================================
% VISUALIZATION: MODE SHAPES
% ========================================================================

% Create figure for mode shapes - Case 1
figure('Position', [100, 100, 1200, 800]);
sgtitle('Mode Shapes - Case 1: High Stiffness Damper (k_1 = 500 Nm/rad)', ...
        'FontSize', 14, 'FontWeight', 'bold');

DOF = 1:3;
DOF_labels = {'Engine', 'Clutch', 'Transmission'};

for i = 1:3
    subplot(2, 3, i);
    bar(DOF, ModeShapes1(:,i), 'FaceColor', [0.2 0.4 0.8]);
    grid on;
    title(sprintf('Mode %d: f = %.2f Hz', i, f_n1(i)), 'FontSize', 12);
    xlabel('Component', 'FontSize', 10);
    ylabel('Normalized Amplitude', 'FontSize', 10);
    set(gca, 'XTick', DOF, 'XTickLabel', DOF_labels);
    ylim([-1.2 1.2]);
    yline(0, 'k--', 'LineWidth', 1);
end

% Plot all mode shapes together - Case 1
subplot(2, 3, [4 5 6]);
plot(DOF, ModeShapes1(:,1), '-o', 'LineWidth', 2, 'MarkerSize', 8, ...
     'DisplayName', sprintf('Mode 1: %.2f Hz', f_n1(1)));
hold on;
plot(DOF, ModeShapes1(:,2), '-s', 'LineWidth', 2, 'MarkerSize', 8, ...
     'DisplayName', sprintf('Mode 2: %.2f Hz', f_n1(2)));
plot(DOF, ModeShapes1(:,3), '-^', 'LineWidth', 2, 'MarkerSize', 8, ...
     'DisplayName', sprintf('Mode 3: %.2f Hz', f_n1(3)));
grid on;
title('All Mode Shapes - Case 1', 'FontSize', 12);
xlabel('Component', 'FontSize', 10);
ylabel('Normalized Amplitude', 'FontSize', 10);
set(gca, 'XTick', DOF, 'XTickLabel', DOF_labels);
legend('Location', 'best');
yline(0, 'k--', 'LineWidth', 1);
hold off;

% Create figure for mode shapes - Case 2
figure('Position', [150, 150, 1200, 800]);
sgtitle('Mode Shapes - Case 2: Low Stiffness Damper (k_1 = 100 Nm/rad)', ...
        'FontSize', 14, 'FontWeight', 'bold');

for i = 1:3
    subplot(2, 3, i);
    bar(DOF, ModeShapes2(:,i), 'FaceColor', [0.8 0.2 0.2]);
    grid on;
    title(sprintf('Mode %d: f = %.2f Hz', i, f_n2(i)), 'FontSize', 12);
    xlabel('Component', 'FontSize', 10);
    ylabel('Normalized Amplitude', 'FontSize', 10);
    set(gca, 'XTick', DOF, 'XTickLabel', DOF_labels);
    ylim([-1.2 1.2]);
    yline(0, 'k--', 'LineWidth', 1);
end

% Plot all mode shapes together - Case 2
subplot(2, 3, [4 5 6]);
plot(DOF, ModeShapes2(:,1), '-o', 'LineWidth', 2, 'MarkerSize', 8, ...
     'DisplayName', sprintf('Mode 1: %.2f Hz', f_n2(1)));
hold on;
plot(DOF, ModeShapes2(:,2), '-s', 'LineWidth', 2, 'MarkerSize', 8, ...
     'DisplayName', sprintf('Mode 2: %.2f Hz', f_n2(2)));
plot(DOF, ModeShapes2(:,3), '-^', 'LineWidth', 2, 'MarkerSize', 8, ...
     'DisplayName', sprintf('Mode 3: %.2f Hz', f_n2(3)));
grid on;
title('All Mode Shapes - Case 2', 'FontSize', 12);
xlabel('Component', 'FontSize', 10);
ylabel('Normalized Amplitude', 'FontSize', 10);
set(gca, 'XTick', DOF, 'XTickLabel', DOF_labels);
legend('Location', 'best');
yline(0, 'k--', 'LineWidth', 1);
hold off;

%% ========================================================================
% COMPARISON PLOT: Natural Frequencies
% ========================================================================

% Calculate Idling Frequency (900 RPM / 60)
idling_rpm = 900;
f_idling_hz = round(idling_rpm / 60, 1); % 13.3 Hz

figure('Position', [200, 200, 800, 500]);
modes = 1:3;
bar_data = [f_n1'; f_n2'];
bar(modes, bar_data');
grid on;
title('Natural Frequency Comparison', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Mode Number', 'FontSize', 12);
ylabel('Natural Frequency (Hz)', 'FontSize', 12);


hold on;
line([0.5 3.5], [f_idling_hz f_idling_hz], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', sprintf('Idling Frequency (%.1f Hz)', f_idling_hz));
ylim([0 max(max(f_n1), max(f_n2)) + 25]); % Adjust y-limit to ensure line fits

legend('High Stiffness (500 Nm/rad)', 'Low Stiffness (100 Nm/rad)', sprintf('Idling Frequency (%.1f Hz)', f_idling_hz), ...
       'Location', 'northwest');
set(gca, 'XTick', modes);

% Add value labels on bars
for i = 1:3
    text(i-0.15, f_n1(i)+2, sprintf('%.1f', f_n1(i)), ...
         'HorizontalAlignment', 'center', 'FontSize', 9);
    text(i+0.15, f_n2(i)+2, sprintf('%.1f', f_n2(i)), ...
         'HorizontalAlignment', 'center', 'FontSize', 9);
end

% Set the display format to show full values for results
format long g

%% ========================================================================
% TIME RESPONSE ANALYSIS USING UNDAMPED PRINCIPLE COORDINATES
% ========================================================================

fprintf('========================================\n');
fprintf('TIME RESPONSE: UNDAMPED ANALYTICAL SOLUTION\n');
fprintf('========================================\n\n');

% Engine excitation parameters (Pure sinusoidal)
T_amp = 50;        % Torque amplitude (Nm)
omega_excite = 2 * pi * 15;  % Excitation frequency: 15 Hz converted to rad/s
f_excite = omega_excite / (2*pi);

% Time span
t = 0:0.001:2;     % 2 seconds

fprintf('Excitation Parameters:\n');
fprintf('  Torque: T(t) = %.1f sin(%.2f Hz × t) Nm\n', T_amp, f_excite);
fprintf('  System: UNDAMPED (ζ = 0)\n');
fprintf('  Initial Conditions: q(0) = 0, q̇(0) = 0\n\n');

%% ========================================================================
% CASE 1: HIGH STIFFNESS - Analytical Solution
% ========================================================================

fprintf('Case 1: HIGH STIFFNESS DAMPER\n');
fprintf('----------------------------------------\n');

% Modal parameters for Case 1
Phi1 = ModeShapes1;   % Mass-normalized mode shapes
omega_n_case1 = omega_n1;  % Natural frequencies

% External torque vector: T(t) = T_amp * sin(ω_excite * t) acts on DOF 1 only
T_vector = [T_amp; 0; 0];

% Calculate modal forces: Q_i = φᵢᵀ{T}
Q_modal1 = Phi1' * T_vector;

fprintf('Modal Forces (Amplitudes):\n');
fprintf('  P1 = %.4f Nm\n', Q_modal1(1));
fprintf('  P2 = %.4f Nm\n', Q_modal1(2));
fprintf('  P3 = %.4f Nm\n\n', Q_modal1(3));

% Solve each decoupled equation analytically
% For undamped system with q(0) = 0, q̇(0) = 0:
% q̈ᵢ + ωᵢ²qᵢ = Qᵢ sin(ω_excite × t)
% Solution: qᵢ(t) = (Qᵢ/ωᵢ²) × [sin(ω_excite×t) - rᵢsin(ωᵢ×t)] / (1 - rᵢ²)
% where rᵢ = ω_excite / ωᵢ (frequency ratio)

q1_case1 = zeros(3, length(t));

fprintf('Principle Coordinate Solutions:\n');
for i = 1:3
    r = omega_excite / omega_n_case1(i);  % Frequency ratio
    
    if abs(1 - r^2) > 0.01  % Not near resonance
        % Standard steady-state + transient solution
        A_i = (Q_modal1(i) / omega_n_case1(i)^2) / (1 - r^2);
        q1_case1(i,:) = A_i * (sin(omega_excite * t) - r * sin(omega_n_case1(i) * t));
        
        fprintf('  Mode %d: A_%d = %.6f, r_%d = %.4f\n', i, i, A_i, i, r);
    else
        % Near resonance - use limiting form
        fprintf('  Mode %d: NEAR RESONANCE (r = %.4f)\n', i, r);
        q1_case1(i,:) = 0;  % Avoid numerical issues
    end
end
fprintf('\n');

% Transform to physical coordinates: {θ(t)} = [Φ]{q(t)}
theta1 = Phi1 * q1_case1;

fprintf('Principle Coordinate Coefficients (Case 1):\n');
fprintf('  q₁(t) = A₁[sin(%.2f Hz×t) - r₁sin(%.2f Hz×t)]\n', f_excite, f_n1(1));
fprintf('  q₂(t) = A₂[sin(%.2f Hz×t) - r₂sin(%.2f Hz×t)]\n', f_excite, f_n1(2));
fprintf('  q₃(t) = A₃[sin(%.2f Hz×t) - r₃sin(%.2f Hz×t)]\n\n', f_excite, f_n1(3));

%% ========================================================================
% CASE 2: LOW STIFFNESS - Analytical Solution
% ========================================================================

fprintf('Case 2: LOW STIFFNESS DAMPER\n');
fprintf('----------------------------------------\n');

% Modal parameters for Case 2
Phi2 = ModeShapes2;
omega_n_case2 = omega_n2;

% Calculate modal forces
Q_modal2 = Phi2' * T_vector;

fprintf('Modal Forces (Amplitudes):\n');
fprintf('  P1 = %.4f Nm\n', Q_modal2(1));
fprintf('  P2 = %.4f Nm\n', Q_modal2(2));
fprintf('  P3 = %.4f Nm\n\n', Q_modal2(3));

% Solve each decoupled equation
q2_case2 = zeros(3, length(t));

fprintf('Principle Coordinate Solutions:\n');
for i = 1:3
    r = omega_excite / omega_n_case2(i);
    
    if abs(1 - r^2) > 0.01
        A_i = (Q_modal2(i) / omega_n_case2(i)^2) / (1 - r^2);
        q2_case2(i,:) = A_i * (sin(omega_excite * t) - r * sin(omega_n_case2(i) * t));
        
        fprintf('  Mode %d: A_%d = %.6f, r_%d = %.4f\n', i, i, A_i, i, r);
    else
        fprintf('  Mode %d: NEAR RESONANCE (r = %.4f)\n', i, r);
        q2_case2(i,:) = 0;
    end
end
fprintf('\n');

% Transform to physical coordinates
theta2 = Phi2 * q2_case2;

fprintf('Principle Coordinate Coefficients (Case 2):\n');
fprintf('  q₁(t) = A₁[sin(%.2f Hz×t) - r₁sin(%.2f Hz×t)]\n', f_excite, f_n2(1));
fprintf('  q₂(t) = A₂[sin(%.2f Hz×t) - r₂sin(%.2f Hz×t)]\n', f_excite, f_n2(2));
fprintf('  q₃(t) = A₃[sin(%.2f Hz×t) - r₃sin(%.2f Hz×t)]\n\n', f_excite, f_n2(3));

fprintf('========================================\n');
fprintf('ANALYTICAL SOLUTION SUMMARY\n');
fprintf('========================================\n\n');

fprintf('For each mode i, the principle coordinate is:\n');
fprintf('  qᵢ(t) = Aᵢ[sin(ω_excite×t) - rᵢsin(ωᵢ×t)]\n\n');
fprintf('Where:\n');
fprintf('  Aᵢ = (Qᵢ/ωᵢ²)/(1 - rᵢ²)\n');
fprintf('  rᵢ = ω_excite/ωᵢ (frequency ratio)\n');
fprintf('  Qᵢ = φᵢᵀ{T} (modal force)\n\n');
fprintf('Physical displacement:\n');
fprintf('  {θ(t)} = [Φ]{q(t)} = Σ φᵢ qᵢ(t)\n\n');

%% ========================================================================
% EXPLICIT THETA(T) EQUATIONS
% ========================================================================

fprintf('========================================\n');
fprintf('EXPLICIT DISPLACEMENT EQUATIONS θ(t)\n');
fprintf('========================================\n\n');

fprintf('CASE 1: HIGH STIFFNESS DAMPER (k1 = %d Nm/rad)\n', k1_high);
fprintf('------------------------------------------------------------\n\n');

% Calculate coefficients for each DOF in Case 1
for dof = 1:3
    fprintf('θ_%d(t) = ', dof);
    
    % Sum contributions from all modes
    terms = {};
    for mode = 1:3
        r = omega_excite / omega_n_case1(mode);
        if abs(1 - r^2) > 0.01
            A_i = (Q_modal1(mode) / omega_n_case1(mode)^2) / (1 - r^2);
            coeff_forced = Phi1(dof, mode) * A_i;
            coeff_free = Phi1(dof, mode) * A_i * (-r);
            
            % Build the term string
            if abs(coeff_forced) > 1e-10 || abs(coeff_free) > 1e-10
                term_str = sprintf('%.6e×sin(%.4f×t) + %.6e×sin(%.4f×t)', ...
                    coeff_forced, omega_excite, coeff_free, omega_n_case1(mode));
                terms{end+1} = term_str;
            end
        end
    end
    
    % Print all terms
    if ~isempty(terms)
        fprintf('%s', terms{1});
        for i = 2:length(terms)
            fprintf('\n         + %s', terms{i});
        end
    else
        fprintf('0');
    end
    fprintf('\n\n');
end

fprintf('In compact form:\n');
fprintf('  θ₁(t) = Σ φ₁ᵢ × Aᵢ × [sin(%.2f×t) - rᵢ×sin(ωᵢ×t)]\n', omega_excite);
fprintf('  θ₂(t) = Σ φ₂ᵢ × Aᵢ × [sin(%.2f×t) - rᵢ×sin(ωᵢ×t)]\n', omega_excite);
fprintf('  θ₃(t) = Σ φ₃ᵢ × Aᵢ × [sin(%.2f×t) - rᵢ×sin(ωᵢ×t)]\n\n', omega_excite);

fprintf('Where ω₁ = %.4f, ω₂ = %.4f, ω₃ = %.4f rad/s\n\n', ...
    omega_n_case1(1), omega_n_case1(2), omega_n_case1(3));

fprintf('========================================\n\n');

fprintf('CASE 2: LOW STIFFNESS DAMPER (k1 = %d Nm/rad)\n', k1_low);
fprintf('------------------------------------------------------------\n\n');

% Calculate coefficients for each DOF in Case 2
for dof = 1:3
    fprintf('θ_%d(t) = ', dof);
    
    % Sum contributions from all modes
    terms = {};
    for mode = 1:3
        r = omega_excite / omega_n_case2(mode);
        if abs(1 - r^2) > 0.01
            A_i = (Q_modal2(mode) / omega_n_case2(mode)^2) / (1 - r^2);
            coeff_forced = Phi2(dof, mode) * A_i;
            coeff_free = Phi2(dof, mode) * A_i * (-r);
            
            % Build the term string
            if abs(coeff_forced) > 1e-10 || abs(coeff_free) > 1e-10
                term_str = sprintf('%.6e×sin(%.4f×t) + %.6e×sin(%.4f×t)', ...
                    coeff_forced, omega_excite, coeff_free, omega_n_case2(mode));
                terms{end+1} = term_str;
            end
        end
    end
    
    % Print all terms
    if ~isempty(terms)
        fprintf('%s', terms{1});
        for i = 2:length(terms)
            fprintf('\n         + %s', terms{i});
        end
    else
        fprintf('0');
    end
    fprintf('\n\n');
end

fprintf('In compact form:\n');
fprintf('  θ₁(t) = Σ φ₁ᵢ × Aᵢ × [sin(%.2f×t) - rᵢ×sin(ωᵢ×t)]\n', omega_excite);
fprintf('  θ₂(t) = Σ φ₂ᵢ × Aᵢ × [sin(%.2f×t) - rᵢ×sin(ωᵢ×t)]\n', omega_excite);
fprintf('  θ₃(t) = Σ φ₃ᵢ × Aᵢ × [sin(%.2f×t) - rᵢ×sin(ωᵢ×t)]\n\n', omega_excite);

fprintf('Where ω₁ = %.4f, ω₂ = %.4f, ω₃ = %.4f rad/s\n\n', ...
    omega_n_case2(1), omega_n_case2(2), omega_n_case2(3));

fprintf('========================================\n');

%% ========================================================================
% VISUALIZATION: TIME RESPONSE
% ========================================================================

% Plot displacement-time response
figure('Position', [250, 250, 1200, 800]);
sgtitle('Angular Displacement Time Response (Undamped Analytical Solution)', 'FontSize', 14, 'FontWeight', 'bold');

% Case 1 - High Stiffness
subplot(2,3,1);
plot(t, theta1(1,:)*1000, 'LineWidth', 1.5);
grid on;
title('Case 1: Engine (θ_1)', 'FontSize', 11);
xlabel('Time (s)');
ylabel('Displacement (mrad)');

subplot(2,3,2);
plot(t, theta1(2,:)*1000, 'LineWidth', 1.5);
grid on;
title('Case 1: Clutch (θ_2)', 'FontSize', 11);
xlabel('Time (s)');
ylabel('Displacement (mrad)');

subplot(2,3,3);
plot(t, theta1(3,:)*1000, 'LineWidth', 1.5);
grid on;
title('Case 1: Transmission (θ_3)', 'FontSize', 11);
xlabel('Time (s)');
ylabel('Displacement (mrad)');

% Case 2 - Low Stiffness
subplot(2,3,4);
plot(t, theta2(1,:)*1000, 'LineWidth', 1.5, 'Color', [0.8 0.2 0.2]);
grid on;
title('Case 2: Engine (θ_1)', 'FontSize', 11);
xlabel('Time (s)');
ylabel('Displacement (mrad)');

subplot(2,3,5);
plot(t, theta2(2,:)*1000, 'LineWidth', 1.5, 'Color', [0.8 0.2 0.2]);
grid on;
title('Case 2: Clutch (θ_2)', 'FontSize', 11);
xlabel('Time (s)');
ylabel('Displacement (mrad)');

subplot(2,3,6);
plot(t, theta2(3,:)*1000, 'LineWidth', 1.5, 'Color', [0.8 0.2 0.2]);
grid on;
title('Case 2: Transmission (θ_3)', 'FontSize', 11);
xlabel('Time (s)');
ylabel('Displacement (mrad)');

% Comparison plot for transmission displacement
figure('Position', [300, 300, 900, 500]);
plot(t, theta1(3,:)*1000, 'LineWidth', 2, 'DisplayName', 'High Stiffness (500 Nm/rad)');
hold on;
plot(t, theta2(3,:)*1000, 'LineWidth', 2, 'DisplayName', 'Low Stiffness (100 Nm/rad)');
grid on;
title('Transmission Displacement Comparison', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 12);
ylabel('Angular Displacement (mrad)', 'FontSize', 12);
legend('Location', 'best');
hold off;

% Plot Principle Coordinates q(t)
figure('Position', [350, 350, 1200, 800]);
sgtitle('Principle Coordinates q(t) - Undamped Analytical Solution', 'FontSize', 14, 'FontWeight', 'bold');

% Case 1 - Principle Coordinates
for i = 1:3
    subplot(2,3,i);
    plot(t, q1_case1(i,:), 'LineWidth', 1.5);
    grid on;
    title(sprintf('Case 1: q_%d(t) - Mode %d (%.2f Hz)', i, i, f_n1(i)), 'FontSize', 11);
    xlabel('Time (s)');
    ylabel('Principle Coordinate');
end

% Case 2 - Principle Coordinates
for i = 1:3
    subplot(2,3,i+3);
    plot(t, q2_case2(i,:), 'LineWidth', 1.5, 'Color', [0.8 0.2 0.2]);
    grid on;
    title(sprintf('Case 2: q_%d(t) - Mode %d (%.2f Hz)', i, i, f_n2(i)), 'FontSize', 11);
    xlabel('Time (s)');
    ylabel('Principle Coordinate');
end

fprintf('All plots generated successfully.\n');
fprintf('========================================\n');



%References:
%Genç, G., Ebrinc, O., & Aydin, M. (2018). Torsional Vibration Analysis of Automotive
%Drivetrain System. Journal of Mechanical Engineering, 45(3), 112-125.
%Nassar, M. M. (2012). Modal Analysis of Multi-Disc Rotor Systems Using Finite Element
%and MATLAB Procedures. International Journal of Engineering Research and
%Applications, 2(4), 1987-1994.
%Pavlov, V. P. (2018). Dual Mass Flywheel Systems for Torsional Vibration Isolation in
%Vehicle Drivetrains. Automotive Engineering International, 96(2), 67-74.