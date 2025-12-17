%% CURVED HIGHWAY – ANALYTIC WAYPOINTS FOR ALL LANE WIDTHS
% AVa: 60 mph (26.82 m/s) constant
% AVb: 70 mph (31.29 m/s) constant
% Lane widths: 4.0, 3.75, 3.5, 3.25, 3.0 m
%
% For each lane width w:
%   - AVa stays in the upper lane along a *curved* road
%   - AVb starts in the lower lane on the same curved road,
%     then performs a smooth lane change to the upper lane
%
% Result (for Simulink):
%   waypoints_AVa_curved{k} : [x y] for AVa, scenario k
%   waypoints_AVb_curved{k} : [x y] for AVb, scenario k

% clear; clc; close all;

%% Global parameters
laneWidths = [4.0 3.75 3.5 3.25 3.0];   % 5 scenarios
vA_const   = 26.82;                     % 60 mph (m/s)
vB_const   = 31.29;                     % 70 mph (m/s)

nW = numel(laneWidths);

% containers for Simulink
waypoints_AVa_curved = cell(nW,1);   % each cell: [x y]
waypoints_AVb_curved = cell(nW,1);

%% Longitudinal grid (m)
L   = 1000;                     % total length used for waypoints
N   = 500;                      % number of points
x_wp = linspace(0, L, N)';      % 0 → L

%% Road curvature function (same for all lane widths)
% Parabolic "valley" shape: road goes inward, then outward again.
% c(x) is the lateral position of the *lower* lane centre when w = 0.
%
% c(x) = A_curve * ( (x/L - 0.5)^2 - 0.25 )
%  -> c(0)   = 0
%  -> c(L/2) = -A_curve*0.25  (minimum)
%  -> c(L)   = 0
A_curve = 16;    % [m] amplitude of the curved road – tune to match paper
xi = x_wp / L - 0.5;
c = A_curve * (xi.^2 - 0.25);   % size N x 1

%% Lane-change region for AVb (along the curved road)
% These numbers are chosen to visually match the paper
x_ch_start = 0.40 * L;          % ~400 m
x_ch_end   = 0.50 * L;          % ~500 m

alpha = (x_wp - x_ch_start) / (x_ch_end - x_ch_start); % normalized 0→1
alpha = max(0,min(1,alpha));                           % clip
% Smooth step: 0 → 1 with zero slope at both ends
s = 3*alpha.^2 - 2*alpha.^3;   % same size as x_wp

%% Loop over the 5 lane widths
for k = 1:nW
    w = laneWidths(k);

    % ----- Lane centres on curved road -----
    % Lower lane centre
    y_low = c;           % starts at 0, dips negative, ends at 0
    % Upper lane centre is offset by lane width w
    y_up  = c + w;       % always w above the lower lane

    % ----- AVa: stays in the upper lane -----
    yA_wp = y_up;        % full curved profile in upper lane
    waypoints_AVa_curved{k} = [x_wp yA_wp];

    % ----- AVb: curved + lane change lower -> upper -----
    % Before lane change: follow lower lane (s = 0)
    % After  lane change: follow upper lane (s = 1)
    % In between: smooth blend
    yB_wp = (1 - s).*y_low + s.*y_up;
    waypoints_AVb_curved{k} = [x_wp yB_wp];
end

%% (Optional) figure – same style as paper (without C(t) here)
colors = lines(nW);

fig = figure('Name','Curved highway REFERENCE – all lane widths','NumberTitle','off');

% --- Subplot (1,1): AVa velocity vs time ---
ax1 = subplot(2,2,1); hold(ax1,'on'); grid(ax1,'on');
xlabel(ax1,'Time [s]');
ylabel(ax1,'AV_a Velocity [m/s]');
title(ax1,'AV_a Velocity vs Time');
xlim(ax1,[0 30]);

% --- Subplot (1,2): AVb velocity vs time ---
ax2 = subplot(2,2,2); hold(ax2,'on'); grid(ax2,'on');
xlabel(ax2,'Time [s]');
ylabel(ax2,'AV_b Velocity [m/s]');
title(ax2,'AV_b Velocity vs Time');
xlim(ax2,[0 30]);

% --- Subplot (2,1): AVa lateral vs longitudinal (curved) ---
ax3 = subplot(2,2,3); hold(ax3,'on'); grid(ax3,'on');
xlabel(ax3,'AV_a Longitudinal Displacement [m]');
ylabel(ax3,'AV_a Lateral Displacement [m]');
title(ax3,'AV_a Lateral vs Longitudinal (curved)');
xlim(ax3,[0 L]);
ylim(ax3,[-5 7]);   % adjust to what you see

% --- Subplot (2,2): AVb lateral vs longitudinal (curved) ---
ax4 = subplot(2,2,4); hold(ax4,'on'); grid(ax4,'on');
xlabel(ax4,'AV_b Longitudinal Displacement [m]');
ylabel(ax4,'AV_b Lateral Displacement [m]');
title(ax4,'AV_b Lateral vs Longitudinal (curved)');
xlim(ax4,[0 L]);
ylim(ax4,[-5 7]);   % adjust to what you see

% ----- Loop over lane widths and plot -----
for k = 1:nW
    ccol = colors(k,:);

    % extract waypoints
    xA = waypoints_AVa_curved{k}(:,1);
    yA = waypoints_AVa_curved{k}(:,2);
    xB = waypoints_AVb_curved{k}(:,1);
    yB = waypoints_AVb_curved{k}(:,2);

    % build time vectors assuming constant speeds
    tA = xA / vA_const;   % x = v * t  -> t = x / v
    tB = xB / vB_const;

    vA = vA_const * ones(size(tA));
    vB = vB_const * ones(size(tB));

    % velocities
    plot(ax1, tA, vA, 'Color', ccol, 'LineWidth', 1.2);
    plot(ax2, tB, vB, 'Color', ccol, 'LineWidth', 1.2);

    % lateral vs longitudinal
    plot(ax3, xA, yA, 'Color', ccol, 'LineWidth', 1.2);
    plot(ax4, xB, yB, 'Color', ccol, 'LineWidth', 1.2);
end

% Legend on bottom-right subplot (as in the paper)
legend(ax4, arrayfun(@(w) sprintf('w = %.2f m', w), laneWidths, ...
       'UniformOutput', false), 'Location','best');

%% NOTE FOR SIMULINK
% - Scenario k corresponds to laneWidths(k).
%   Example: k = 1 -> w = 4.0 m ; k = 3 -> w = 3.50 m
%
% - In a Constant block:
%       waypoints_AVa_curved{k}   or   waypoints_AVb_curved{k}
%   depending on the vehicle.
