%% STRAIGHT HIGHWAY – ANALYTIC WAYPOINTS FOR ALL LANE WIDTHS
% AVa: 60 mph (26.82 m/s) constant
% AVb: 70 mph (31.29 m/s) constant
% Lane widths: 4.0, 3.75, 3.5, 3.25, 3.0 m
%
% For each lane width w:
%   - AVa stays in the upper lane: y = w
%   - AVb performs an overtake from the lower lane: y = 0 -> y = w
%
% Result:
%   waypoints_AVa_straight{k} : [x y] for AVa, scenario k
%   waypoints_AVb_straight{k} : [x y] for AVb, scenario k

% clear; clc; close all;

%% Global parameters
laneWidths = [4.0 3.75 3.5 3.25 3.0];   % 5 scenarios
vA_const   = 26.82;                     % 60 mph (m/s) – info only
vB_const   = 31.29;                     % 70 mph (m/s)

nW = numel(laneWidths);

% containers for Simulink
waypoints_AVa_straight = cell(nW,1);   % each cell: [x y]
waypoints_AVb_straight = cell(nW,1);

%% Common longitudinal grid (m)
L   = 1000;                     % total length used for waypoints
N   = 500;                     % number of points
x_wp = linspace(0, L, N)';     % 0 → L

% lane-change region (like in your example: 80–160 m)
x_ch_start = 0.20 * L;         % 80 m if L = 400
x_ch_end   = 0.40 * L;         % 160 m if L = 400

%% Loop over the 5 lane widths
for k = 1:nW
    w = laneWidths(k);

    % ----- AVa: upper lane, y = w -----
    yA_wp = w * ones(size(x_wp));
    waypoints_AVa_straight{k} = [x_wp yA_wp];

    % ----- AVb: overtake 0 -> w -----
    yB_wp = zeros(size(x_wp));                         % starts in lower lane

    % linear ramp part between x_ch_start and x_ch_end
    idx = (x_wp >= x_ch_start) & (x_wp <= x_ch_end);
    yB_wp(idx) = w * (x_wp(idx) - x_ch_start) / (x_ch_end - x_ch_start);

    % after that AVb stays in the upper lane
    idx2 = x_wp > x_ch_end;
    yB_wp(idx2) = w;

    waypoints_AVb_straight{k} = [x_wp yB_wp];
end

%% (Optional) small figure to check the 5 scenarios
% Figure with 4 subplots (same style as the paper)

colors = lines(nW);

fig = figure('Name','Straight highway REFERENCE – all lane widths','NumberTitle','off');

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

% --- Subplot (2,1): AVa lateral vs longitudinal ---
ax3 = subplot(2,2,3); hold(ax3,'on'); grid(ax3,'on');
xlabel(ax3,'AV_a Longitudinal Displacement [m]');
ylabel(ax3,'AV_a Lateral Displacement [m]');
title(ax3,'AV_a Lateral vs Longitudinal');
xlim(ax3,[0 max(waypoints_AVa_straight{1}(:,1))]);
ylim(ax3,[0 5]);

% --- Subplot (2,2): AVb lateral vs longitudinal ---
ax4 = subplot(2,2,4); hold(ax4,'on'); grid(ax4,'on');
xlabel(ax4,'AV_b Longitudinal Displacement [m]');
ylabel(ax4,'AV_b Lateral Displacement [m]');
title(ax4,'AV_b Lateral vs Longitudinal');
xlim(ax4,[0 max(waypoints_AVb_straight{1}(:,1))]);
ylim(ax4,[0 5]);

% ----- Loop over lane widths and plot -----
for k = 1:nW
    c = colors(k,:);

    % extract waypoints
    xA = waypoints_AVa_straight{k}(:,1);
    yA = waypoints_AVa_straight{k}(:,2);
    xB = waypoints_AVb_straight{k}(:,1);
    yB = waypoints_AVb_straight{k}(:,2);

    % build time vectors assuming constant speeds
    tA = xA / vA_const;   % x = v * t  -> t = x / v
    tB = xB / vB_const;

    vA = vA_const * ones(size(tA));
    vB = vB_const * ones(size(tB));

    % velocities
    plot(ax1, tA, vA, 'Color', c, 'LineWidth', 1.2);
    plot(ax2, tB, vB, 'Color', c, 'LineWidth', 1.2);

    % lateral vs longitudinal
    plot(ax3, xA, yA, 'Color', c, 'LineWidth', 1.2);
    plot(ax4, xB, yB, 'Color', c, 'LineWidth', 1.2);
end

% Legend on bottom-right subplot (as in the paper)
legend(ax4, arrayfun(@(w) sprintf('w = %.2f m', w), laneWidths, ...
       'UniformOutput', false), 'Location','best');


%% NOTE FOR SIMULINK
% - Scenario k corresponds to laneWidths(k).
%   Example: k = 1 -> w = 4.0 m ; k = 3 -> w = 3.50 m
%
% - In a Constant block:
%       waypoints_AVa_straight{k}   or   waypoints_AVb_straight{k}
%   depending on the vehicle.
