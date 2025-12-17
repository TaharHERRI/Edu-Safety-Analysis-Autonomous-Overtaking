%% CAS 781 — One-click project runner (Simulink + SDI plots)
% This single script is meant to reproduce all major results results
% by running everything from one file.
%
% It will:
%   1) Load parameters / MPC / VB table (Plant_MPC_design.m)
%   2) Generate analytic REFERENCE waypoints (straight + curved)
%   3) Run 5 lane-width simulations (straight) and plot overlay
%   4) Run 5 lane-width simulations (curved) and plot overlay
%   5) Run a 2-run comparison at w = 4 m (fault vs no-fault) and plot
%
% Notes:
%   - Requires the current folder to be the project folder containing:
%       Adaptive_MPC_design.slx, Plant_MPC_design.m,
%       trajectories_waypoints_straight.m, trajectories_waypoints_curved.m
%   - Uses Simulation Data Inspector (SDI) to extract and plot signals.

%% 0) Setup
clear; close all; clc;

%--- Robust project root detection (works even if script is run from Temp) ---
thisFile = mfilename('fullpath');
if isempty(thisFile)
    st = dbstack('-completenames');   % fallback
    thisFile = st(1).file;
end
projRoot = fileparts(thisFile);

% Look upward until we find the Simulink model (project root marker)
targetModel = "Adaptive_MPC_design.slx";
maxUp = 6;
for k = 1:maxUp
    if isfile(fullfile(projRoot, targetModel))
        break;
    end
    parent = fileparts(projRoot);
    if strcmp(parent, projRoot)
        break;
    end
    projRoot = parent;
end

% Last resort: if still not found, assume current folder
if ~isfile(fullfile(projRoot, targetModel))
    warning("Project root not found via file location. Falling back to pwd().");
    projRoot = pwd;
end

cd(projRoot);
addpath(genpath(projRoot));
fprintf("Project root: %s\n", projRoot);

run(fullfile(projRoot, "clean_project.m"));


mdl = "Adaptive_MPC_design";
load_system(mdl);

% Optional: clear SDI to avoid mixing with older runs
Simulink.sdi.clear;

% Plot style
cfgPlot = struct();
cfgPlot.caseSensitive = false;
cfgPlot.componentIdx  = [1 1];    % plot (1,1) when a signal is a vector/matrix
cfgPlot.lineWidth     = 1.8;

cfgPlot.patterns = struct();
cfgPlot.patterns.v_a = ["V_a", "v_a", "AVa speed", "speed_a", "Va"];
cfgPlot.patterns.v_b = ["V_b", "v_b", "AVb speed", "speed_b", "Vb"];
cfgPlot.patterns.y_a = ["y_a", "AVa) Lateral position", "poseA", "yA"];
cfgPlot.patterns.y_b = ["y_b", "AVb) Lateral position", "poseB", "yB"];
cfgPlot.patterns.C   = ["C", "collision metric", "relative collision", "C_raw"];

stopTime = "300"; % seconds (adjust if your model uses another stop time)

%% 1) Generate analytic waypoints (creates: laneWidths, waypoints_AVa_*, waypoints_AVb_*)
run(fullfile(projRoot, "trajectories_waypoints_straight.m"));
run(fullfile(projRoot, "trajectories_waypoints_curved.m"));

% Keep a local copy of laneWidths (defined by the waypoint scripts)
laneWidthsList = laneWidths(:)';

%% 2) Load controller / vehicle parameters (creates: VB_table, mpcobj, etc.)
run(fullfile(projRoot, "Plant_MPC_design.m"));

%% 3) Run 5 lane-widths (STRAIGHT) — no fault
fprintf('\n=== Running STRAIGHT scenarios (no fault) ===\n');
for k = 1:numel(laneWidthsList)
    w = laneWidthsList(k);

    % Select waypoint set
    idx = findClosestLaneWidth(w, laneWidthsList);
    waypoints_AVa = waypoints_AVa_straight{idx}; %#ok<NASGU>
    waypoints_AVb = waypoints_AVb_straight{idx}; %#ok<NASGU>

    % Required workspace variables (highlighted in your model)
    y0_AVa = w;                  %lane width / initial y for AVa
    fault_injection = uint8(0); %#ok<NASGU>

    % MPC state init (if your MPC block uses x0 from workspace)
    x0 = mpcstate(mpcobj);      
    x0.Plant = [0; 0; 0; y0_AVa];
    x0.Disturbance = zeros(size(x0.Disturbance));
    x0.Noise       = zeros(size(x0.Noise));

    set_param(mdl, "StopTime", stopTime);

    sim(mdl); % produces an SDI run if logging is enabled
end

% Plot last 5 runs (straight)
labelsStraight = arrayfun(@(w) sprintf("Lane Width %g m", w), laneWidthsList, "UniformOutput", false);
plotLastNRunsFromSDI(5, labelsStraight, cfgPlot, "Straight (5 lane widths)");

%% 4) Run 5 lane-widths (CURVED) — no fault
fprintf('\n=== Running CURVED scenarios (no fault) ===\n');
for k = 1:numel(laneWidthsList)
    w = laneWidthsList(k);

    idx = findClosestLaneWidth(w, laneWidthsList);
    waypoints_AVa = waypoints_AVa_curved{idx}; %#ok<NASGU>
    waypoints_AVb = waypoints_AVb_curved{idx}; %#ok<NASGU>

    y0_AVa = w;                 
    fault_injection = uint8(0); %#ok<NASGU>

    x0 = mpcstate(mpcobj);      
    x0.Plant = [0; 0; 0; y0_AVa];
    x0.Disturbance = zeros(size(x0.Disturbance));
    x0.Noise       = zeros(size(x0.Noise));

    set_param(mdl, "StopTime", stopTime);

    sim(mdl);
end

labelsCurved = labelsStraight; % same mapping order
plotLastNRunsFromSDI(5, labelsCurved, cfgPlot, "Curved (5 lane widths)");

%% 5) Fault-injection demo (2 runs) at w = 4 m (order requested: Fault then No Fault)
% If you changed which UCA you implement, keep the same switch variable name
% (fault_injection) and set it accordingly in your model.
fprintf('\n=== Running FAULT demo at w = 4 m (Fault then No Fault) ===\n');

w = 4.0;
idx = findClosestLaneWidth(w, laneWidthsList);
waypoints_AVa = waypoints_AVa_curved{idx};
waypoints_AVb = waypoints_AVb_curved{idx};
y0_AVa = w; 

% --- Run 1: With fault injection
fault_injection = uint8(1); %#ok<NASGU>
x0 = mpcstate(mpcobj);      
x0.Plant = [0; 0; 0; y0_AVa];
x0.Disturbance = zeros(size(x0.Disturbance));
x0.Noise       = zeros(size(x0.Noise));
set_param(mdl, "StopTime", stopTime);
sim(mdl);

% --- Run 2: Without fault injection
fault_injection = uint8(0); 
x0 = mpcstate(mpcobj);      
x0.Plant = [0; 0; 0; y0_AVa];
x0.Disturbance = zeros(size(x0.Disturbance));
x0.Noise       = zeros(size(x0.Noise));
set_param(mdl, "StopTime", stopTime);
sim(mdl);

labelsFault = {"With Fault Injection", "Without Fault Injection"};
plotLastNRunsFromSDI(2, labelsFault, cfgPlot, "Fault demo (w = 4 m)");

%% ===================== LOCAL FUNCTIONS =====================

function k = findClosestLaneWidth(lane_width, laneWidths)
% Robust index of the closest lane width (handles floating-point mismatch)
    [~, k] = min(abs(laneWidths - lane_width));
end

function plotLastNRunsFromSDI(nRuns, runLabels, cfg, figTitle)
% Export last N SDI runs -> extract y_a, y_b, v_a, v_b, C -> plot in fixed layout.
    runIDs = Simulink.sdi.getAllRunIDs;
    if isempty(runIDs)
        error("SDI contains no runs. Run the simulation with SDI logging enabled first.");
    end

    n = min(nRuns, numel(runIDs));
    selRunIDs = runIDs(end-n+1:end);  % chronological order within the N last runs
    nSel = numel(selRunIDs);

    % Ensure labels length
    if numel(runLabels) ~= nSel
        % If user gave 5 labels but only 3 runs exist, truncate.
        runLabels = runLabels(1:min(numel(runLabels), nSel));
        if numel(runLabels) < nSel
            extra = arrayfun(@(k) sprintf("Run %d", k), (numel(runLabels)+1):nSel, "UniformOutput", false);
            runLabels = [runLabels(:); extra(:)];
        else
            runLabels = runLabels(:);
        end
    end
    runLabels = cellstr(runLabels);

    % Extract signals from each run
    out = struct();
    out.runs = repmat(struct("id",[],"signals",struct()), 1, nSel);

    for i = 1:nSel
        rid = selRunIDs(i);
        r   = Simulink.sdi.getRun(rid);

        % Most recent releases: r.export returns a Dataset directly
        ds = r.export;

        out.runs(i).id = rid;
        out.runs(i).signals.v_a = localFindSignal(ds, cfg.patterns.v_a, cfg);
        out.runs(i).signals.v_b = localFindSignal(ds, cfg.patterns.v_b, cfg);
        out.runs(i).signals.y_a = localFindSignal(ds, cfg.patterns.y_a, cfg);
        out.runs(i).signals.y_b = localFindSignal(ds, cfg.patterns.y_b, cfg);
        out.runs(i).signals.C   = localFindSignal(ds, cfg.patterns.C,   cfg);
    end

    % --- Plot layout (3x2) with legend tile
    fig = figure("Name", "SDI Auto Plot", "Color", "w");
    t = tiledlayout(fig, 3, 2, "TileSpacing", "compact", "Padding", "compact");
    title(t, figTitle, "Interpreter","none");

    axYA = nexttile(t, 1); localPlot(axYA, out, "y_a", runLabels, "AVa lateral position", "y_a", cfg.lineWidth);
    axYB = nexttile(t, 2); localPlot(axYB, out, "y_b", runLabels, "AVb lateral position", "y_b", cfg.lineWidth);
    axVA = nexttile(t, 3); localPlot(axVA, out, "v_a", runLabels, "Va", "v_a", cfg.lineWidth);
    axVB = nexttile(t, 4); localPlot(axVB, out, "v_b", runLabels, "Vb", "v_b", cfg.lineWidth);
    axC  = nexttile(t, 5); localPlot(axC,  out, "C",   runLabels, "C",  "C",   cfg.lineWidth);

    xlabel(axC, "Time (s)");

    % Legend tile (bottom-right)
    axLeg = nexttile(t, 6);
    axis(axLeg, "off"); hold(axLeg, "on");

    h = gobjects(1, nSel);
    for i = 1:nSel
        h(i) = plot(axLeg, nan, nan, "DisplayName", runLabels{i}, "LineWidth", cfg.lineWidth);
    end
    lgd = legend(axLeg, h, runLabels, "Location", "best");
    lgd.Box = "on";
end

function sig = localFindSignal(ds, patterns, cfg)
% Find first matching signal in a Dataset by name patterns.
    sig = struct("name","", "ts",[], "found",false);

    if ~isa(ds, "Simulink.SimulationData.Dataset")
        error("Expected a Simulink.SimulationData.Dataset from SDI export.");
    end

    n = ds.numElements;
    if n == 0, return; end

    names = strings(1,n);
    for i = 1:n
        names(i) = string(ds.getElement(i).Name);
    end

    idx = [];
    for p = 1:numel(patterns)
        pat = string(patterns(p));
        if strlength(pat) == 0, continue; end
        if cfg.caseSensitive
            m = find(contains(names, pat), 1, "first");
        else
            m = find(contains(lower(names), lower(pat)), 1, "first");
        end
        if ~isempty(m), idx = m; break; end
    end

    if isempty(idx)
        warning("Signal not found. Patterns tried: %s", strjoin(string(patterns), ", "));
        return;
    end

    elem = ds.getElement(idx);
    sig.name = string(elem.Name);

    v = elem.Values;
    if isa(v, "timeseries")
        ts = v;
    elseif isa(v, "Simulink.Timeseries")
        ts = v.TimeSeries;
    elseif isstruct(v) && isfield(v,"Time") && isfield(v,"Data")
        ts = timeseries(v.Data, v.Time);
        ts.Name = char(sig.name);
    else
        ts = v; % hope it's already timeseries-like
    end

    ts = localSelectComponent(ts, cfg.componentIdx);

    sig.ts = ts;
    sig.found = true;
end

function tsOut = localSelectComponent(tsIn, componentIdx)
% Ensure we always output a scalar timeseries (Nx1) for plotting.
    if isempty(componentIdx)
        tsOut = tsIn;
        return;
    end

    t = tsIn.Time(:);
    d = tsIn.Data;
    N = numel(t);

    sz = size(d);
    timeDim = find(sz == N, 1, "first");
    if isempty(timeDim)
        error("Impossible de trouver la dimension temporelle dans Data.");
    end

    if timeDim ~= 1
        perm = [timeDim, 1:timeDim-1, timeDim+1:ndims(d)];
        d = permute(d, perm);
    end

    idx = repmat({1}, 1, ndims(d));
    idx{1} = 1:N;
    for k = 1:min(numel(componentIdx), ndims(d)-1)
        idx{k+1} = componentIdx(k);
    end

    dsel = squeeze(d(idx{:}));
    dsel = dsel(:);

    tsOut = timeseries(dsel, t);
    tsOut.Name = tsIn.Name;
end

function localPlot(ax, out, fieldName, runLabels, titleTxt, yLabelTxt, lw)
% Plot a signal across runs (no legend here; legend is in tile 6)
    hold(ax, "on"); grid(ax, "on");

    for k = 1:numel(out.runs)
        sig = out.runs(k).signals.(fieldName);
        if ~isfield(sig, "found") || ~sig.found || isempty(sig.ts), continue; end
        plot(ax, sig.ts.Time, sig.ts.Data, "DisplayName", runLabels{k}, "LineWidth", lw);
    end

    title(ax, titleTxt, "Interpreter","none");
    ylabel(ax, yLabelTxt, "Interpreter","none");
end
