%%
% Vehicle parameters
m = 1575;
Iz = 2875;
lf = 1.2;
lr = 1.6;
Cf = 19000;
Cr = 33000;

Vx = 35;   % nominal speed for design

% --------- Continuous model ---------
A = [-(2*Cf+2*Cr)/m/Vx, 0, -Vx-(2*Cf*lf-2*Cr*lr)/m/Vx, 0;
     0, 0, 1, 0;
     -(2*Cf*lf-2*Cr*lr)/Iz/Vx, 0, -(2*Cf*lf^2+2*Cr*lr^2)/Iz/Vx, 0;
     1, Vx, 0, 0];
B = [2*Cf/m 0 2*Cf*lf/Iz 0]';   
C = [0 0 0 1; 0 1 0 0];
D = zeros(2,1);

%% Discretization and MPC creation
Ts = 0.1;

plant   = ss(A,B,C,D);
plant_d = c2d(plant,Ts);

mpcobj = mpc(plant_d, Ts);

% --- Horizons used in the paper/lab (reasonable values for lateral control) ---
mpcobj.PredictionHorizon = 20;     % Prediction window (number of steps)
mpcobj.ControlHorizon    = 5;      % Control move window

% --- Paper constraints: steering angle = ±30°, steering rate = ±15°/s ---
deltaMax     = deg2rad(30);        % Maximum steering angle (radians)
deltaRateMax = deg2rad(15)*Ts;     % Max steering rate per sample (rad/step)
                                    % Example: 15°/s * 0.1 s = 1.5° per step

mpcobj.MV.Min     = -deltaMax;
mpcobj.MV.Max     =  deltaMax;
mpcobj.MV.RateMin = -deltaRateMax;
mpcobj.MV.RateMax =  deltaRateMax;

% --- Weights: prioritize lateral position, very low weight on yaw angle ---
mpcobj.Weights.OutputVariables          = [5 0.1];   % [weight_y, weight_psi]
mpcobj.Weights.ManipulatedVariables     = 0;         % no penalty on steering magnitude
mpcobj.Weights.ManipulatedVariablesRate = 0.01;      % small penalty on steering rate


%% VB table

% speeds in m/s (10, 30, 50, 70 mph)
VB_table = [ ...
    4.4704   1.14   2.29   4.58;   % 10 mph
   13.4112   5.72  11.43  22.86;   % 30 mph
   22.3520  13.34  26.67  53.34;   % 50 mph
   31.2928  24.00  48.01  96.01];  % 70 mph
% columns: [v_ref  LL   ML    UL]

%%
function k = findClosestLaneWidth(lane_width)
% lane_width: scalaire

laneWidths = [4.0 3.75 3.5 3.25 3.0];

% index du plus proche (robuste aux flottants)
[~, k] = min(abs(laneWidths - lane_width));
end

%%
% 1) Create an empty controller state associated with your mpcobj
x0 = mpcstate(mpcobj);

% 2) Define the state of the internal model (the 4 states of the lateral model)

y0_AVa = 3; %Lane width used in the simulation
x0.Plant = [0; 0; 0; y0_AVa];

% (optional but clean: reset the rest to zero)
x0.Disturbance = zeros(size(x0.Disturbance)); 
x0.Noise       = zeros(size(x0.Noise));

% idx = findClosestLaneWidth(y0_AVa);
% waypoints_AVa = waypoints_AVa_curved{idx};
% waypoints_AVb = waypoints_AVb_curved{idx};
waypoints_AVa = waypoints_AVa_straight{idx};
waypoints_AVb = waypoints_AVb_straight{idx};

fault_injection = uint8(0);
% fault_injection = uint8(1);   % pour activer

