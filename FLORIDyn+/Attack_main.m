%% Main script for FLORIDyn Cyberattack simulations

%% Choosing windfield scenario
%   Options:
%   -   'no_change'                     -> No change in wind speed and direction
%   -   'direction_change'              -> Change in wind direction of 45 degrees
%   -   'realistic_no_change'           -> Realistic wind field with no change in wind speed and direction
%   -   'realistic_direction_change'    -> Realistic wind field with change in wind direction of 45 degrees
U.WindScenarios = 'no_change'; 

%% Choosing grid demand and critical values
U.demand = 31.8e6; % in W
U.marge = 0.1;      % fraction of demand

%% Choosing steady state time
U.t_steady = 300; % in seconds

%% Choosing attack scenario
%   Supported attack options:
%   -   "none"           -> No attack
%   -   "Brake"          -> Apply mechanical brake to the turbine
%   -   "change_yaw"     -> Change yaw angle of the turbine  
%   -   "Startup"        -> Startup command for the turbine
%   -   "P2S"            -> Pitch to stall (emergency stop)
%   -   "SetP_XX"        -> Set turbine to XX% of rated power (e.g., SetP_40 for 40%)

% Enter attack scenarios for each turbine:
%   Atk.TurbineX = [attack1, attack2, ...];
%   Atk.t_TurbineX = [time1, time2, ...];
%   Use "none" and NaN for unused slots.
Atk.Turbine1 = ["Brake", "none", "none", "none", "none"];
Atk.t_Turbine1 = [500, NaN, NaN, NaN, NaN];

Atk.Turbine2 = ["SetP_75", "SetP_100", "none", "none", "none"];
Atk.t_Turbine2 = [NaN, NaN, NaN, NaN, NaN];

Atk.Turbine3 = ["SetP_75", "SetP_100", "none", "none", "none"];
Atk.t_Turbine3 = [NaN, NaN, NaN, NaN, NaN];

Atk.Turbine4 = ["Brake", "Startup", "none", "none", "none"];
Atk.t_Turbine4 = [1, 511, NaN, NaN, NaN];

%% Running main script
% This will execute the main simulation with the above settings and attacksv
main(U, Atk);

