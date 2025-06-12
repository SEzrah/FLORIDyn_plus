function main(U, Atk)
main_addPaths;

%% Set controller type
%   Options: 
%   -   'SOWFA_greedy_yaw' 
%   -   'SOWFA_bpa_tsr_yaw'
%   -   'FLORIDyn_greedy'
%   -   'AxialInduction'

% Control.init:
%   Set to true if you are starting a new simulation, if you are copying
%   the states from a previous simulation, set to false.

Control.Type = 'FLORIDyn_greedy';
Control.init = true;

%% Getting demand, critical values and steady state time
demand = U.demand;                     % in W
demand_upper = demand * (1 + U.marge); % upper limit with margin
demand_lower = demand * (1 - U.marge); % lower limit with margin
t_steady = U.t_steady;                 % steady state time

%% Load Layout
[T,fieldLims,VCpCt,chain] = loadLayout('Customizable',...
    "position",[...
        500 500 170 284;...     % T0
        1500 500 170 284;...     % T1
        2500 500 170 284;...     % T2
        3500 500 170 284;...     % T3
        ],...
    "fieldLim",[0 0; 4500 2000]);

%% Setting default variables for attack scenarios
nT = length(T.D); % number of turbines
Control.yaw  = T.yaw;

% Initialize attack scenario variables (NaN or false by default)
Control.delta_yaw = NaN(1, nT);
Control.t_yaw_start = NaN(1, nT);
Control.t_yaw_end = NaN(1, nT);

Atk.factor = NaN(5, nT);
Atk.t_downreg = NaN(5, nT);

Atk.P2S = NaN(5, nT);
Atk.t_P2S = NaN(5, nT);

Atk.brake = NaN(5, nT);
Atk.t_brake = NaN(5, nT);

Atk.startup = NaN(5, nT);
Atk.t_startup = NaN(5, nT);

%% Load the environment
%       'no_change'                 -> Constant wind speed, direction and 
%                                       amb. turbulence
%       'direction_change'          -> Wind direction change at a specified tim with 
%                                       a duration of a given time (all places at the same time)  
%       'realistic              '   -> Wind field modeled as an Ornstein-Uhlenbeck process
%                                       change starting after 300s
switch U.WindScenarios
    case 'no_change'
        [U, I, UF, Sim] = loadWindField('Custom',... 
            'windAngle_start',45,... %[deg]
            'windAngle_end',45,...  %[deg]
            'ChangeTime',400,...    %[s]
            'Change_Duration',300,...%[s] rotate speed should be between 0.5 and 1 deg/s
            'SimDuration',800,...   %[s]
            'FreeSpeed',true,...
            'Interaction',true,...
            'posMeasFactor',2000,...
            'alpha_z',0.1,...
            'windSpeed',8,...
            'ambTurbulence',0.06);
    case 'direction_change'
        [U, I, UF, Sim] = loadWindField('Custom',... 
            'windAngle_start',45,... %[deg]
            'windAngle_end',90,...  %[deg]
            'ChangeTime',100,...    %[s]
            'Change_Duration',400,...%[s] rotate speed should be between 0.5 and 1 deg/s
            'SimDuration',700,...   %[s]
            'FreeSpeed',true,...
            'Interaction',true,...
            'posMeasFactor',2000,...
            'alpha_z',0.1,...
            'windSpeed',6,...
            'ambTurbulence',0.06);
    case 'realistic_no_change'
        [U, I, UF, Sim] = loadWindField('Normal_distribution',... 
            'windAngle_start',45,... %[deg]
            'windAngle_end',45,...  %[deg]
            'ChangeTime',800,...    %[s]
            'Change_Duration',100,...%[s] rotate speed should be between 0.5 and 1 deg/s
            'SimDuration',1500,...   %[s]
            'FreeSpeed',true,...
            'Interaction',true,...
            'posMeasFactor',1000,...
            'alpha_z',0.1,...
            'windSpeed',8,...
            'ambTurbulence',0.06);
    case 'realistic_direction_change'
        [U, I, UF, Sim] = loadWindField('Normal_distribution',... 
            'windAngle_start',45,... %[deg]
            'windAngle_end',90,...  %[deg]
            'ChangeTime',100,...    %[s]
            'Change_Duration',200,...%[s] rotate speed should be between 0.5 and 1 deg/s
            'SimDuration',500,...   %[s]
            'FreeSpeed',true,...
            'Interaction',true,...
            'posMeasFactor',2000,...
            'alpha_z',0.1,...
            'windSpeed',10,...
            'ambTurbulence',0.06);
end

%% Visulization options
% Set to true or false to enable/disable various plots and outputs
% Only the selected plots will be generated and their data will be saved to the 'data' folder.
% This allows you to reduce computational load by disabling unnecessary plots and exports.
%   .online:      Live OPs and wind field plot
%   .Snapshots:   Save OP plots (requires .online = true)
%   .FlowField:   Plot flow field at end of simulation
%   .PowerOutput: Plot generated power at end
%   .Console:     Show simulation progress in console
%   .LoadEffects: Plot fatigue load coefficients (Not Yet Implemented)
%   .Windspeed:   Plot windspeed per turbine
%   .totPower:    Plot total power output
%   .Cor:         Plot correlation between wind and power
Vis.online      = false;
Vis.Snapshots   = false;
Snap.time_1     =  20;
Snap.time_2     =  40;
Vis.FlowField   = false;
Vis.PowerOutput = true;
Vis.Console     = true;
Vis.LoadEffects = false;
Vis.Windspeed   = true;
Vis.totPower    = true;
Vis.Cor         = true; 
%% Create starting OPs and build opList
%   Creates the observation point struct (OP) and extends the chain struct.
%   Here, the distribution of the OPs in the wake is set. Currently the
%   avaiable distributions are:
%   'sunflower'         : Recommended distibution with equal spread of the 
%                           OPs across the rotor plane.
%   '2D_horizontal'     : OPs in two horizontal planes, silightly above and
%                           below hub height
%   '2D_vertical'       : OPs in two vertical planes, right and left of the
%                           narcelle.

[OP, chain] = assembleOPList(chain,T,'sunflower');

%% Running attack scenarios
[Atk] = Attack_Scenarios(Atk);

%% Running FLORIDyn
[powerHist,avgPower, OP,T,chain, C_t, wt_factors, n_turbines, WindHist]=...
    FLORIDyn(T,OP,U,I,UF,Sim,fieldLims,VCpCt,chain,Vis,Control, Snap, Atk); 

%% Slicing data starting from steady state
% finding the index of the first time step that is greater than or equal to t_steady
idx_steady = find(powerHist(:,1) >= t_steady, 1, 'first');
% slicing the data from that index onwards
powerHist = powerHist(idx_steady:end, :);
WindHist = WindHist(:, idx_steady:end);
avgPower = avgPower(:, idx_steady:end);

powerHist(:,1) = powerHist(:,1) - t_steady; % Adjust time to start from 0

%% Compare power plot
if Vis.PowerOutput
    % --- Individual Turbine and Average Power Plot ---
    f1 = figure;
    hold on
    nT = length(T.D);
    labels = cell(nT + 1, 1);  % For each turbine + average

    % ========== FLORIDyn data ==========
    for iT = 1:nT
        plot(powerHist(:,1), powerHist(:,iT+1), 'LineWidth', 1.5)
        labels{iT} = ['T' num2str(iT-1) ' FLORIDyn'];
    end

    % ====== Average Power (same plot) ======
    plot(powerHist(:,1), avgPower, 'k--', 'LineWidth', 2)  % dashed black
    labels{nT+1} = 'Average Power';

    % Export power history and average power to Excel
    writematrix(powerHist, fullfile('data', 'powerHist.xlsx'));
    writematrix(avgPower', fullfile('data', 'avgPower.xlsx'));           

    hold off
    grid on
    xlim([0 powerHist(end,1)])
    xlabel('Time [s]')
    ylabel('Power generated [W]')
    title([num2str(nT) ' turbine case'])
    legend(labels)

    % ==== Prep for export ==== %
    f1.Units               = 'centimeters';
    f1.Position(3)         = 16.1; % A4 line width
    set(gca,'LooseInset', max(get(gca,'TightInset'), 0.04))
    f1.PaperPositionMode   = 'auto';
end

%% total Power Plot
if Vis.totPower
    % --- Total Power Plot (Separate Figure) ---
    f2 = figure;
    totalPower = sum(powerHist(:, 2:end), 2); % sum across turbines
   
    % Identify crossing points
    is_outside = (totalPower > demand_upper) | (totalPower < demand_lower);
    % Detect transitions (crossings) by comparing with previous point
    cross_idx = find(diff(is_outside) ~= 0) + 1; % +1 to get the point after the transition
    cross_times = powerHist(cross_idx, 1); % Get time values for crossings

    plot(powerHist(:,1), totalPower, 'r-', 'LineWidth', 2)
    hold on
    yline(demand, 'b--', 'LineWidth', 2); % blue dashed line
    yline(demand_upper, 'g--', 'LineWidth', 1.5); % green dashed line for upper limit
    yline(demand_lower, 'm--', 'LineWidth', 1.5); % magenta dashed line for lower limit

    % Plot vertical line at each crossing time with time label
    for i = 1:length(cross_times)
        xline(cross_times(i), 'k:', 'LineWidth', 2, ...
            'Label', sprintf('Cross: %.1fs', cross_times(i)), ...
            'LabelOrientation', 'horizontal', ...
            'LabelVerticalAlignment', 'bottom');
    end

    % Export total power to Excel
    writematrix(totalPower, fullfile('data', 'totalPower.xlsx'));

    hold off
    grid on
    xlim([0 powerHist(end,1)])
    xlabel('Time [s]')
    ylabel('Total Power [W]')
    title([num2str(nT) ' turbine case - Total Power'])
    % legend('Total Power', 'Demand', 'Demand Upper', 'Demand Lower', 'Cross')

    % ==== Prep for export ==== %
    f2.Units               = 'centimeters';
    f2.Position(3)         = 16.1;
    set(gca,'LooseInset', max(get(gca,'TightInset'), 0.04))
    f2.PaperPositionMode   = 'auto';
end


%% Load visualisation
% Not correctly implemented yet, but this is the place to do it
% Currently based on W. Cai, Y. Hu, F. Fang, L. Yao, and J. Liu, “Wind farm power production and fatigue load optimization based on dynamic partitioning and wake redirection of wind turbines,” Applied Energy, vol. 339, p. 121000, Jun. 2023, doi: 10.1016/j.apenergy.2023.121000.
if Vis.LoadEffects
    C_dis       = 0.7;                                           % disturbance coefficient, uit de literatuur
    
    C_fat       = powerHist(:, 1);                               % de eerste kolom van C_fat zijn de tijdstippen

    for n_turbine = 1:n_turbines

        % f_power
        P_sim       = powerHist(:, n_turbine+1);                 % de power geAtkurende de simulatie (W), vector met hoogte = NoTimeSteps
        T_sim       = Sim.Duration;           	                 % de lengte van de simulatie in seconden (? of aantal?), scalar
        P_int       = cumtrapz(Sim.TimeStep, P_sim.');           % integraal van P over 0 tot t, trapeziodal method, breedte van intervallen = TimeStep

        f_p         = P_int ./(Pow.P_rated * T_sim);                 % array met breedte NoTimeSteps en hoogte 1

        % f_turbulence
        I_a         = 0.06;                                      % dit moet gewoon ambTurbulence zijn maar die is lokaal in loadWindField.m dus even zo, gebeund
        C_t_sim     = C_t(:, n_turbine);                         % vector met hoogte = NoTimeSteps
        wt_factor   = wt_factors(n_turbine);
        I_w         = wt_factor .* (1.2 .* C_t_sim).^(0.5);      % vector met hoogte = NoTimeSteps, wake turbulence
        I_eff       = (I_a^2 + I_w.^2).^(0.5);                   % vector met hoogte = NoTimeSteps
        I_int       = cumtrapz(Sim.TimeStep, I_eff.');           % integraal van I_eff over 0 tot t


        f_t         = I_int ./ T_sim;                            % array met breedte NoTimeSteps en hoogte 1

        C_fat_n     = f_p + C_dis .* f_t;                        % array met breedte NoTimeSteps en hoogte 1

        C_fat       = cat(2, C_fat, C_fat_n.');                  % voor elke turbine voegen we de kolom met C_fat per tijdstip toe aan de matrix C_fat die begint met de kolom met tijdstappen
    end


    % Plotting
    f = figure;
    hold on
    nT = length(T.D);
    labels = cell(nT,1);
    for n_turbine = 1:n_turbines
        plot(C_fat(:,1),C_fat(:, n_turbine+1),'LineWidth',1.5)         % uitzetten van de C_fat tegen de tijd
        labels{n_turbine} = ['T' num2str(n_turbine-1)];                % het vullen van de lege labels-array met de juiste labels
    end

    hold off

    dim_OPdotU = size(OP.U);
    %disp(dim_OPdotU)

    dim_TdotU  = size(T.U);
    %disp(dim_TdotU)
    %disp(T.U)



    dim_wt_factors = size(wt_factors);

    % Export fatigue load coefficients to Excel
    writematrix(C_fat, fullfile('data','fatigueLoadCoefficients.xlsx'));

    
    grid on
    xlim([0 powerHist(end,1)])
    xlabel('Time [s]')
    ylabel('Fatigue load coefficients')
    legend(labels)
    % ==== Prep for export ==== %
    % scaling
    f.Units               = 'centimeters';
    f.Position(3)         = 16.1; % A4 line width
    set(gca,'LooseInset', max(get(gca,'TightInset'), 0.04))
    f.PaperPositionMode   = 'auto';

end    

%% Compare windspeed plot
if Vis.Windspeed
    % Plotting
    f = figure;
    hold on
    nT = length(T.D);
    labels = cell(nT,1);
    
    % ========== FLORIDyn data =========
    for iT = 1:length(T.D)
        plot(powerHist(:,1),WindHist(iT, :),'LineWidth',1.5)
        labels{end-nT+iT} = ['T' num2str(iT-1) ' FLORIDyn'];
    end
    
    % Export windspeed history to Excel
    writematrix(WindHist', fullfile('data','windHist.xlsx'));

    hold off
    grid on
    xlim([0 powerHist(end,1)])
    xlabel('Time [s]')
    ylabel('Windspeed per turbine in [m/s]')
    title([num2str(nT) ' turbine case'])
    legend(labels)
    
    % ==== Prep for export ==== %
    % scaling
    f.Units               = 'centimeters';
    f.Position(3)         = 16.1; % A4 line width
    set(gca,'LooseInset', max(get(gca,'TightInset'), 0.04))
    f.PaperPositionMode   = 'auto';
end

%% Correlation plot
if Vis.Cor
    WindHist_T = WindHist';
    avgPower = sum(powerHist(:, 2:end), 2);  % sum across turbines
    avgWind = mean(WindHist_T(:, 2:end), 2);      % Y

    % Define moving window size (in time steps)
    windowSize = 300;

    % Compute moving correlation
    rollingCorr = movcorr(avgWind, avgPower, windowSize, 'Endpoints', 'shrink');
    
    % Export rolling correlation to Excel
    writematrix(rollingCorr, fullfile('data','rollingCorr.xlsx')); 

    % Plot
    figure;
    plot(powerHist(:,1), rollingCorr, 'b-', 'LineWidth', 2)
    xlabel('Time [s]')
    ylabel('Rolling Correlation Coefficient')
    title(['Moving Correlation Between Wind Speed and Power (Window = ' num2str(windowSize) ')'])
    grid on

    detection = min(rollingCorr) < 0.2; % Threshold for detection, adjust as needed
    if detection
        [minimum, t_detection] = min(rollingCorr);
        disp('Cyberattack detected at time:');
        disp(t_detection + 300);
    else
        disp('No cyberattack detected');
    end
    
end