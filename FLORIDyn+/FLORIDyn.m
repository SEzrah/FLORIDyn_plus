function [powerHist, avgPower, OP,T,chain, C_t, wt_factors, n_turbines, WindHist]=FLORIDyn(T,OP,U,I,UF,Sim,fieldLims,VCpCt,chain,Vis,Control,Snap,Atk)
    % FLORIDyn simulation
    % INPUT 
    %   T           := Struct;    All data related to the turbines
    %    .pos       := [nx3] mat; x & y positions and nacelle height for all n
    %                             turbines.
    %    .D         := [nx1] vec; Diameter of all n turbines
    %    .yaw       := [nx1] vec; Yaw setting of the n turbines    (Allocation)
    %    .Ct        := [nx1] vec; Current Ct of the n turbines     (Allocation)
    %    .Cp        := [nx1] vec; Current Cp of the n turbines     (Allocation)
    %    .P         := [nx1] vec; Power production                 (Allocation)
    %
    %   OP          := Struct;    Data related to the state of the OPs
    %    .pos       := [nx3] vec; [x,y,z] world coord. (can be nx2)
    %    .dw        := [nx1] vec; downwind position (wake coordinates)
    %    .r         := [nx1] vec; Reduction factor: u = U*(1-r)
    %    .yaw       := [nx1] vec; yaw angle (wake coord.) at the time of creat.
    %    .Ct        := [nx1] vec; Ct coefficient at the time of creation
    %    .t_id      := [nx1] vec; Turbine OP belongs to
    %    .U         := [nx2] vec; Uninfluenced wind vector at OP position
    %    .I_f       := [nx1] vec; Foreign added turbunlence
    %
    %   U           := Struct;    All data related to the wind
    %    .abs       := [txn] mat; Absolute value of the wind vector at the n
    %                             measurement points for t time steps. If t=1,
    %                             the wind speed is constant.
    %    .ang       := [txn] mat; Same as .abs, but for the angle of the vector
    %
    %   I           := Struct;    All data connected to the ambient turbulence
    %    .val       := [txn] mat; Same as U.abs, but for the turbulence
    %                             intensity
    %    .pos       := [nx2] mat; Measurement positions (same as wind!)
    %
    %   UF          := Struct;    Data connected to the (wind) field
    %    .lims      := [2x2] mat; Interpolation area
    %    .IR        := [mxn] mat; Maps the n measurements to the m grid points
    %                             of the interpolated mesh
    %    .Res       := [1x2] mat; x and y resolution of the interpolation mesh
    %    .pos       := [nx2] mat; Measurement positions
    %    .airDen    := double;    AirDensity
    %    .alpha_z   := double;    Atmospheric stability (see above)
    %    .z_h       := double;    Height of the measurement
    %
    %   Sim         := Struct;    Simulation data
    %    .Duration  := double;    Duration of the Simulation in seconds
    %    .TimeStep  := double;    Duration of one time step
    %    .TimeSteps := [1xt] vec; All time steps
    %    .NoTimeSteps= int;       Number of time steps
    %    .FreeSpeed := bool;      OPs traveling with free wind speed or own
    %                             speed
    %    .WidthFactor= double;    Multiplication factor for the field width
    %    .Interaction= bool;      Whether the wakes interact with each other
    %    .redInteraction = bool;  All OPs calculate their interaction (false)
    %                             or only the OPs at the rotor plane (true)
    %
    %   fieldLims   := [2x2] mat; limits of the wind farm area (must not be the
    %                             same as the wind field!)
    %
    %   Pow         := Struct;    All data related to the power calculation
    %    .eta       := double;    Efficiency of the used turbine
    %    .p_p       := double;    cos(yaw) exponent for power calculation 
    %
    %   VCtCp       := [nx3];     Wind speed to Ct & Cp mapping
    %                               (Only used by controller script)
    %
    %   chain       := Struct;    Data related to the OP management / chains
    %    .NumChains := int;       Number of Chains per turbine
    %    .Length    := int/[nx1]; Length of the Chains - either uniform for all
    %                             chains or individually set for every chain.
    %    .List      := [nx5] vec; [Offset, start_id, length, t_id, relArea]
    %    .dstr      := [nx2] vec; Relative y,z distribution of the chain in the
    %                             wake, factor multiplied with the width, +-0.5
    % 
    %   Vis         := Struct;    What to visualize
    %    .online    := bool;      Enable online plotting
    %    .Snapshots := bool;      Store scatter plots as pictures (requires
    %                             online to be true)
    %    .FlowField := bool;      Plots the flow field
    %    .PowerOutput := bool;    Plots the power output after the simulation
    %    .Console   := bool;      Console output about the simulation state and
    %                             progress with time estimation
    %
    %   Control     := Struct;    Controller related settings and variables
    %    .init      := bool;      Initialize turbine states
    %    .Type      := String;    Control strategy
    %       [If needed]
    %    .yawSOWFA  := [n x t+1]vec; [time, yaw T0, yaw T1, ...] 
    %    .cpInterp  := scattered interpolant object; C_P as function of bpa/tsr
    %    .ctInterp  := scattered interpolant object; C_T as function of bpa/tsr
    % ======================================================================= %
    % OUTPUT
    %   powerHist   := [nx(nT+1)] [Time,P_T0,P_T1,...,]
    %                             Stores the time and the power output of the 
    %                             turbine at that time
    %
    %   OP          := Struct;    Data related to the state of the OPs
    %    .pos       := [nx3] vec; [x,y,z] world coord. (can be nx2)
    %    .dw        := [nx1] vec; downwind position (wake coordinates)
    %    .yaw       := [nx1] vec; yaw angle (wake coord.) at the time of creat.
    %    .Ct        := [nx1] vec; Ct coefficient at the time of creation
    %    .t_id      := [nx1] vec; Turbine OP belongs to
    %    .U         := [nx2] vec; Uninfluenced wind vector at OP position
    %    .u         := [nx1] vec; Effective wind speed at OP position
    %
    %   T           := Struct;    All data related to the turbines
    %    .pos       := [nx3] mat; x & y positions and nacelle height for all n
    %                             turbines.
    %    .D         := [nx1] vec; Diameter of all n turbines
    %    .yaw       := [nx1] vec; Yaw setting of the n turbines    (Allocation)
    %    .Ct        := [nx1] vec; Current Ct of the n turbines     (Allocation)
    %    .Cp        := [nx1] vec; Current Cp of the n turbines     (Allocation)
    %    .P         := [nx1] vec; Power production                 (Allocation)
    % 
    %   UF          := Struct;    Data connected to the (wind) field
    %    .lims      := [2x2] mat; Interpolation area
    %    .IR        := [mxn] mat; Maps the n measurements to the m grid points
    %                             of the interpolated mesh
    %    .Res       := [1x2] mat; x and y resolution of the interpolation mesh
    %    .pos       := [nx2] mat; Measurement positions
    %    .airDen    := double;    AirDensity
    %    .alpha_z   := double;    Atmospheric stability (see above)
    %    .z_h       := double;    Height of the measurement
    %
    %   Sim         := Struct;    Data connected to the Simulation
    %    .Duration  := double;    Duration of the Simulation in seconds
    %    .TimeStep  := double;    Duration of one time step
    %    .TimeSteps := [1xt] vec; All time steps
    %    .NoTimeSteps= int;       Number of time steps
    %    .FreeSpeed := bool;      OPs traveling with free wind speed or own
    %                             speed
    %    .WidthFactor= double;    Multiplication factor for the field width
    %    .Interaction= bool;      Whether the wakes interact with each other
    %    .redInteraction = bool;  All OPs calculate their interaction (false)
    %                             or only the OPs at the rotor plane (true)
    % ======================================================================= %

    %% Preparation for Simulation
    %   Script starts the visulization, checks whether the field variables are
    %   changing over time, prepares the console progress output and sets
    %   values for the turbines and observation points which may not be 0
    %   before the simulation starts.
    SimulationPrep;
    
    n_turbines = size(T.pos,1); % Number of turbines
    
    C_t = zeros(1, n_turbines);         % Store Ct values for load effects
    wt_factors = zeros(1, n_turbines);  % Store weight factors for load effects

%% initializing variables
    % % yaw changes
    % T.yaw_new = zeros(1, size(T.pos, 2)); % Store current yaw values during shutdown
    % T.is_yawing = false(1, size(T.pos, 2)); % Track which turbines are yawing
    % T.yaw_initial = zeros(1, size(T.pos, 2));
    % T.yaw_override = false(1, size(T.pos, 2));

    T.P2S = false(1, n_turbines);
    T.shutdown = false(1, n_turbines);
    T.trackP = false(1, n_turbines);
    T.brake = false(1, n_turbines);
    T.startup = false(1, n_turbines);
    T.factor = zeros(1, n_turbines);

    T.currentstate = repmat("Greedy", 1, n_turbines);

    for k = 1:Sim.NoTimeSteps
        if Vis.Console;tic;end

  
        % Update measurements if they are variable
        if UangVar; U_ang = U.ang(k,:); end
        if UabsVar; U_abs = U.abs(k,:); end
        if IVar;    I_val = I.val(k,:); end
        
        %================= CONTROLLER & POWER CALCULATION ====================%
        % Update Turbine data to get controller input
        T.U = getWindVec4(T.pos, U_abs, U_ang, UF);
        T.I0 = getAmbientTurbulence(T.pos, UF.IR, I_val, UF.Res, UF.lims);

        % ControllerScript;
        [T] = Finite_State_Machine(k, T, n_turbines, UF, Sim, Atk);

        %% Setting yaw relative to wind direction. 
        % If wind direction changes by more than 5 degrees, yaw is set to new wind direction with a rate of 1 degree per second.
    

        T.yaw   = atan2(T.U(:,2),T.U(:,1));

        % T.yaw(1) = T.yaw(1) + 10 / 180 * pi; % Adjust yaw for the first turbine


        % if k > 300 && k < 350
        %     T.yaw(1) = T.yaw(1) + (0.5 * (k-300) / 180 * pi);
        %     T.yaw(3) = T.yaw(3) + (0.5 * (k-300) / 180 * pi);
        % elseif k >= 340
        %     T.yaw(1) = T.yaw(1) + 25/180*pi;
        %     T.yaw(3) = T.yaw(3) + 25/180*pi;
        % end


        % if k==1
        %     T.yaw   = atan2(T.U(:,2),T.U(:,1));
        %     changing_yaw = false;
        % else
        %     T.yaw_new = atan2(T.U(:,2),T.U(:,1));

        %     if abs(T.yaw_new - T.yaw) > (5 / 180 * pi)
        %         changing_yaw = true;
        %     elseif abs(T.yaw_new - T.yaw) < (1 / 180 * pi)
        %         changing_yaw = false;
        %     end

        %     if changing_yaw == true & T.yaw_new > T.yaw
        %         T.yaw = T.yaw + (0.5 * Sim.TimeStep / 180 * pi);
        %     elseif changing_yaw == true & T.yaw_new < T.yaw
        %         T.yaw = T.yaw - (0.5 * Sim.TimeStep / 180 * pi);
        %     else
        %         T.yaw = T.yaw;
        %     end
        % end


        % if ~isempty(delta_yaw)
        %     % Convert times to simulation steps k
        %     k_yaw_start = round(t_yaw_start ./ Sim.TimeStep);
        %     k_yaw_end = round(t_yaw_end ./ Sim.TimeStep);
            
        %     % Find turbines that need to start yawing at this step
        %     idx_yaw = find(~isnan(k_yaw_start) & k_yaw_start == k);
            
        %     % Initialize yaw for turbines starting at this step
        %     if ~isempty(idx_yaw)
        %         T.is_yawing(idx_yaw) = true;
        %         T.yaw_initial(idx_yaw) = T.yaw(idx_yaw);
        %     end
            
        %     % Find turbines that need to stop yawing at this step
        %     idx_stop_yaw = find(~isnan(k_yaw_end) & k_yaw_end == k);

        %     if ~isempty(idx_stop_yaw)
        %         T.is_yawing(idx_stop_yaw) = false; % Lock the yaw at this position
        %         T.yaw_override(idx_stop_yaw) = true;
        %     end
            
        %     yawing_turbine = find(T.is_yawing);
        %     for i = 1:length(yawing_turbine)
        %         t = yawing_turbine(i);
                
        %         % Gradually change yaw angle
        %         if k > k_yaw_start(find(~isnan(k_yaw_start) & t == 1:length(k_yaw_start), 1))
        %            T.yaw(t) = T.yaw(t) + (delta_yaw(t)) * (k - k_yaw_start(t)) / (k_yaw_end(t) - k_yaw_start(t));
        %         end
        %     end
        %     idx_override = find(T.yaw_override);
        %     T.yaw(idx_override) = T.yaw_initial(idx_override) + delta_yaw(idx_override);
        % end

        for t = 1:n_turbines
            if ~isempty(Atk.factor(t,:))
                idx_downreg = find(~isnan(Atk.factor(t,:)));

                for idx = idx_downreg
                    k_downreg = round(Atk.t_downreg(t,idx) ./ Sim.TimeStep);
                    if k == k_downreg
                        T.trackP(t) = true;
                        T.factor(t) = Atk.factor(t,idx);
                    end
                end
            end    
        
            if ~isempty(Atk.brake(t,:))
                idx_brake = find(~isnan(Atk.brake(t,:)));

                for idx = idx_brake
                    k_brake = round(Atk.t_brake(t,idx) / Sim.TimeStep);
                    if k == k_brake
                        T.brake(t) = true;
                    end
                end
            end

            if ~isempty(Atk.P2S(t,:))
                idx_P2S = find(~isnan(Atk.P2S(t,:)));

                for idx = idx_P2S
                    k_P2S = round(Atk.t_P2S(t,idx) / Sim.TimeStep);
                    if k == k_P2S
                        T.P2S(t) = true;
                    end
                end
            end

            if ~isempty(Atk.startup(t,:))
                idx_startup = find(~isnan(Atk.startup(t,:)));

                for idx = idx_startup
                    k_startup = round(Atk.t_startup(t,idx) / Sim.TimeStep);
                    if k == k_startup
                        T.startup(t) = true;
                    end
                end
            end
        end

        %% Calculate the power output of the turbines
        T.P = 0.5*UF.airDen*(T.D/2).^2.*pi.*T.Cp.*T.u.^3.* T.param.eta.*cos(T.yaw-atan2(T.U(:,2),T.U(:,1))).^T.param.p_p;

        
        powerHist(:,k)= T.P;
        avgPower(k) = mean(T.P);         
        WindHist(:,k)  = T.u;
        
        % disp(T.Ct);
        %================= INSERT NEW OBSERVATION POINTS =====================%
        OP = initAtRotorPlane(OP, chain, T);
        
        %====================== INCREMENT POSITION ===========================%
        % Update wind dir and speed along with amb. turbulence intensity
        OP.U = getWindVec4(OP.pos, U_abs, U_ang, UF);
        
        OP.I_0 = getAmbientTurbulence(OP.pos, UF.IR, I_val, UF.Res, UF.lims);
        
        % Save old position for plotting if needed
        if or(Vis.online,and(Vis.FlowField,k == Sim.NoTimeSteps))
            OP_pos_old = OP.pos; %#ok<NASGU>
        end 
        
        % Enable full interaction for last plot to get true flow field for the
        % plot
        if and(Vis.FlowField,k == Sim.NoTimeSteps)
            Sim.reducedInteraction = false;
        end
        
        % Calculate the down and crosswind steps along with the windspeed at
        % the turbine rotor planes
        [OP, T]=makeStep2(OP, chain, T, Sim);
        
        % Increment the index of the chain starting entry
        chain.List = shiftChainList(chain.List);
        
        %===================== ONLINE VISULIZATION ===========================%
        if Vis.online; OnlineVis_plot; end
        % if and(Vis.FlowField,k == 0.5*Sim.NoTimeSteps)
        %     if Vis.online; hold off; end
        %     PostSimVis;
        % end

        if and(Vis.FlowField, k == Snap.time_1/Sim.TimeStep)
            if Vis.online; hold off; end
            PostSimVis
        end
        if and(Vis.FlowField, k == Snap.time_2/Sim.TimeStep)
            if Vis.online; hold off; end
            PostSimVis
        end

        % Display the current simulation progress
        if Vis.Console;ProgressScript;end

        %===================== LOADS VISUALISATION ===========================%
        if Vis.LoadEffects
            C_t_k = T.Ct.';
            C_t = cat(1, C_t, C_t_k);

            wt_factors_k = 1 - T.u.' ./ U.windSpeed;
            wt_factors   = cat(1, wt_factors, wt_factors_k);
        end      
    end
    
    C_t(1,:) = [];
    wt_factors(1, :)    = [];


    %% Store power output together with time line
    powerHist = powerHist';
    powerHist = [Sim.TimeSteps',powerHist];
end

    %% ===================================================================== %%
    % = Reviewed: 2020.12.23 (yyyy.mm.dd)                                   = %
    % === Author: Marcus Becker                                             = %
    % == Contact: marcus.becker@tudelft.nl                                  = %
    % ======================================================================= %