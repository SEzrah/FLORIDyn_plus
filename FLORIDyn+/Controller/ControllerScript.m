yaw = zeros(size(T.yaw));

%% Choosing control type
switch Control.Type
    case 'SOWFA_greedy_yaw'
        % Read yaw of SOWFA Sim (deg)
        for iT = 1:nT
            yaw(iT) = interp1(...
                Control.yawSOWFA(iT:nT:end,2),...
                Control.yawSOWFA(iT:nT:end,3),Sim.TimeSteps(k));
        end
        
        % Yaw conversion SOWFA to FLORIDyn
        yaw = (270*ones(size(yaw))-yaw)/180*pi;
        
        % Calculate Ct and Cp based on the wind speed
        %    Ct is restricted at 1, otherwise complex numbers appear in the FLORIS
        %    equations
        T.Cp    = interp1(VCpCt(:,1),VCpCt(:,2),T.u);
        T.Ct    = min(interp1(VCpCt(:,1),VCpCt(:,3),T.u),0.89);
    case 'SOWFA_bpa_tsr_yaw'
        % Read yaw of SOWFA Sim (deg)
        for iT = 1:nT
            yaw(iT) = interp1(...
                Control.yawSOWFA(iT:nT:end,2),...
                Control.yawSOWFA(iT:nT:end,3),Sim.TimeSteps(k));
        end
        
        % Yaw conversion SOWFA to FLORIDyn
        yaw = (270*ones(size(yaw))-yaw)/180*pi;
        
        % Ct / Cp calculation based on the blade pitch and tip speed ratio
        for iT = 1:nT
            bpa = max(interp1(bladePitch(iT:nT:end,2),bladePitch(iT:nT:end,3),Sim.TimeSteps(k)),0);
            tsr = interp1(tipSpeed(iT:nT:end,2),tipSpeed(iT:nT:end,3),Sim.TimeSteps(k))/T.u(1);
            T.Cp(iT) = Control.cpInterp(bpa,tsr);
            T.Ct(iT) = Control.ctInterp(bpa,tsr);
        end
    case 'FLORIDyn_greedy'
        % Calculate Ct and Cp based on the wind speed
        %    Ct is restricted at 1, otherwise complex numbers appear in the FLORIS
        %    equations
        T.Cp    = interp1(VCpCt(:,1),VCpCt(:,2),T.u);
        T.Ct    = min(interp1(VCpCt(:,1),VCpCt(:,3),T.u),0.89);
        
        % Normal yaw (yaw is defined clockwise)
        yaw = (-yaw)/180*pi;
    case 'AxialInduction'
        yaw = Control.yaw;  % Deg
        yaw = (-yaw)/180*pi;% Deg2rad + fitting yaw to SOWFA sim
        T.Ct = 4*T.axi.*(1-T.axi.*cos(yaw));
        T.Cp = 4*T.axi.*(1-T.axi).^2;
end


%% Setting yaw relative to wind direction. 
% If wind direction changes by more than 5 degrees, yaw is set to new wind direction with a rate of 1 degree per second.
if k==1
    T.yaw   = atan2(T.U(:,2),T.U(:,1));
    T.yaw   = T.yaw + yaw;
    changing_yaw = false;
else
    T.yaw_new = atan2(T.U(:,2),T.U(:,1));

    if abs(T.yaw_new - T.yaw) > (5 / 180 * pi)
        changing_yaw = true;
    elseif abs(T.yaw_new - T.yaw) < (1 / 180 * pi)
        changing_yaw = false;
    end

    if changing_yaw == true & T.yaw_new > T.yaw
        T.yaw = T.yaw + (0.5 * Sim.TimeStep / 180 * pi);
    elseif changing_yaw == true & T.yaw_new < T.yaw
        T.yaw = T.yaw - (0.5 * Sim.TimeStep / 180 * pi);
    else
        T.yaw = T.yaw;
    end
end
