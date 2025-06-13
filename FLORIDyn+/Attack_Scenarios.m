function [Atk] = Attack_Scenarios(Atk)

Atk.matrix = [Atk.Turbine1; Atk.Turbine2; Atk.Turbine3; Atk.Turbine4];
Atk.t_matrix = [Atk.t_Turbine1; Atk.t_Turbine2; Atk.t_Turbine3; Atk.t_Turbine4];

for x = 1:4
    Atk.Turbinex = Atk.matrix(x,:);
    Atk.t_Turbinex = Atk.t_matrix(x,:);

    for i = 1:length(Atk.Turbinex)
        switch Atk.Turbinex(i)
            case 'SetP_75'
                Atk.factor(x, i) = 0.75;
                Atk.t_downreg(x, i) = Atk.t_Turbinex(i);
            case 'SetP_40'
                Atk.factor(x, i) = 0.4;
                Atk.t_downreg(x, i) = Atk.t_Turbinex(i);
            case 'SetP_25'
                Atk.factor(x, i) = 0.25;
                Atk.t_downreg(x, i) = Atk.t_Turbinex(i);
            case 'SetP_100'
                Atk.factor(x, i) = 1.0;
                Atk.t_downreg(x, i) = Atk.t_Turbinex(i);
            case 'P2S'
                Atk.P2S(x, i) = true;
                Atk.t_P2S(x, i) = Atk.t_Turbinex(i);
            case 'Startup'
                Atk.startup(x, i) = true;
                Atk.t_startup(x, i) = Atk.t_Turbinex(i);
            case 'change_yaw'
                Atk.delta_yaw(x, i) = 40; 
                Atk.t_yaw_start(x, i) = Atk.t_Turbinex(i); 
                Atk.yaw_rate = 0.5; % degrees per second
            case 'Brake'
                Atk.brake(x, i) = true;
                Atk.t_brake(x, i) = Atk.t_Turbinex(i);
        end
    end
end
