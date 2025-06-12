function [y_filtered]=lowpassfilter(y_actual, y_set, Sim)
    time_per_rotation = 178.3 * pi / 90;                    %Time for one rotation in seconds at rated tip speed (90 m/s)
    omega_0 = (2 * pi) / time_per_rotation;                 %Rated speed in rad/s
    J = 199846144.5;                                        %Moment of inertia in kg*m^2
    H = 0.5 * J * omega_0^2 / (10 * 10^6);                  %Inertia time constant in seconds
    dy = (Sim.TimeStep/ (2*H)) * (y_set - y_actual);
    y_filtered = y_actual + dy;
end
