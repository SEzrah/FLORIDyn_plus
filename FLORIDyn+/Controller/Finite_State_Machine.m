function [T] = Finite_State_Machine(k, T, n_turbines, UF, Sim, Atk)

%% initialisation
windspd_vec = T.VCtCp.windspd_vec;
Cp_vec = T.VCtCp.Cp_vec;
beta_vec_ws = T.VCtCp.beta_vec_ws;
omega_vec_ws = T.VCtCp.omega_vec_ws;

beta_vec = T.CpCtTSR.beta_vec;
lambda_vec = T.CpCtTSR.lambda_vec;
Cp_matrix = T.CpCtTSR.Cp_matrix;
Ct_matrix = T.CpCtTSR.Ct_matrix;

J = T.param.J;                                                   % inertia in kg*m^2
R = T.param.R;                                                   % Rotor radius [m]
A = T.param.A;                                                   % rotor area (m^2)   
pitch_rate = T.param.pitch_rate;                                       % deg/s
rated_ws = T.param.rated_ws;                                           % m/s

rho = UF.airDen;                        
dt = Sim.TimeStep;


%% Update turbine states
disp(T.currentstate);
for i = 1:length(T.currentstate)
    switch T.currentstate(i)
          case 'Greedy'
               T.beta(i) = interp1(windspd_vec, beta_vec_ws, T.u(i));
               T.omega(i) = interp1(windspd_vec, omega_vec_ws, T.u(i)) * 2*pi / 60;
               T.lambda(i) = T.omega(i) * R / T.u(i);

               T.Cp(i) = interp2(beta_vec, lambda_vec, Cp_matrix, T.beta(i), T.lambda(i));
               T.Ct(i) = interp2(beta_vec, lambda_vec, Ct_matrix, T.beta(i), T.lambda(i));

               if T.trackP(i) == true & ~(Atk.factor(i) == 1)
                    T.currentstate(i) = 'P_setpoint';
               elseif T.brake(i) == true | T.P2S(i) == true
                    T.currentstate(i) = 'Shutdown';
               else
                    T.currentstate(i) = 'Greedy';
               end
          
          case 'P_setpoint'
               % Setting Cp setpoint
               T.beta_greedy(i) = interp1(windspd_vec, beta_vec_ws, T.u(i));
               T.omega_greedy(i) = interp1(windspd_vec, omega_vec_ws, T.u(i)) * 2*pi / 60;
               T.lambda_greedy(i) = T.omega_greedy(i) * R / T.u(i);
               T.Cp_greedy(i) = interp2(beta_vec, lambda_vec, Cp_matrix, T.beta_greedy(i), T.lambda_greedy(i));
               T.Cp_desired(i) = T.factor(i) * T.Cp_greedy(i);
               T.idx_lambda = interp1(lambda_vec, 1:length(lambda_vec), T.lambda_greedy(i));

               if T.Cp_desired(i) < T.Cp(i)
                    T.Kp = 8e7;
                    T.Ki = 8e6;
               else
                    T.Kp = 3e8;
                    T.Ki = 8e6;
               end

               T.integral_error = zeros(1, n_turbines);
               T.error = zeros(1, n_turbines);
               T.trackP(i) = false;
               
               if T.u(i) <= rated_ws
                    T.currentstate(i) = 'Gen_torque';
               else
                    T.currentstate(i) = 'Pitch_control';
               end

          case 'Gen_torque'
               Kp = T.Kp;
               Ki = T.Ki; 

               Cp_curve = interp1(beta_vec, Cp_matrix', T.beta(i))';
               T.lambda_desired(i) = interp1(Cp_curve(1:ceil(T.idx_lambda)), lambda_vec(1:ceil(T.idx_lambda)), T.Cp_desired(i));

               T.omega_desired(i) = T.lambda_desired(i) * T.u(i) / R;

               % PI-controller for generator torque
               T.error(i) = T.omega(i) - T.omega_desired(i);
               T.integral_error(i) = T.integral_error(i) + T.error(i) * dt;
               T.integral_error(i) = max(-0.1, T.integral_error(i)); 
               T_g = Kp * T.error(i) + Ki * T.integral_error(i);
               T_g = max(0, T_g);  
               T.Cq = T.Ct(i) / T.lambda(i);
               T_a = 0.5 * rho * A * T.Cq * (T.u(i)^3);
               domega = (T_a - T_g) / J;
               disp(T.error(i));
               disp(T.integral_error(i));
               T.omega(i) = T.omega(i) + dt * domega;  
               T.lambda(i) = T.omega(i) * R / T.u(i);

               T.Cp(i) = interp2(beta_vec, lambda_vec, Cp_matrix, T.beta(i), T.lambda(i));
               T.Ct(i) = interp2(beta_vec, lambda_vec, Ct_matrix, T.beta(i), T.lambda(i));
               if T.brake(i) == true | T.P2S(i) == true
                    T.currentstate(i) = 'Shutdown';
               elseif T.trackP(i) == true
                    T.currentstate(i) = 'P_setpoint';
               elseif T.omega(i) == T.omega_desired(i) | abs(T.omega(i)-T.omega_desired(i)) < 0.0001
                    T.currentstate(i) = 'Steady';
               else
                    T.currentstate(i) = 'Gen_torque';
               end

          case 'Pitch_control'
               Kp = 3e8;
               Ki = 3e7; %8e7

               % Find desired TSR and Pitch angle
               T.omega_desired(i) = interp1(Cp_vec, omega_vec_ws, T.Cp_desired(i)) * 2*pi / 60;
               T.lambda_desired(i) = T.omega_desired(i) * R / T.u(i);
               Cp_curve = interp1(lambda_vec, Cp_matrix, T.lambda_desired(i));
               T.beta_desired(i) = interp1(Cp_curve(2:end), beta_vec(2:end), T.Cp_desired(i));

               % Updating pitch angle
               beta_error = T.beta_desired(i) - T.beta(i);
               beta_step = sign(beta_error) * min(abs(beta_error), pitch_rate * dt);
               T.beta(i) = T.beta(i) + beta_step;

               %PI-controller for Generator torque
               T.Cq = T.Ct(i) / T.lambda(i);
               T_a = 0.5 * rho * A * T.Cq * (T.u(i)^3);   
               error = T.omega(i) - T.omega_desired(i);
               T.integral_error(i) = T.integral_error(i) + error * dt;
               T_g = Kp * error + Ki * T.integral_error(i);
               domega = (T_a - T_g) / J;
               T.omega(i) = T.omega(i) + dt * domega;
               T.lambda(i) = T.omega(i) * R / T.u(i);

               T.Cp(i) = interp2(beta_vec, lambda_vec, Cp_matrix, T.beta(i), T.lambda(i));
               T.Ct(i) = interp2(beta_vec, lambda_vec, Ct_matrix, T.beta(i), T.lambda(i));
               if T.brake(i) == true | T.P2S(i) == true;
                    T.currentstate(i) = 'Shutdown';
               elseif T.omega(i) == T.omega_desired(i) | abs(T.omega(i)-T.omega_desired(i)) < 0.0001
                    T.currentstate(i) = 'Steady';
               else
                    T.currentstate(i) = 'Pitch_control';
               end
          
          case 'Steady'
               T.Cp(i) = interp2(beta_vec, lambda_vec, Cp_matrix, T.beta(i), T.lambda(i));
               T.Ct(i) = interp2(beta_vec, lambda_vec, Ct_matrix, T.beta(i), T.lambda(i));

               if T.brake(i) == true | T.P2S(i) == true
                    T.currentstate(i) = 'Shutdown';
               elseif T.trackP(i) == true
                    T.currentstate(i) = 'P_setpoint';
               else
                    T.currentstate(i) = 'Steady';
               end

          case 'Shutdown'
               T.Cq = T.Ct(i) / T.lambda(i);
               T_a = 0.5 * rho * A * T.Cq * (T.u(i)^3);
               T.gen_torque_fixed(i) = T_a;

               if T.brake(i) == true
                    T.currentstate(i) = 'Emergency_brake';
               else
                    T.currentstate(i) = 'Pitch_to_stall';
               end

          case 'Emergency_brake'
               brake_torque = 5e6;  % Arbitrary large torque
               T.Cq = T.Ct(i) / T.lambda(i);
               T_a = 0.5 * rho * A * T.Cq * (T.u(i)^3);
               domega = (T_a - (brake_torque + T.gen_torque_fixed(i))) / J;
               T.omega(i) = max(0.001, T.omega(i) + dt * domega); 
               T.lambda(i) = T.omega(i) * R / T.u(i);
               T.Cp(i) = max(0.01, interp2(beta_vec, lambda_vec, Cp_matrix, T.beta(i), T.lambda(i)));
               T.Ct(i) = max(0.01,interp2(beta_vec, lambda_vec, Ct_matrix, T.beta(i), T.lambda(i)));

               if T.Cp(i) == 0.01;
                    T.brake(i) = false;
                    T.currentstate(i) = 'Stalled';
               else
                    T.currentstate(i) = 'Emergency_brake';
               end

          case 'Pitch_to_stall'
               Kp = 1e7;
               Ki = 1e6;
               T.beta_desired(i) = 45;
               beta_error = T.beta_desired(i) - T.beta(i);
               beta_step = sign(beta_error) * min(abs(beta_error), pitch_rate * dt);
               T.beta(i) = T.beta(i) + beta_step;


               %PI-controller for Generator torque
               T.omega_desired(i) = max(T.omega_desired(i) - 0.1, 0);
               T.Cq = T.Ct(i) / T.lambda(i);
               T_a = 0.5 * rho * A * T.Cq * (T.u(i)^3);   
               error = T.omega(i) - T.omega_desired(i);
               T.integral_error(i) = T.integral_error(i) + error * dt;
               T_g = Kp * error + Ki * T.integral_error(i);
               T_g = max(0, T_g);
     
               domega = (T_a - T_g) / J;
               T.omega(i) = T.omega(i) + dt * domega;
               T.lambda(i) = T.omega(i) * R / T.u(i);

               T.omega(i) = max(0.0001, T.omega(i) + dt * domega); 
               T.lambda(i) = T.omega(i) * R / T.u(i);
               T.Cp(i) = max(0.01, interp2(beta_vec, lambda_vec, Cp_matrix, T.beta(i), T.lambda(i)));
               T.Ct(i) = max(0.1,interp2(beta_vec, lambda_vec, Ct_matrix, T.beta(i), T.lambda(i)));

               if T.Cp(i) == 0.01 & T.beta(i) == T.beta_desired(i);
                    T.P2S(i) = false;
                    T.currentstate(i) = 'Stalled';
               else
                    T.currentstate(i) = 'Pitch_to_stall';
               end

          case 'Stalled'
               T.Cp(i) = 0.01;
               T.Ct(i) = 0.1;
               T.omega(i) = 0.001;
               T.beta(i) = 45;

               if T.startup(i) == true
                    T.integral_error(i) = 0;
                    T.beta_greedy(i) = interp1(windspd_vec, beta_vec_ws, T.u(i));
                    T.omega_greedy(i) = interp1(windspd_vec, omega_vec_ws, T.u(i)) * 2*pi / 60;

                    T.beta_desired(i) = T.beta_greedy(i);
                    T.omega_desired(i) = T.omega_greedy(i);

                    T.currentstate(i) = 'Starting_up';
               else
                    T.currentstate(i) = 'Stalled';
               end

          case 'Starting_up'
               Kp = 3e8;
               Ki = 8e6;
               pitch_rate = 0.75;

               beta_error = T.beta_desired(i) - T.beta(i);
               beta_step = sign(beta_error) * min(abs(beta_error), pitch_rate * dt);
               T.beta(i) = T.beta(i) + beta_step;

               T.error(i) = T.omega(i) - T.omega_desired(i);
               T.integral_error(i) = T.integral_error(i) + T.error(i) * dt;
               T.integral_error(i) = max(-0.1, T.integral_error(i));  % Prevent integral windup

               T_g = Kp * T.error(i) + Ki * T.integral_error(i);
               T_g = max(0, T_g);  
               T.Cq = T.Ct(i) / T.lambda(i);
               T_a = 0.5 * rho * A * T.Cq * (T.u(i)^3);
               domega = (T_a - T_g) / J;

               T.omega(i) = T.omega(i) + dt * domega;  
               T.lambda(i) = T.omega(i) * R / T.u(i);
          
               T.Cp(i) = max(0.01, interp2(beta_vec, lambda_vec, Cp_matrix, T.beta(i), T.lambda(i)));
               T.Ct(i) = max(0.1, interp2(beta_vec, lambda_vec, Ct_matrix, T.beta(i), T.lambda(i)));
               if T.omega(i) == T.omega_desired(i)
                    T.startup(i) = false;
                    T.currentstate(i) = 'Greedy';
               else
                    T.currentstate(i) = 'Starting_up';
               end
     end
end







