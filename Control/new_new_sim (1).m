function new_new_sim()
    clear; clc; close all;

    fprintf('=== 3DOF Rocket Simulator (Hybrid TVC + Fins) ===\n\n');

    % Create rocket configuration
    rocket = createRocket();

    % Create controller parameters
    controller = createController();

    % Create environment
    env = createEnvironment();

    % Simulation parameters
    sim_params = struct();
    sim_params.dt = 0.05;
    sim_params.max_time = 45;
    sim_params.controller_dt = 0.05;
    sim_params.launch_time = 0.5;

    % Run simulation
    fprintf('Starting simulation...\n');
    tic;
    results = runSimulation(rocket, controller, env, sim_params);
    elapsed = toc;

    % Plot results
    fprintf('Simulation complete in %.2f seconds!\n', elapsed);
    fprintf('Plotting results...\n');
    plotResults(results, rocket);

    fprintf('\n=== SIMULATION SUMMARY ===\n');
    fprintf('Max altitude: %.2f m\n', max(results.altitude));
    fprintf('Max velocity: %.2f m/s\n', max(results.velocity));
    fprintf('Flight time: %.2f s\n', results.time(end));
    fprintf('Max angle deviation: %.2f deg\n', max(abs(results.theta_deg)));
end

%% ========================================================================
%  ROCKET CONFIGURATION
%% ========================================================================
function rocket = createRocket()
    rocket = struct();

    % Mass
    rocket.mass_dry = 2.0;
    rocket.mass_propellant = 0.5;
    rocket.mass_wet = 2.5;
    rocket.Iy_dry = 0.8;
    rocket.Iy_wet = 1.0;

    % Geometry
    rocket.length = 1.2;
    rocket.diameter = 0.1;
    rocket.reference_area = pi * (rocket.diameter/2)^2;
    rocket.nose_length = 0.25;
    rocket.nose_type = 'conical';

    % CG
    rocket.cg_wet = 0.65;
    rocket.cg_dry = 0.60;

    % Fins
    rocket.num_fins = 4;
    rocket.fin_root_chord = 0.15;
    rocket.fin_tip_chord = 0.08;
    rocket.fin_span = 0.10;
    rocket.fin_sweep = 0.05;
    rocket.fin_root_position = 0.95;

    % Fin control properties
    rocket.fin_control_position = rocket.fin_root_position + rocket.fin_root_chord;
    rocket.fin_max_deflection = 15; % deg
    rocket.fin_effectiveness = 2.0; % control coefficient
    rocket.fin_area = rocket.num_fins * rocket.fin_span * ((rocket.fin_root_chord + rocket.fin_tip_chord)/2);

    % TVC
    rocket.tvc_position = 1.15;
    rocket.tvc_max_angle = 20;

    % Actuator
    rocket.actuator_max_speed = 400; % deg/s
    rocket.actuator_resolution = 0.5; % deg

    % Motor
    rocket.motor = createMotor();

    % Aerodynamics
    rocket = calculateAero(rocket);

    % Static margin
    stability_margin = (rocket.CP - rocket.cg_wet) / rocket.diameter;
    fprintf('Static margin at launch = %.2f calibers\n', stability_margin);

    fprintf('Rocket created:\n');
    fprintf('  Mass: %.2f kg\n', rocket.mass_wet);
    fprintf('  CP: %.3f m, CG: %.3f m\n', rocket.CP, rocket.cg_wet);
    fprintf('  Static margin: %.2f cal\n', (rocket.CP - rocket.cg_wet)/rocket.diameter);
end

function motor = createMotor()
    motor = struct();
    motor.name = 'H128_long20s';
    motor.burn_time = 20.0;
    motor.propellant_mass = 1.0;
    motor.time = [0, 0.5, 2, 5, 10, 15, 18, 19.5, 20, 20.5];
    motor.thrust = [50, 50, 50, 50, 50, 50, 50, 50, 0, 0]; % N
end

function thrust = getThrust(motor, t)
    if t < 0 || t > motor.time(end)
        thrust = 0;
    else
        thrust = interp1(motor.time, motor.thrust, t, 'linear');
    end
end

function m_prop = getPropellantMass(motor, t)
    if t < 0
        m_prop = motor.propellant_mass;
    elseif t >= motor.burn_time
        m_prop = 0;
    else
        m_prop = motor.propellant_mass * (1 - t/motor.burn_time);
    end
end

%% ========================================================================
%  AERODYNAMICS
%% ========================================================================
function rocket = calculateAero(rocket)
    CNa_nose = 2.0;
    CP_nose = 0.666 * rocket.nose_length;

    n = rocket.num_fins;
    Cr = rocket.fin_root_chord;
    Ct = rocket.fin_tip_chord;
    s = rocket.fin_span;
    Lf = rocket.fin_sweep;
    d = rocket.diameter;

    Lm = Lf + 0.5 * (Cr - Ct);
    K_fb = 1 + (d/2) / (s + d/2);
    CNa_fins = K_fb * (4 * n * (s/d)^2) / (1 + sqrt(1 + (2*Lm/(Cr+Ct))^2));

    X = rocket.fin_root_position;
    CP_fins = X + Lm + (Cr - Ct) * (Cr + 2*Ct) / (3 * (Cr + Ct));

    rocket.CNa = CNa_nose + CNa_fins;
    rocket.CP = (CNa_nose * CP_nose + CNa_fins * CP_fins) / rocket.CNa;
    rocket.Cd0 = 0.35;
end

function [Cd, CNa] = getAeroCoeffs(rocket, alpha)
    Cd = rocket.Cd0 + 1.5 * rocket.CNa * alpha^2;
    CNa = rocket.CNa;
end

%% ========================================================================
%  ENVIRONMENT
%% ========================================================================
function env = createEnvironment()
    env = struct();
    env.g = 9.81;
    env.T0 = 288.15;
    env.P0 = 101325;
    env.rho0 = 1.225;
end

function [rho, T, P] = getAtmosphere(env, h)
    T = env.T0 - 0.0065 * h;
    if T < 216.65
        T = 216.65;
    end
    P = env.P0 * (T / env.T0)^5.255;
    rho = P / (287.05 * T);
end

%% ========================================================================
%  CONTROLLER
%% ========================================================================
function controller = createController()
    controller = struct();
    controller.Kp = 1.0;
    controller.Ki = 0.1;
    controller.Kd = 1.0;
    controller.K_all = 1.0;
    controller.K_damping = 0.5;
    controller.target_angle = 0;
    controller.reference_thrust = 60;

    fprintf('Controller: Kp=%.2f, Ki=%.2f, Kd=%.2f\n', ...
        controller.Kp, controller.Ki, controller.Kd);
end

%% ========================================================================
%  DYNAMICS (Hybrid TVC + Fins)
%% ========================================================================
function dState = rocketDynamics(t, state, rocket, controller, env, sim_params)
    x = state(1);
    z = state(2);
    vx = state(3);
    vz = state(4);
    theta = state(5);
    omega = state(6);
    actuator_angle = state(7); % radians
    int_error = state(8);

    if t < sim_params.launch_time
        dState = zeros(8,1);
        return;
    end

    t_motor = t - sim_params.launch_time;
    m_prop = getPropellantMass(rocket.motor, t_motor);
    mass = rocket.mass_dry + m_prop;
    frac = m_prop / rocket.motor.propellant_mass;
    Iy = rocket.Iy_dry + (rocket.Iy_wet - rocket.Iy_dry) * frac;
    cg = rocket.cg_dry + (rocket.cg_wet - rocket.cg_dry) * frac;

    thrust = getThrust(rocket.motor, t_motor);
    [rho, ~, ~] = getAtmosphere(env, max(z,0));

    V = sqrt(vx^2 + vz^2);
    if V < 0.1, V = 0.1; end
    vel_angle = atan2(vx, vz);
    alpha = vel_angle - theta;

    [Cd, CNa] = getAeroCoeffs(rocket, alpha);
    q = 0.5 * rho * V^2;

    % --- PID Controller ---
    error_angle = deg2rad(controller.target_angle) - theta;
    error_rate = -omega;
    int_error_limited = max(min(int_error,50),-50);

    P = controller.Kp * error_angle;
    I = controller.Ki * int_error_limited;
    D = controller.Kd * error_rate;

    base_cmd = P + I + D;

    % --- Blending Factor ---
    q_blend = min(q / 500, 1);  % 0=low speed -> TVC; 1=high speed -> fins

    % --- TVC Command ---
    if thrust > 10
        thrust_factor = controller.reference_thrust / thrust;
    else
        thrust_factor = 1;
    end
    tvc_cmd = base_cmd * (1 - q_blend) * thrust_factor;
    tvc_cmd = max(min(tvc_cmd, deg2rad(rocket.tvc_max_angle)), -deg2rad(rocket.tvc_max_angle));

    % --- Fin Command ---
    fin_cmd = base_cmd * q_blend;
    fin_cmd = max(min(fin_cmd, deg2rad(rocket.fin_max_deflection)), -deg2rad(rocket.fin_max_deflection));

    % --- Forces ---
    T_axial = thrust*cos(tvc_cmd);
    T_normal = thrust*sin(tvc_cmd);
    Ft_x = T_axial*sin(theta) + T_normal*cos(theta);
    Ft_z = T_axial*cos(theta) - T_normal*sin(theta);

    drag = q*Cd*rocket.reference_area;
    normal = q*CNa*rocket.reference_area*alpha;

    Fd_x = -drag*(vx/V);
    Fd_z = -drag*(vz/V);
    Fn_x = normal*cos(vel_angle);
    Fn_z = -normal*sin(vel_angle);
    Fg_x = 0;
    Fg_z = -mass*env.g;

    % --- Fin Forces & Moments ---
    fin_normal = q * rocket.fin_effectiveness * fin_cmd;
    M_fin = (rocket.fin_control_position - cg) * fin_normal;
    Fn_fin_x = fin_normal*cos(vel_angle);
    Fn_fin_z = -fin_normal*sin(vel_angle);

    % --- Total Moment ---
    M_aero = (rocket.CP - cg)*normal;
    M_tvc = thrust*sin(tvc_cmd)*(rocket.tvc_position - cg);
    M_total = M_aero + M_tvc + M_fin;

    % --- Acceleration ---
    Fx = Fg_x + Ft_x + Fd_x + Fn_x + Fn_fin_x;
    Fz = Fg_z + Ft_z + Fd_z + Fn_z + Fn_fin_z;
    ax = Fx/mass;
    az = Fz/mass;
    alpha_ang = M_total/Iy;

    dState = zeros(8,1);
    dState(1)=vx;
    dState(2)=vz;
    dState(3)=ax;
    dState(4)=az;
    dState(5)=omega;
    dState(6)=alpha_ang;
    dState(7)=0;        % actuator bypass
    dState(8)=error_angle; % integral
end

%% ========================================================================
%  SIMULATION RUNNER & RESULTS
%% ========================================================================
function results = runSimulation(rocket, controller, env, sim_params)
    state0 = [0;0.1;0;1;deg2rad(5);0;0;0];
    tspan = [0 sim_params.max_time];
    options = odeset('RelTol',1e-3,'AbsTol',1e-4,'Events',@(t,y) groundEvent(t,y),'MaxStep',0.1);

    [t,state] = ode45(@(t,y) rocketDynamics(t,y,rocket,controller,env,sim_params), tspan, state0, options);
    results = processResults(t,state,rocket,sim_params);
end

function [value,isterminal,direction] = groundEvent(~,state)
    value = state(2);
    isterminal = 1;
    direction = -1;
end

function results = processResults(t,state,rocket,sim_params)
    results.time = t;
    results.x = state(:,1);
    results.altitude = state(:,2);
    results.vx = state(:,3);
    results.vz = state(:,4);
    results.theta = state(:,5);
    results.omega = state(:,6);
    results.actuator_angle = state(:,7);
    results.velocity = sqrt(results.vx.^2 + results.vz.^2);
    results.theta_deg = rad2deg(results.theta);
    results.omega_deg = rad2deg(results.omega);
end

%% ========================================================================
%  PLOTTING
%% ========================================================================
function plotResults(results,rocket)
    figure('Position',[100 100 1200 700]);

    subplot(2,2,1);
    plot(results.time,results.altitude,'b-','LineWidth',2); grid on; xlabel('Time (s)'); ylabel('Altitude (m)'); title('Altitude');

    subplot(2,2,2);
    plot(results.time,results.theta_deg,'r-','LineWidth',2); grid on; xlabel('Time (s)'); ylabel('Pitch Angle (deg)'); title('Pitch Angle'); yline(0,'k--','Target');

    subplot(2,2,3);
    plot(results.time,results.omega_deg,'g-','LineWidth',2); grid on; xlabel('Time (s)'); ylabel('Angular Velocity (deg/s)'); title('Angular Velocity');

    subplot(2,2,4);
    plot(results.x,results.altitude,'b-','LineWidth',2); grid on; xlabel('Horizontal (m)'); ylabel('Altitude (m)'); title('Trajectory'); axis equal;
end
