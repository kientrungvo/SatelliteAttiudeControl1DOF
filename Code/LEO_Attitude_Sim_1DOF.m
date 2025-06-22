%==========================================================================
% Filename    : satellite_attitude_sim_main.m
% Mô tả       : Simulate the attitude of a satellite with 1 DOF
% Author      : Trung-Kien Vo
% Email       : kienvt2712@gmail.com
% Date        : 22/06/2025
% Note        : This is my source code that I written myself
% Reference   : https://charleslabs.fr/en/project-Reaction+Wheel+Attitude+Control
% All the variables of motion model are taken in the reference link.
%==========================================================================

clc; clear; close all;
satellite_attitude_simulation();

function satellite_attitude_simulation()
    addpath("tools");

    %% === 1. Generate trajectory and orbit variables ===
    [t, theta_orbit, r_ECI, R_E, mu, R] = initOrbit();

    %% === 2. Generate figure and UI ===
    [~, axes_plot, h_plot, h_text, panel_handles] = initFigureUI(r_ECI, R_E);

    %% === 3. Generate 1DOF motion model and sensors variables ===
    Magnetic_Field = [0;0;1];
    Magnetic_Error = deg2rad(1);
    Gyro_error = deg2rad(0.1);

    dt = t(2)-t(1);
    I_rw = 1.36e-4; 
    I_sat = 2e-4;
    state.roll_sat = 0;
    state.omega_sat = 0;
    state.omega_rw = 0;
    
    rpm_arr = zeros(1, length(t));
    roll_meas = zeros(1, length(t));
    roll_err_prev = 0;
    i_roll_err = 0;
    
    rpm_max_rw = 100;      % Max rpm of reaction wheel
    delta_rpm_max_rw = 30; % Max rpm rate of reaction wheel

    %% === 4. Simulating with time ===
    for i = 1:length(t)
        % Read PID from UI
        [Kp, Ki, Kd, roll_desire] = readUI(panel_handles);

        % PID control
        roll_err_now = wrapToPi(roll_meas(i) - roll_desire);
        d_roll_err = (roll_err_now - roll_err_prev)/dt;
        i_roll_err = i_roll_err + roll_err_now*dt;
        roll_err_prev = roll_err_now;
        
        rpm_gain = PIDController(Kp, Kd, Ki, roll_err_now, d_roll_err, i_roll_err);
        
        % We have to consider how to control the satellite when the max rpm
        % of the reation wheel is maximum. In this case, the satellite may
        % not generate the torque

        if (state(i).omega_rw >= rpm_max_rw*(2*pi / 60))
            if (rpm_gain > 0)
                rpm_arr(i+1) = 0;
            end
        elseif (state(i).omega_rw <= -rpm_max_rw*((2*pi / 60)))
            if (rpm_gain < 0)
                rpm_arr(i+1) = 0;
            end
        else 
            rpm_arr(i+1) = rpm_arr(i) + sign(rpm_gain)*min(abs(rpm_gain), delta_rpm_max_rw);
        end
        
        if (abs(rpm_arr(i+1)) > rpm_max_rw)
            rpm_arr(i+1) = sign(rpm_arr(i+1))*rpm_max_rw;
        end

        % Simulating the system
        state(i+1) = ReactionWheel1DOF(state(i), rpm_arr(i), rpm_arr(i+1), I_sat, I_rw, dt, Gyro_error);
        Rz = rotz(theta_orbit(i));
        Rx = rotx(state(i+1).roll_sat);
        R_total = Rz * Rx;

        % Update figure
        T = eye(4);
        T(1:3,1:3) = R_total;
        T(1:3,4) = r_ECI(:,i);
        set(axes_plot.transform, 'Matrix', T);

        % Read sensors
        [roll_meas(i+1),B_body] = Magnetometer(R, r_ECI(:,i), Magnetic_Field, state(i+1).roll_sat, Magnetic_Error);
        accel_meas = Accelerometer(r_ECI(:,i), R_total, mu, Gyro_error);

        % Draw figure
        updatePlots(h_plot, t(i), state(i+1), roll_meas(i+1), rpm_arr(i+1), B_body, accel_meas);
        updateInfoText(h_text, t(i), state(i+1), roll_meas(i+1), rpm_arr(i+1), Kp, Ki, Kd, B_body, accel_meas, rpm_gain);

        drawnow;
    end
end

function [t, theta_orbit, r_ECI, R_E, mu, R] = initOrbit()
    mu = 3.986e14; R_E = 6371e3; h = 500e3; R = R_E + h;
    omega_orbit = sqrt(mu / R^3);
    T_orbit = 2*pi / omega_orbit;
    dt = 0.1;
    t = 0:dt:T_orbit;
    theta_orbit = omega_orbit * t;
    r_ECI = R * [cos(theta_orbit); sin(theta_orbit); zeros(1,length(t))];
end

function [fig, axes_plot, h_plot, h_text, panel] = initFigureUI(r_ECI, R_E)
    fig = figure('Color','w','Position',[100 100 1400 600]);

    % Subplot satellite
    subplot(2,4,1);
    axis equal; grid on; hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Vệ tinh quay quanh Trái Đất (3D)');
    view(35,20);
    [xe, ye, ze] = sphere(50);
    surf(R_E*xe, R_E*ye, R_E*ze, 'FaceAlpha', 0.3, 'EdgeColor','none', 'FaceColor',[0.2 0.6 1]);
    plot3(r_ECI(1,:), r_ECI(2,:), r_ECI(3,:), 'r--');
    axes_plot.transform = hgtransform;
    drawSatellite(axes_plot.transform);
    
    % ECI axis
    L = 40e5;
    quiver3(0,0,0,L,0,0,'r','LineWidth',2);
    quiver3(0,0,0,0,L,0,'g','LineWidth',2);
    quiver3(0,0,0,0,0,L,'b','LineWidth',2);

    subplot(2,4,2); grid on;
    h_plot.roll = animatedline('Color','b','LineWidth',1.5);
    h_plot.roll_meas = animatedline('Color','r','LineStyle','--');
    title('Góc roll (deg)'); ylabel('Roll'); xlabel('t (s)');
    legend('Real','Measured');

    subplot(2,4,3); grid on;
    h_plot.omega = animatedline('Color','k','LineWidth',1.5);
    title('Tốc độ quay vệ tinh (rad/s)'); ylabel('\omega (rad/s)'); xlabel('t (s)');

    subplot(2,4,4); grid on;
    h_plot.rpm = animatedline('Color','m','LineWidth',1.5);
    title('Tốc độ bánh xe phản lực (rpm)'); ylabel('RPM'); xlabel('t (s)');
    
    subplot(2,4,5); grid on;
    h_plot.magX = animatedline('Color','r','LineWidth',1.5);
    h_plot.magY = animatedline('Color','g','LineWidth',1.5);
    h_plot.magZ = animatedline('Color','b','LineWidth',1.5);
    title('Cảm biến từ trường đã chuẩn hóa'); ylabel('Magnetude'); xlabel('t (s)');
    legend('MagX','MagY','MagZ');
    
    subplot(2,4,6); grid on;
    h_plot.AccX = animatedline('Color','r','LineWidth',1.5);
    h_plot.AccY = animatedline('Color','g','LineWidth',1.5);
    h_plot.AccZ = animatedline('Color','b','LineWidth',1.5);
    title('Cảm biến gyro (m/s^2)'); ylabel('Magnetude'); xlabel('t (s)');
    legend('AccX','AccY','AccZ');

    subplot(2,4,7); axis off;
    h_text = text(0, 1, '', 'FontSize', 10, 'VerticalAlignment','top');

    panel = initUIPanel();
end

function panel = initUIPanel()
    panel.uip = uipanel('Title','PID & roll desire','FontSize',10,'BackgroundColor','white','Position',[0.75 0.25 0.1 0.25]);
    % Kp
    uicontrol('Parent',panel.uip,'Style','text','String','Kp','Units','normalized','Position',[0.05 0.85 0.2 0.1]);
    panel.Kp = uicontrol('Parent',panel.uip,'Style','edit','String','5','Units','normalized','Position',[0.3 0.85 0.6 0.1]);
    % Ki
    uicontrol('Parent',panel.uip,'Style','text','String','Ki','Units','normalized','Position',[0.05 0.7 0.2 0.1]);
    panel.Ki = uicontrol('Parent',panel.uip,'Style','edit','String','0.01','Units','normalized','Position',[0.3 0.7 0.6 0.1]);
    % Kd
    uicontrol('Parent',panel.uip,'Style','text','String','Kd','Units','normalized','Position',[0.05 0.55 0.2 0.1]);
    panel.Kd = uicontrol('Parent',panel.uip,'Style','edit','String','2','Units','normalized','Position',[0.3 0.55 0.6 0.1]);
    % Roll
    uicontrol('Parent',panel.uip,'Style','text','String','Roll (deg)','Units','normalized','Position',[0.05 0.35 0.2 0.1]);
    panel.roll = uicontrol('Parent',panel.uip,'Style','edit','String','-90','Units','normalized','Position',[0.3 0.35 0.6 0.1]);
end

function [Kp, Ki, Kd, roll_desire] = readUI(panel)
    Kp = str2double(get(panel.Kp,'String'));
    Ki = str2double(get(panel.Ki,'String'));
    Kd = str2double(get(panel.Kd,'String'));
    roll_desire = deg2rad(str2double(get(panel.roll,'String')));
end

function updatePlots(h_plot, t_now, state, theta_meas, rpm, B_body, accel_meas)
    addpoints(h_plot.roll, t_now, rad2deg(state.roll_sat));
    addpoints(h_plot.roll_meas, t_now, rad2deg(theta_meas));
    addpoints(h_plot.omega, t_now, state.omega_sat);
    addpoints(h_plot.rpm, t_now, rpm);
    addpoints(h_plot.magX, t_now, B_body(1)/norm(B_body));
    addpoints(h_plot.magY, t_now, B_body(2)/norm(B_body));
    addpoints(h_plot.magZ, t_now, B_body(3)/norm(B_body));
    addpoints(h_plot.AccX, t_now, accel_meas(1));
    addpoints(h_plot.AccY, t_now, accel_meas(2));
    addpoints(h_plot.AccZ, t_now, accel_meas(3));
end

function updateInfoText(h_text, t_now, state, roll_meas, rpm, Kp, Ki, Kd, B_body, accel_meas, rpm_gain)
    info_str = {
        sprintf('Thời gian: %.1f s', t_now);
        sprintf('Roll thực: %.2f deg', rad2deg(state.roll_sat));
        sprintf('Roll đo:   %.2f deg', rad2deg(roll_meas));
        sprintf('Tốc độ vệ tinh: %.4f rad/s', state.omega_sat);
        sprintf('RPM bánh xe: %.2f', rpm);
        sprintf('Magnetometer: X=%.2f | Y =%.2f | Z=%.2f', B_body(1)/norm(B_body), B_body(2)/norm(B_body), B_body(3)/norm(B_body));
        sprintf('Accelerometer: X=%.2f | Y =%.2f | Z=%.2f', accel_meas(1), accel_meas(2), accel_meas(3));
        sprintf('Kp=%.2f | Ki=%.2f | Kd=%.2f', Kp, Ki, Kd);
        sprintf('Tín hiệu điều khiển (rpm gain): %.2f', rpm_gain);
        };
    set(h_text, 'String', info_str);
end

function drawSatellite(parent)
    % Box satellite and reaction wheel
    Lx = 10e5; Ly = 8e5; Lz = 20e5;
    [x, y, z] = ndgrid([0 1], [0 1], [0 1]);
    x = (x - 0.5) * Lx;
    y = (y - 0.5) * Ly;
    z = (z - 0.5) * Lz;
    fv_box = surf2patch(x(:,:,1), y(:,:,1), z(:,:,1), 'triangles');
    patch(fv_box, 'FaceColor', [0.7 0.7 0.7], 'FaceAlpha', 0.9, 'EdgeColor', 'none', 'Parent', parent);

    % Reaction wheel: cylinder
    [Rc, H] = deal(12e5, 2e5);
    [xc, yc, zc] = cylinder(Rc, 50);
    zc = H * (zc - 0.5);
    temp = xc; xc = zc; zc = yc; yc = temp;
    xc = xc + Lx/2 + H/2;
    surface(xc, yc, zc, 'FaceColor', [1 0.4 0], 'EdgeColor', 'none', 'FaceAlpha', 0.9, 'Parent', parent);

    % XYZ body frame of the 1 DOFsatellite
    L = 40e5;
    quiver3(0,0,0,L,0,0,'r','LineWidth',2,'Parent',parent);
    quiver3(0,0,0,0,L,0,'g','LineWidth',2,'Parent',parent);
    quiver3(0,0,0,0,0,L,'b','LineWidth',2,'Parent',parent);
end

function R = rotx(theta)
    R = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
end

function R = rotz(theta)
    R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
end
