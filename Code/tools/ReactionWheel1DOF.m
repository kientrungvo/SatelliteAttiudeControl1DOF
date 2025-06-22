function state = ReactionWheel1DOF(state, rpm_prev, rpm_now, I_sat, I_rw, dt, gyro_error)
% Motion model of satellite and reaction wheel in 1 DOF (around X axis)
% From the conservation of angular momentum
% I_rw * omega_rw + I_sat * omega_sat = const
% Derivative both side by 0 gives: 
% => I_rw * alpha_rw + I_sat * alpha_sat = 0;

% Input:
%   state   : the structure including:
%               .theta_sat (rad)
%               .omega_sat (rad/s)
%               .omega_rw  (rad/s)
%   rpm_prev: the rw round per min of previous step
%   rpm_now: the rw round per min of current step
%   tau_rw  : the torque acting on the wheel (Nm)
%   I_sat   : moment of inertia of satellite (kg.m^2)
%   I_rw    : moment of inertia of reaction wheel (kg.m^2)
%   dt      : timestep (s)
%
% Output:
%   state   : the update state after applied input
    
    omega_prev = rpm_prev * 2*pi / 60;
    omega_now  = rpm_now  * 2*pi / 60;
    % angular acceleration of the wheel
    alpha_rw = (omega_now - omega_prev) / dt;

    tau_rw = I_rw * alpha_rw;

    % angular acceleration of satellite
    alpha_sat = -tau_rw / I_sat;

    % Update angular velocity
    state.omega_rw = state.omega_rw + alpha_rw * dt;
    state.omega_sat = state.omega_sat + alpha_sat * dt;
    omega_gyro = Gyroscope(state.omega_sat, gyro_error);
    
    % Update rotation angle (roll)
    state.roll_sat = state.roll_sat + omega_gyro * dt;
    state.roll_sat = wrapToPi(state.roll_sat);
end