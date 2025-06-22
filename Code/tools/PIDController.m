function rpm_gain = PIDController(Kp, Kd, Ki, angle_err, d_angle_err, i_angle_err) 
    rpm_gain = Kp * angle_err + Kd * d_angle_err + Ki * i_angle_err;
end