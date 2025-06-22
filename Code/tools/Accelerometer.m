function accel_meas = Accelerometer(r_ECI, R_ECI_to_body, mu, noise_std)
    r_norm = norm(r_ECI);
    g_ECI = -mu / r_norm^3 * r_ECI;

    g_body = R_ECI_to_body * g_ECI;

    accel_meas = g_body + noise_std * randn(3,1);
end