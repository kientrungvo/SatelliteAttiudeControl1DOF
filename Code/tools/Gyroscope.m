function omega_gyro = Gyroscope(omega_true, noise_std)
    noise = noise_std * randn;
    omega_gyro = omega_true + noise;
end
