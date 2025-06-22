function [roll,B_body] = Magnetometer(R_E, r_ECI, Magnetic_Field, theta_roll, error)
    B_eci = getEarthMagneticField(R_E, r_ECI, Magnetic_Field);
    B_body = rotateECItoBody(B_eci, theta_roll);
    roll = atan2(B_body(2), B_body(3));
    roll = roll + error*rand();
end

function B_eci = getEarthMagneticField(R_E, r_ECI, Magnetic_Field)
    B0 = 3.12e-5;
    r_norm = norm(r_ECI);
    B_eci = B0 * (R_E / r_norm)^3 * Magnetic_Field;
end

function B_body = rotateECItoBody(B_eci, theta_roll)
    % Only consider rotation around the X-body axis (1 DOF), equivalent to rotation around the X-ECI
    % => use the rotation matrix around X axis
    Rx = [1, 0, 0;
          0, cos(theta_roll), sin(theta_roll);
          0, -sin(theta_roll),  cos(theta_roll)];

    B_body = Rx * B_eci;
end