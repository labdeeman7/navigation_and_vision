function [results, display_table] = CLOSED_LOOP_INT_KF...
    (DR_data, GNSS_KF_data, tau, init_latitude, init_longitude, init_height, PSD)
    
    Define_Constants;

    times = DR_data(:,1);
    avg_speed = sum(DR_data(:,4:5),2) / 2;
    gyro = DR_data(:,6);
    heading = DR_data(:,7)*rad;
    
    GNSS_L = GNSS_KF_data(:,2)*rad;
    GNSS_lamda = GNSS_KF_data(:,3)*rad;
    GNSS_h = GNSS_KF_data(:,4);
    GNSS_vN = GNSS_KF_data(:,5);
    GNSS_vE = GNSS_KF_data(:,6);
    
    latitudes = zeros(size(times));
    longitudes = latitudes;
    velocity_north = latitudes;
    velocity_east = latitudes;
    
    SDGr = 5.2;  % signal in space, ionosphere, troposphere, multipath error
    SDGv = 0.05; % wheel speed sensor noise
    
    %% measurement matrix
    H = zeros(4);
    H(1,3) = -1; H(2,4) = -1; H(3,1) = -1; H(4,2) = -1;
    
    for i = 1:size(times,1)
        if i == 1
            vN = 0.5 * (cos(heading(i)) + cos(heading(i))) * avg_speed(i);
            vE = 0.5 * (sin(heading(i)) + sin(heading(i))) * avg_speed(i);
        else
            vN = 0.5 * (cos(heading(i)) + cos(heading(i-1))) * avg_speed(i);
            vE = 0.5 * (sin(heading(i)) + sin(heading(i-1))) * avg_speed(i);
        end
        
        if i == 1
            [RN, RE] = Radii_of_curvature(init_latitude);
            L = init_latitude + vN*(times(i)-times(i)) / (RN + init_height);
            lamda = init_longitude + vE*(times(i)-times(i)) / ((RE + init_height)*cos(L));
        else
            [RN, RE] = Radii_of_curvature(latitudes(i-1));
            L = latitudes(i-1) + vN*(times(i)-times(i-1)) / (RN + init_height);
            lamda = longitudes(i-1) + vE*(times(i)-times(i-1)) / ((RE + init_height)*cos(L));
        end
        
        if i == 1
            vN = avg_speed(1) * cos(heading(1));
            vE = avg_speed(1) * sin(heading(1));
        else
            vN = 1.7*vN - 0.7*velocity_north(i-1);
            vE = 1.7*vE - 0.7*velocity_east(i-1);
        end
        
        x = zeros(4,1);
        Psi = compute_transition_mat(tau, L, GNSS_h(i));
        Q = compute_sys_noise_covmat(tau, PSD, L, GNSS_h(i));
        if i == 1
            %% Initialise error covariance matrix
            [RN, RE] = Radii_of_curvature(L);
            P = [1; 1; 10^2/(RN+GNSS_h(1))^2; 10^2/((RE+GNSS_h(1))^2*cos(L))];
            P = diag(P);
        else
            P = Psi * P * Psi' + Q;
        end
        R = integrated_measurement_noise_covmat(SDGr, SDGv, L, GNSS_h(i));
        K = P * H' * (H*P*H' + R);
        dz = [GNSS_L(i) - L;
             GNSS_lamda(i) - lamda;
             GNSS_vN(i) - vN;
             GNSS_vE(i) - vE];
        x = x + K*dz;
        
        disp(x);
        
        L = L - x(3);
        lamda = lamda - x(4);
        vN = vN - x(1);
        vE = vE - x(2);
        
        latitudes(i) = L;
        longitudes(i) = lamda;
        velocity_north(i) = vN;
        velocity_east(i) = vE;
    end
    
    results = zeros(size(times,1),5);
    results(:,1) = times;
    results(:,2) = latitudes*deg;
    results(:,3) = longitudes*deg;
    results(:,4) = velocity_north;
    results(:,5) = velocity_east;
    
    header = {'TIME(s)', 'INT-KF LATITUDE(°)', 'INT-KF LONGITUDE(°)', 'INT-KF NORTH(m/s)', 'INT-KF EAST(m/s)'};
    display_table = [header; num2cell(results)];
end