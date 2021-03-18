function [velocity_results, t, sat2user_rr, clk_drifts] = GET_ECEF_VELOCITIES(range_data, rangerate_data, v_ea, clk_dft)
    [cartesian_position, innovation_vectors, measurement_matrices, sat2user, LoS_set, sagnac_set]...
        = GET_ECEF_POSITIONS(range_data);

    t = range_data(2:size(range_data,1),1);
    sat = transpose(range_data(1,2:size(range_data,2)));
    rangerate_data = rangerate_data(2:size(rangerate_data),2:size(rangerate_data,2));
    
    sat2user_rr = zeros(size(t,1), size(sat,1));
    velocity_results = zeros(size(t,1), 3);
    clk_drifts = zeros(size(t,1), 1);
    
    if nargin < 3
        v_ea = [0;0;0];
        clk_dft = 0;
    elseif nargin < 4
        clk_dft = 0;
    end
    
    for i = 1:size(t,1)

        sat_cart = zeros(3,size(sat,1));
        sat_vels = zeros(3,size(sat,1));

        for j = 1:size(sat,1)
            [r_ej, v_ej] = Satellite_position_and_velocity(t(i), sat(j));
            r_ej = r_ej';
            v_ej = v_ej';

            sat_cart(:,j) = r_ej';
            sat_vels(:,j) = v_ej';
            r_ea = transpose(cartesian_position(i,:));

            rdot_aj = ...
                get_user2sat_rangerate(...
                r_ea, r_ej, LoS_set(:,j,i), sagnac_set(:,:,j,i), v_ea, v_ej...
                );
            sat2user_rr(i,j) = rdot_aj;        
        end

        x = [v_ea; clk_dft];
        dz = get_meas_innovation_vec(rangerate_data(i,:), sat2user_rr(i,:), clk_dft);
        H = measurement_matrices(:,:,i);

        x = calculate_new_state(x, H, dz);
        v_ea = x(1:3);
        clk_dft = x(4);

        velocity_results(i,:) = transpose(v_ea);
        clk_drifts(i) = clk_dft;
    end
end