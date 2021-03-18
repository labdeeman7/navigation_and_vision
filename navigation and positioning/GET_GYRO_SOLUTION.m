function result = GET_GYRO_SOLUTION(DR_data, tau)
    rad = pi/180;
    deg = 180/pi;

    gyro = DR_data(:,6);
    compass = DR_data(:,7)*rad;
    
    heading = compass(1);
    result = zeros(size(gyro));
    
    for i = 1:size(gyro,1)
        heading = heading + gyro(i) * tau;
        result(i) = heading;
    end
end