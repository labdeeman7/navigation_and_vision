function [latitudes, longitudes] = get_DR_latitudes_longitudes...
    (initial_latitude, initial_longitude, h, vN, vE, time)

    latitudes = zeros(size(time));
    longitudes = latitudes;
    
    for i = 1:size(time,1)
        if i == 1
            [RN, RE] = Radii_of_curvature(initial_latitude);
            latitudes(i) = initial_latitude + vN(i)*(time(i)-time(i)) / (RN + h);
            longitudes(i) = initial_longitude + vE(i)*(time(i)-time(i)) / ((RE + h)*cos(latitudes(i)));
        else
            [RN, RE] = Radii_of_curvature(latitudes(i-1));
            latitudes(i) = latitudes(i-1) + vN(i)*(time(i)-time(i-1)) / (RN + h);
            longitudes(i) = longitudes(i-1) + vE(i)*(time(i)-time(i-1)) / ((RE + h)*cos(latitudes(i)));
        end
    end
end