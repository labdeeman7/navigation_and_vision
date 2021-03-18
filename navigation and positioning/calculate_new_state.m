function x = calculate_new_state(x, H, dz)
    x = x + inv(H'*H)*H' * dz;
end