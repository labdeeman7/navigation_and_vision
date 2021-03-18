function C_Ie = get_sagnac_mat(r)
    Define_Constants;
    C_Ie = eye(3);
    C_Ie(1,2) = omega_ie * r/c;
    C_Ie(2,1) = -C_Ie(1,2);
end