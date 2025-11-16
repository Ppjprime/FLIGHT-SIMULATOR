function Cvb = calc_Cvb(phi, theta, psi)


c_phi   = cos(phi);
s_phi   = sin(phi);
c_theta = cos(theta);
s_theta = sin(theta);
c_psi   = cos(psi);
s_psi   = sin(psi);

Cvb = zeros(3,3);

Cvb(1,1) =  c_theta * c_psi;
Cvb(1,2) =  c_theta * s_psi;
Cvb(1,3) = -s_theta;

Cvb(2,1) =  s_phi * s_theta * c_psi - c_phi * s_psi;
Cvb(2,2) =  s_phi * s_theta * s_psi + c_phi * c_psi;
Cvb(2,3) =  s_phi * c_theta;

Cvb(3,1) =  c_phi * s_theta * c_psi + s_phi * s_psi;
Cvb(3,2) =  c_phi * s_theta * s_psi - s_phi * c_psi;
Cvb(3,3) =  c_phi * c_theta;

end
