% drive matrix (linear part)
% Input:Start matrix(A),End matrix(B),Percentage between A&B(0 <= r <= 1)
% Output:Drive matrix (D_r=[n_r o_r a_r p_r])
function D_r =  LinearDrive(A,B,r)
nA=[A(1,1);A(2,1);A(3,1)]; oA=[A(1,2);A(2,2);A(3,2)]; aA=[A(1,3);A(2,3);A(3,3)]; pA=[A(1,4);A(2,4);A(3,4)];
nB=[B(1,1);B(2,1);B(3,1)]; oB=[B(1,2);B(2,2);B(3,2)]; aB=[B(1,3);B(2,3);B(3,3)]; pB=[B(1,4);B(2,4);B(3,4)];
% input parameter
x = dot( nA , (pB - pA) ); y = dot( oA , (pB - pA) ); z = dot( aA , (pB - pA) ); psi = atan2( dot(oA,aB) , dot(nA,aB) );
theta = atan2( sqrt( dot(oA,aB)^2 + dot(nA,aB)^2 ) , dot(aA,aB) );
Vtheta = 1-cos(theta);
sin_phi = -sin(psi)*cos(psi)*Vtheta*dot(nA,nB)+...
    ( (cos(psi)^2)*Vtheta+cos(theta) )*dot(oA,nB)-sin(psi)*sin(theta)*dot(aA,nB);
cos_phi = -sin(psi)*cos(psi)*Vtheta*dot(nA,oB)+...
    ( (cos(psi)^2)*Vtheta+cos(theta) )*dot(oA,oB)-sin(psi)*sin(theta)*dot(aA,oB);
phi = atan2(sin_phi,cos_phi);

% fixed PSI
rx=x*r;  ry=y*r;  rz=z*r;  rtheta=theta*r;  rphi=phi*r;   

S_psi=sin(psi);  C_psi=cos(psi);  S_rtheta=sin(rtheta);  C_rtheta=cos(rtheta);  
V_rtheta=1-C_rtheta;  S_rphi=sin(rphi);  C_rphi=cos(rphi);

% compute D_r (D_r = T_r*Ra_r*Ro_r)
T_r = [1 0 0 rx;    0 1 0 ry;      0 0 1 rz;      0 0 0 1];
Ra_r = [(S_psi^2)*V_rtheta+C_rtheta,       -S_psi*C_psi*V_rtheta, C_psi*S_rtheta, 0;
              -S_psi*C_psi*V_rtheta, (C_psi^2)*V_rtheta+C_rtheta, S_psi*S_rtheta, 0;
                    -C_psi*S_rtheta,             -S_psi*S_rtheta,       C_rtheta, 0;
                                  0,                           0,              0, 1];
Ro_r = [C_rphi, -S_rphi, 0, 0;  S_rphi,  C_rphi, 0, 0;    0,   0, 1, 0;    0,   0, 0, 1];         
D_r = T_r*Ra_r*Ro_r;
end
