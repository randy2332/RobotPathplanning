
function D_r = TransitionDrive(A,B,C,r)
% input parameter
nA=[A(1,1);A(2,1);A(3,1)];  oA=[A(1,2);A(2,2);A(3,2)];
aA=[A(1,3);A(2,3);A(3,3)];  pA=[A(1,4);A(2,4);A(3,4)];
nB=[B(1,1);B(2,1);B(3,1)];  oB=[B(1,2);B(2,2);B(3,2)];
aB=[B(1,3);B(2,3);B(3,3)];  pB=[B(1,4);B(2,4);B(3,4)];
nC=[C(1,1);C(2,1);C(3,1)];  oC=[C(1,2);C(2,2);C(3,2)];
aC=[C(1,3);C(2,3);C(3,3)];  pC=[C(1,4);C(2,4);C(3,4)];
x1 = dot( nB , (pA - pB) );  y1 = dot( oB , (pA - pB) );
z1 = dot( aB , (pA - pB) );  psi1 = atan2( dot(oB,aA) , dot(nB,aA) );
theta1 = atan2( sqrt( dot(oB,aA)^2 + dot(nB,aA)^2 ) , dot(aB,aA) );
Vtheta1 = 1-cos(theta1);
sin_phi1 = -sin(psi1)*cos(psi1)*Vtheta1*dot(nB,nA)+...
    ( (cos(psi1)^2)*Vtheta1+cos(theta1) )*dot(oB,nA)-sin(psi1)*sin(theta1)*dot(aB,nA);
cos_phi1 = -sin(psi1)*cos(psi1)*Vtheta1*dot(nB,oA)+...
    ( (cos(psi1)^2)*Vtheta1+cos(theta1) )*dot(oB,oA)-sin(psi1)*sin(theta1)*dot(aB,oA);
phi1 = atan2( sin_phi1 , cos_phi1 );
% input parameter
x2 = dot( nB , (pC - pB) );  y2 = dot( oB , (pC - pB) );  z2 = dot( aB , (pC - pB) );
psi2 = atan2( dot(oB,aC) , dot(nB,aC) );
theta2 = atan2( sqrt( dot(oB,aC)^2 + dot(nB,aC)^2 ) , dot(aB,aC) );
Vtheta2 = 1-cos(theta2);
sin_phi2 = -sin(psi2)*cos(psi2)*Vtheta2*dot(nB,nC)+...
    ( (cos(psi2)^2)*Vtheta2+cos(theta2) )*dot(oB,nC)-sin(psi2)*sin(theta2)*dot(aB,nC);
cos_phi2 = -sin(psi2)*cos(psi2)*Vtheta2*dot(nB,oC)+...
    ( (cos(psi2)^2)*Vtheta2+cos(theta2) )*dot(oB,oC)-sin(psi2)*sin(theta2)*dot(aB,oC);
phi2 = atan2( sin_phi2 , cos_phi2 );
if abs(psi2-psi1)>pi/2   % of psi,phi)-> let |psi2-psi1| < pi/2
    psi1=psi1+pi;
    theta1=-theta1;
end
rx=((x2*0.2/0.5+x1)*(2-r)*(r^2)-2*x1)*r+x1;  ry=((y2*0.2/0.5+y1)*(2-r)*(r^2)-2*y1)*r+y1; 
rz=((z2*0.2/0.5+z1)*(2-r)*(r^2)-2*z1)*r+z1;
rpsi=(psi2-psi1)*r+psi1;
rtheta=((theta2*0.2/0.5+theta1)*(2-r)*(r^2)-2*theta1)*r+theta1;
rphi=((phi2*0.2/0.5+phi1)*(2-r)*(r^2)-2*phi1)*r+phi1;
S_rpsi=sin(rpsi);  C_rpsi=cos(rpsi);  S_rtheta=sin(rtheta);  C_rtheta=cos(rtheta);
V_rtheta=1-C_rtheta;  S_rphi=sin(rphi);  C_rphi=cos(rphi);
T_r = [1 0 0 rx;        0 1 0 ry;        0 0 1 rz;      0 0 0 1];% compute D_r (D_r = T_r*Ra_r*Ro_r)
Ra_r = [(S_rpsi^2)*V_rtheta+C_rtheta,      -S_rpsi*C_rpsi*V_rtheta, C_rpsi*S_rtheta, 0;
             -S_rpsi*C_rpsi*V_rtheta, (C_rpsi^2)*V_rtheta+C_rtheta, S_rpsi*S_rtheta, 0;
                    -C_rpsi*S_rtheta,             -S_rpsi*S_rtheta,        C_rtheta, 0;
                                   0,                            0,               0, 1];
Ro_r = [C_rphi, -S_rphi, 0, 0;   S_rphi,  C_rphi, 0, 0;    0,       0, 1, 0;     0,   0, 0, 1];
D_r = T_r*Ra_r*Ro_r;   
end

