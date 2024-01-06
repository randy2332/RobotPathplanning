
function theta = inverse(T)
disp(T);
d_array = [0 0 0.149 0.433 0 0];
a_array = [0 0.432 -0.02 0 0 0];
alpha = [-1/2 0 1/2 -1/2 1/2 0];
fprintf("Inverse Kinematic cal...:\n");
n_o_a_p = T;

nx = n_o_a_p(1,1); ny = n_o_a_p(2,1); nz = n_o_a_p(3,1);
ox = n_o_a_p(1,2); oy = n_o_a_p(2,2); oz = n_o_a_p(3,2);
ax = n_o_a_p(1,3); ay = n_o_a_p(2,3); az = n_o_a_p(3,3);
px = n_o_a_p(1,4); py = n_o_a_p(2,4); pz = n_o_a_p(3,4);


T_in = [nx ox ax px;
        ny oy ay py ;
        nz oz az pz;
        0  0  0  1;] ;


% theta1 have two values 
theta1_1_radian = atan2(py,px)-atan2(d_array(3),sqrt(px^2+py^2-d_array(3)^2 )) ;
theta1_1_degree = rad2deg(theta1_1_radian);
theta1_2_radian = atan2(py,px)-atan2(d_array(3),-sqrt(px^2+py^2-d_array(3)^2 )) ;
theta1_2_degree = rad2deg(theta1_2_radian);

% theta3 also have two values 
M = (px^2+py^2+pz^2-(a_array(2)^2+a_array(3)^2+d_array(3)^2+d_array(4)^2))/(2*a_array(2));
theta3_1_radian = atan2(M,sqrt(a_array(3)^2+d_array(4)^2-M^2)) -atan2(a_array(3),d_array(4));
theta3_1_degree = rad2deg(theta3_1_radian);

theta3_2_radian = atan2(M,-sqrt(a_array(3)^2+d_array(4)^2-M^2)) -atan2(a_array(3),d_array(4));
theta3_2_degree = rad2deg(theta3_2_radian);



% theta2  have four values (because theta2 = theta23 - theta3 )
% case1 theta1_1 theta3_1 
A = cos(theta1_1_radian)*px+sin(theta1_1_radian)*py;
B = pz;
p1 = sqrt(A^2+B^2);
C = a_array(3)+a_array(2)*cos(theta3_1_radian);

theta23_radian = acos(C/p1)-atan2(B,A);
theta23_degree = rad2deg(theta23_radian);

theta2_1_radian = theta23_radian - theta3_1_radian;
theta2_1_degree = rad2deg(theta2_1_radian);

% case2 theta1_2 theta3_1 
A = cos(theta1_2_radian)*px+sin(theta1_2_radian)*py;
B = pz;
p1 = sqrt(A^2+B^2);
C = a_array(3)+a_array(2)*cos(theta3_1_radian);

theta23_radian = acos(C/p1)-atan2(B,A);
theta23_degree = rad2deg(theta23_radian);

theta2_2_radian = theta23_radian - theta3_1_radian;
theta2_2_degree = rad2deg(theta2_2_radian);

% case3 theta1_1 theta3_2 
A = cos(theta1_1_radian)*px+sin(theta1_1_radian)*py;
B = pz;
p1 = sqrt(A^2+B^2);
C = a_array(3)+a_array(2)*cos(theta3_2_radian);

theta23_radian = acos(C/p1)-atan2(B,A);
theta23_degree = rad2deg(theta23_radian);

theta2_3_radian = theta23_radian - theta3_2_radian;
theta2_3_degree = rad2deg(theta2_3_radian);

% case4 theta1_2 theta3_2 
A = cos(theta1_2_radian)*px+sin(theta1_2_radian)*py;
B = pz;
p1 = sqrt(A^2+B^2);
C = a_array(3)+a_array(2)*cos(theta3_2_radian);

theta23_radian = acos(C/p1)-atan2(B,A);
theta23_degree = rad2deg(theta23_radian);

theta2_4_radian = theta23_radian - theta3_2_radian;
theta2_4_degree = rad2deg(theta2_4_radian);

%So far, we have four case 
output_q2_1 = [theta1_1_degree theta2_1_degree theta3_1_degree 0 0 0];
output_q2_2 = [theta1_2_degree theta2_2_degree theta3_1_degree 0 0 0];
output_q2_3 = [theta1_1_degree theta2_3_degree theta3_2_degree 0 0 0];
output_q2_4 = [theta1_2_degree theta2_4_degree theta3_2_degree 0 0 0];

%theta4
%We first compute A1~A3 there are four case
for j =1:4
    if j==1
        output_q2 = output_q2_1;
    elseif j==2
        output_q2 = output_q2_2;
    elseif j==3
        output_q2 = output_q2_3;
    elseif j==4
        output_q2 = output_q2_4;
    end

    for i=1:3
        angle_radian = deg2rad(output_q2(i));% cos and sin function receive randian but not angle
        matrix = [cos(angle_radian), -sin(angle_radian)*cospi(alpha(i)), sin(angle_radian)*sinpi(alpha(i)), a_array(i)*cos(angle_radian); 
                 sin(angle_radian), cos(angle_radian)*cospi(alpha(i)) , -cos(angle_radian)*sinpi(alpha(i)), a_array(i)*sin(angle_radian);
                 0, sinpi(alpha(i)), cospi(alpha(i)) , d_array(i);
                 0, 0, 0, 1];
        if i==1
            A1_q2 = matrix;
        elseif i == 2
            A2_q2 = matrix;
        elseif i == 3
            A3_q2 = matrix;
        end
    end

    if j==1
        T4_6_1 =inv(A3_q2)*inv(A2_q2)*inv(A1_q2)* T_in ; %T is input value
    elseif j==2
        T4_6_2 =inv(A3_q2)*inv(A2_q2)*inv(A1_q2)* T_in ;
    elseif j==3
        T4_6_3 =inv(A3_q2)*inv(A2_q2)*inv(A1_q2)* T_in ;
    elseif j==4
        T4_6_4 =inv(A3_q2)*inv(A2_q2)*inv(A1_q2)* T_in ;
    end

end

%theta 4 have 2 value , thus original_output * num(theta) = 8 
%case 1
theta4_1_radian = atan2(T4_6_1(2,3),T4_6_1(1,3)) ;
if theta4_1_radian>0
    theta4_5_radian = theta4_1_radian-pi; 
else
    theta4_5_radian = theta4_1_radian+pi; 
end
theta4_1_degree = rad2deg(theta4_1_radian);
theta4_5_degree = rad2deg(theta4_5_radian);
output_q2_1(4) = theta4_1_degree;
output_q2_5 = output_q2_1;
output_q2_5(4) = theta4_5_degree;
%case 2
theta4_2_radian = atan2(T4_6_2(2,3),T4_6_2(1,3)) ;
if theta4_2_radian>0
    theta4_6_radian = theta4_2_radian-pi; 
else
    theta4_6_radian = theta4_2_radian+pi; 
end
theta4_2_degree = rad2deg(theta4_2_radian);
theta4_6_degree = rad2deg(theta4_6_radian);
output_q2_2(4) = theta4_2_degree;
output_q2_6 = output_q2_2;
output_q2_6(4) = theta4_6_degree;
%case 3
theta4_3_radian = atan2(T4_6_3(2,3),T4_6_3(1,3)) ;
if theta4_3_radian>0
    theta4_7_radian = theta4_3_radian-pi; 
else
    theta4_7_radian = theta4_3_radian+pi; 
end
theta4_3_degree = rad2deg(theta4_3_radian);
theta4_7_degree = rad2deg(theta4_7_radian);
output_q2_3(4) = theta4_3_degree;
output_q2_7 = output_q2_3;
output_q2_7(4) = theta4_7_degree;
%case 4
theta4_4_radian = atan2(T4_6_4(2,3),T4_6_4(1,3)) ;
if theta4_4_radian>0
    theta4_8_radian = theta4_4_radian-pi; 
else
    theta4_8_radian = theta4_4_radian+pi; 
end
theta4_4_degree = rad2deg(theta4_4_radian);
theta4_8_degree = rad2deg(theta4_8_radian);
output_q2_4(4) = theta4_4_degree;
output_q2_8 = output_q2_4;
output_q2_8(4) = theta4_8_degree;


%theta 5
%compute A5
for j =1:8
    if j==1
        output_q2 = theta4_1_degree;
    elseif j==2
        output_q2 = theta4_2_degree;
    elseif j==3
        output_q2 = theta4_3_degree;
    elseif j==4
        output_q2 = theta4_4_degree;
    elseif j==5
        output_q2 = theta4_5_degree;
    elseif j==6
        output_q2 = theta4_6_degree;
    elseif j==7
        output_q2 = theta4_7_degree;
    elseif j==8
        output_q2 = theta4_8_degree;
    end

    angle_radian = deg2rad(output_q2);% cos and sin function receive randian but not angle
    matrix = [cos(angle_radian), -sin(angle_radian)*cospi(alpha(i)), sin(angle_radian)*sinpi(alpha(i)), a_array(i)*cos(angle_radian); 
             sin(angle_radian), cos(angle_radian)*cospi(alpha(i)) , -cos(angle_radian)*sinpi(alpha(i)), a_array(i)*sin(angle_radian);
             0, sinpi(alpha(i)), cospi(alpha(i)) , d_array(i);
             0, 0, 0, 1];

    if j==1
        T5_6_1 = matrix\T4_6_1 ; %T is input value
    elseif j == 2
        T5_6_2 = matrix\T4_6_2;
    elseif j == 3
        T5_6_3 = matrix\T4_6_3 ;
    elseif j == 4
        T5_6_4 = matrix\T4_6_4;
    elseif j == 5
        T4_6_5 = T4_6_1;
        T5_6_5 = matrix\T4_6_5;
    elseif j == 6
        T4_6_6 = T4_6_2;
        T5_6_6 = matrix\T4_6_6 ;
    elseif j == 7
        T4_6_7 =T4_6_3;
        T5_6_7 = matrix\T4_6_7;
    elseif j == 8
        T4_6_8 = T4_6_4;
        T5_6_8 = matrix\T4_6_8;
    end
end

%case 1
theta5_1_radian = atan2(T5_6_1(1,3),T5_6_1(2,3)) ;
theta5_1_degree = rad2deg(theta5_1_radian);
output_q2_1(5) = theta5_1_degree;
%case 2
theta5_2_radian = atan2(T5_6_2(1,3),T5_6_2(2,3)) ;
theta5_2_degree = rad2deg(theta5_2_radian);
output_q2_2(5) = theta5_2_degree;
%case 3
theta5_3_radian = atan2(T5_6_3(1,3),T5_6_3(2,3)) ;
theta5_3_degree = rad2deg(theta5_3_radian);
output_q2_3(5) = theta5_3_degree;
%case 4
theta5_4_radian = atan2(T5_6_4(1,3),T5_6_4(2,3)) ;
theta5_4_degree = rad2deg(theta5_4_radian);
output_q2_4(5) =  theta5_4_degree;
%case 5
theta5_5_radian = atan2(T5_6_5(1,3),T5_6_5(2,3)) ;
theta5_5_degree = rad2deg(theta5_5_radian);
output_q2_5(5) = theta5_5_degree;
%case 6
theta5_6_radian = atan2(T5_6_6(1,3),T5_6_6(2,3)) ;
theta5_6_degree = rad2deg(theta5_6_radian);
output_q2_6(5) = theta5_6_degree;
%case 7
theta5_7_radian = atan2(T5_6_7(1,3),T5_6_7(2,3)) ;
theta5_7_degree = rad2deg(theta5_7_radian);
output_q2_7(5) = theta5_7_degree;
%case 8
theta5_8_radian = atan2(T5_6_8(1,3),T5_6_8(2,3)) ;
theta5_8_degree = rad2deg(theta5_8_radian);
output_q2_8(5) = theta5_8_degree;


%theta 6
%case 1
theta6_1_radian = atan2(T4_6_1(3,2),-T4_6_1(3,1));
theta6_1_degree = rad2deg(theta6_1_radian);
output_q2_1(6) = theta6_1_degree;
%case 2
theta6_2_radian = atan2(T4_6_2(3,2),-T4_6_2(3,1));
theta6_2_degree = rad2deg(theta6_2_radian);
output_q2_2(6) = theta6_2_degree;
%case 3
theta6_3_radian = atan2(T4_6_3(3,2),-T4_6_3(3,1));
theta6_3_degree = rad2deg(theta6_3_radian);
output_q2_3(6) = theta6_3_degree;
%case 4
theta6_4_radian = atan2(T4_6_4(3,2),-T4_6_4(3,1));
theta6_4_degree = rad2deg(theta6_4_radian);
output_q2_4(6) = theta6_4_degree;

%  solve
%case 5
theta6_5_radian = atan2(T4_6_5(3,2),-T4_6_5(3,1));
if theta6_5_radian>0
    theta6_5_radian = theta6_5_radian-pi; 
else
    theta6_5_radian = theta6_5_radian+pi; 
end
theta6_5_degree = rad2deg(theta6_5_radian);
output_q2_5(6) = theta6_5_degree;
%case 6
theta6_6_radian = atan2(T4_6_6(3,2),-T4_6_6(3,1));
if theta6_6_radian>0
    theta6_6_radian = theta6_6_radian-pi; 
else
    theta6_6_radian = theta6_6_radian+pi; 
end
theta6_6_degree = rad2deg(theta6_6_radian);
output_q2_6(6) = theta6_6_degree;
%case 7
theta6_7_radian = atan2(T4_6_7(3,2),-T4_6_7(3,1));
if theta6_7_radian>0
    theta6_7_radian = theta6_7_radian-pi; 
else
    theta6_7_radian = theta6_7_radian+pi; 
end
theta6_7_degree = rad2deg(theta6_7_radian);
output_q2_7(6) = theta6_7_degree;
%case 8
theta6_8_radian = atan2(T4_6_8(3,2),-T4_6_8(3,1));
if theta6_8_radian>0
    theta6_8_radian = theta6_8_radian-pi; 
else
    theta6_8_radian = theta6_8_radian+pi; 
end
theta6_8_degree = rad2deg(theta6_8_radian);
output_q2_8(6) = theta6_8_degree;


%show result
for i=1:8
    checksafe = 1;% 0 out range ,1 safe
    if i==1
        dis_out =  output_q2_1;
    elseif i==2
        dis_out =  output_q2_2;
    elseif i==3
        dis_out =  output_q2_3;
    elseif i==4
        dis_out =  output_q2_4;
    elseif i==5
        dis_out =  output_q2_5;
    elseif i==6
        dis_out =  output_q2_6;
    elseif i==7
        dis_out =  output_q2_7;
    elseif i==8
        dis_out =  output_q2_8;
    end

    %fprintf("Corresponding variable (theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8) \n");
    
    for j = 1:6
        
        if j==1
            if -160<dis_out(j) &&dis_out(j)<160
                continue
            else
                %fprintf("theta1 is out of range!\n")
                checksafe = 0;
            end
        
        elseif j==2
            if -125<dis_out(j) &&dis_out(j)<125
                continue
            else
                %fprintf("theta2 is out of range!\n")
                checksafe = 0;
            end
        
        elseif j==3
            if -135<dis_out(j) &&dis_out(j)<135
                continue
            else
                %fprintf("theta3 is out of range!\n")
                checksafe = 0;
            end
        elseif j==4
            if -140<dis_out(j) &&dis_out(j)<140
                continue
            else
                %fprintf("theta4 is out of range!\n")
                checksafe = 0;
            end
        elseif j==5
             if -100<dis_out(j) &&dis_out(j)<100
                continue
    
            else
                %fprintf("theta5 is out of range!\n")
                checksafe = 0;
            end
        elseif j==6
             if -260<dis_out(j) &&dis_out(j)<260
                continue
    
             else
                %fprintf("theta6 is out of range!\n")
                checksafe = 0;
             end
        end
    end
    
    disp(dis_out)
    if checksafe == 1
        theta = dis_out;
    end
end
%INVERSE Summary of this function goes here
%   Detailed explanation goes here
fprintf('final out is:');
disp(theta);
%outputArg2 = inputArg2;
end

