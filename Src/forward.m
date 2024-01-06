function [output,T] = forward(input)
%Q1

%input_str = input;
vector_input =input %str2num(input_str);%let the string type to float type

ranges = [-160, 160; -125, 125; -135, 135; -140, 140; -100, 100; -260, 260];
%check range
for i = 1:6
    if vector_input(i) < ranges(i, 1) || vector_input(i) > ranges(i, 2)
        fprintf("theta%d is out of range!\n", i);
    end
end
%end of check

% parameter
% We let the  parameter of kinematic table to array with 6 value 
% The reason we do this step is for computing A1~A6 easily
d_array = [0 0 0.149 0.433 0 0];
a_array = [0 0.432 -0.02 0 0 0];
alpha = [-1/2 0 1/2 -1/2 1/2 0];% origin is alpha == [-pi/2 0 pi/2 -pi/2 pi/2 0]
                                % We  ignore pi because we will use cospi instead of cos, which can avoid cos(pi/2) not 0 but very small value;

% Initialize transformation matrix as identity
T = eye(4);
%build A1*A2*...A6
for i =1:6
    angle_radian = deg2rad(vector_input(i));% cos and sin function receive randian but not angle
    A_i = [cos(angle_radian), -sin(angle_radian)*cospi(alpha(i)), sin(angle_radian)*sinpi(alpha(i)), a_array(i)*cos(angle_radian); 
             sin(angle_radian), cos(angle_radian)*cospi(alpha(i)) , -cos(angle_radian)*sinpi(alpha(i)), a_array(i)*sin(angle_radian);
             0, sinpi(alpha(i)), cospi(alpha(i)) , d_array(i);
             0, 0, 0, 1];
    %disp(T);
    T = T * A_i;
end

% result

angle3_radian = atan2(T(2,3),T(1,3));
angle3_degree = rad2deg(angle3_radian);

angle2_radian = atan2(cos(angle3_radian)*T(1,3)+sin(angle3_radian)*T(2,3),T(3,3));
angle2_degree = rad2deg(angle2_radian);

angle1_radian = atan2(-sin(angle3_radian)*T(1,1)+cos(angle3_radian )*T(2,1) ,-sin(angle3_radian)*T(1,2)+cos(angle3_radian)*T(2,2));
angle1_degree = rad2deg(angle1_radian);

output  = [T(1,4) T(2,4) T(3,4) angle1_degree angle2_degree angle3_degree,T(1,3) T(2,3) T(3,3)];

%display result
fprintf("[n o a p]:\n")
disp(T)
fprintf("output:\n")
disp(output)



end

