function plot_Cartesian_motion(A,B,C,SamplingTime,x,y,z,a)
    

    t = 0 : SamplingTime : 1;
    % Plot Cartesian motion position/velocity/acceleration
    PX = [x{1} x{2} x{3}];         %Position of X,Y,Z     
    PY = [y{1} y{2} y{3}];
    PZ = [z{1} z{2} z{3}];
    
   %Velocity of X,Y,Z
    VX = diff(PX)/SamplingTime;     VX(151) = VX(150);
    VY = diff(PY)/SamplingTime;     VY(151) = VY(150);
    VZ = diff(PZ)/SamplingTime;     VZ(151) = VZ(150);
    
    %Accelaration of X,Y,Z
    AX = diff(VX)/SamplingTime;     AX(151) = AX(150);
    AY = diff(VY)/SamplingTime;     AY(151) = AX(150);
    AZ = diff(VZ)/SamplingTime;     AZ(151) = AX(150);
    
    figure(5);
    subplot(3,1,1);
    plot(t,PX);
    title('x');
    xlabel('time(sec)');
    ylabel('position(m)');
    grid on
    subplot(3,1,2);
    plot(t,PY);
    title('y');
    xlabel('time(sec)');
    ylabel('position(m)');
    grid on;
    subplot(3,1,3);
    plot(t,PZ);
    title('z');
    xlabel('time(sec)');
    ylabel('position(m)');
    grid on;
    
    figure(6);
    subplot(3,1,1);
    plot(t(1:end-1),VX);
    title('x');
    xlabel('time(sec)');
    ylabel('velocity(m/sec)');
    grid on
    subplot(3,1,2);
    plot(t(1:end-1),VY);
    title('y');
    xlabel('time(sec)');
    ylabel('velocity(m/sec)');
    grid on;
    subplot(3,1,3);
    plot(t(1:end-1),VZ);
    title('z');
    xlabel('time(sec)');
    ylabel('velocity(m/sec)');
    grid on;
    
    figure(7);
    subplot(3,1,1);
    plot(t(1:end-2),AX);
    title('x');
    xlabel('time(sec)');
    ylabel('acceleration(m/sec-2)');
    
    grid on
    subplot(3,1,2);
    plot(t(1:end-2),AY);
    title('y');
    xlabel('time(sec)');
    ylabel('acceleration(m/sec-2)');
    grid on;
    subplot(3,1,3);
    plot(t(1:end-2),AZ);
    title('z');
    xlabel('time(sec)');
    ylabel('acceleration(m/sec-2)');
    grid on;
    
    % Plot Cartesian motion 3D

    
    figure(8);
    for i = 1 : size(x{1},2)
        plot3([x{1}(i),x{1}(i)+0.1*a{1}(1,i)],[y{1}(i),y{1}(i)+0.1*a{1}(2,i)],[z{1}(i),z{1}(i)+0.1*a{1}(3,i)],'c-');
        hold on;
    end
    for i = 1 : size(x{2},2)
        plot3([x{2}(i),x{2}(i)+0.1*a{2}(1,i)],[y{2}(i),y{2}(i)+0.1*a{2}(2,i)],[z{2}(i),z{2}(i)+0.1*a{2}(3,i)],'c-');
        hold on;
    end
    for i = 1 : size(x{3},2)
        plot3([x{3}(i),x{3}(i)+0.1*a{3}(1,i)],[y{3}(i),y{3}(i)+0.1*a{3}(2,i)],[z{3}(i),z{3}(i)+0.1*a{3}(3,i)],'c-');
        hold on;
    end
    plot3(x{1},y{1},z{1},'b-');% (AA'、A'C'、C'C)path
    plot3(x{2},y{2},z{2},'b-');
    plot3(x{3},y{3},z{3},'b-');
    title('3D path of Cartesian Space planning')
    xlabel('X-axis(m)');
    ylabel('Y-axis(m)');
    zlabel('Z-axis(m)');

    text(A(1,4),A(2,4),A(3,4),strcat('A(',num2str(A(1,4)),',',num2str(A(2,4)),',',num2str(A(3,4)),')'));
    text(B(1,4),B(2,4),B(3,4),strcat('B(',num2str(B(1,4)),',',num2str(B(2,4)),',',num2str(B(3,4)),')'));
    text(C(1,4),C(2,4),C(3,4),strcat('C(',num2str(C(1,4)),',',num2str(C(2,4)),',',num2str(C(3,4)),')'));

    scatter3(A(1,4),A(2,4),A(3,4));% position A、B、C
    scatter3(B(1,4),B(2,4),B(3,4));
    scatter3(C(1,4),C(2,4),C(3,4));

    
    grid on;
    hold off;


end