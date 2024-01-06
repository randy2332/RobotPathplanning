function plot_joint_motion(A,B,C,SamplingTime,q,dq,d2q)

    % Plot joint motion angle, angular velocity, angular acceleration, and 3D path

    t = 0 : SamplingTime : 1;
    figure(1);
    for i = 1 : 6
        subplot(3,2,i);
        plot(t,[q{1}(i,:) q{2}(i,:) q{3}(i,:)]);
        title(strcat('Joint',num2str(i)));
        ylabel(strcat('angle(degree)'));
        xlabel('time(sec)');
        grid on;
    end
    sgtitle("Joint value");
     
    figure(2);
    for i = 1 : 6
        subplot(3,2,i);
        plot(t,[dq{1}(i,:) dq{2}(i,:) dq{3}(i,:)]);
        title(strcat('Joint',num2str(i)));
        ylabel(strcat('angular velocity(degree/sec)'));
        xlabel('time(sec)');
        grid on;
    end
    sgtitle("Joint value");
     
    figure(3);
    for i = 1 : 6
        subplot(3,2,i);
        plot(t,[d2q{1}(i,:) d2q{2}(i,:) d2q{3}(i,:)]);
        title(strcat('Joint',num2str(i)));
        ylabel(strcat('angular acceleration(degree/sec-2)'));
        xlabel('time(sec)');
        grid on;
    end
    sgtitle("Joint value");

    % Plot Joint motion 3D
    for i = 1 : 3
        for index = 1 : size(q{i},2)
            [cp, ~] = forward(q{i}(:,index));   
            px{i}(index) = cp(1);
            py{i}(index) = cp(2);
            pz{i}(index) = cp(3);
            ax{i}(index) = cp(7);
            ay{i}(index) = cp(8);
            az{i}(index) = cp(9);
            
        end
    end
    
 
    figure(4);
    for i = 1 : size(px{1},2)
        plot3([px{1}(i),px{1}(i)+0.1*ax{1}(i)],[py{1}(i),py{1}(i)+0.1*ay{1}(i)],[pz{1}(i),pz{1}(i)+0.1*az{1}(i)],'c-');
        hold on;
    end
    for i = 1 : size(px{2},2)
        plot3([px{2}(i),px{2}(i)+0.1*ax{2}(i)],[py{2}(i),py{2}(i)+0.1*ay{2}(i)],[pz{2}(i),pz{2}(i)+0.1*az{2}(i)],'c-');
        hold on;
    end
    for i = 1 : size(px{3},2)
        plot3([px{3}(i),px{3}(i)+0.1*ax{3}(i)],[py{3}(i),py{3}(i)+0.1*ay{3}(i)],[pz{3}(i),pz{3}(i)+0.1*az{3}(i)],'c-');
        hold on;
    end
    plot3(px{1},py{1},pz{1},'b-');% (AA'、A'C'、C'C)path
    plot3(px{2},py{2},pz{2},'b-');
    plot3(px{3},py{3},pz{3},'b-');
    title('3D path of Joint Space planning ')
    xlabel('X-axis(m)');
    ylabel('Y-axis(m)');
    zlabel('Z-axis(m)');
 
    text(A(1,4),A(2,4),A(3,4),strcat('A(',num2str(A(1,4)),',',num2str(A(2,4)),',',num2str(A(3,4)),')'));
    text(B(1,4),B(2,4),B(3,4),strcat('B(',num2str(B(1,4)),',',num2str(B(2,4)),',',num2str(B(3,4)),')'));
    text(C(1,4),C(2,4),C(3,4),strcat('C(',num2str(C(1,4)),',',num2str(C(2,4)),',',num2str(C(3,4)),')'));

    grid on;
    hold off;

   
end