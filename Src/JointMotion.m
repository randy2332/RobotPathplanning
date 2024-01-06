function [q,dq,d2q] = JointMotion(A,B,C,SamplingTime)
    % JointMotion: Calculate joint angles, velocities, and accelerations
    % for a path from A to B to C.
    thetaA = inverse(A);
    thetaB = inverse(B);
    thetaC = inverse(C);
    
    % Initialize
    q1 = []; dq1 = []; d2q1 = [];
    q2 = []; dq2 = []; d2q2 = [];
    q3 = []; dq3 = []; d2q3 = [];
    
    % path A to A'
    count = 1;
    for t = -0.5 : SamplingTime : -0.2
        h = (t+0.5)/0.5;
        q1(:,count) = (thetaB - thetaA) * h + thetaA; % angle
        dq1(:,count) = (thetaB - thetaA)/0.5;         % angular velocity
        d2q1(:,count) = zeros(6,1);                   % angular acceleration
        count = count + 1;
    end
    
    % path A' to C'
    count = 1;
    thetaA_p = (thetaB - thetaA) * ((-0.2+0.5)/0.5) + thetaA;
    dB = thetaA_p - thetaB;
    dC = thetaC - thetaB;
    for t = (-0.2+SamplingTime) : SamplingTime : (0.2 - SamplingTime)
        
        h = (t+0.2)/0.4;
        
        q2(:,count) = ((dC*0.2/0.5+dB)*(2-h)*(h^2)-2*dB)*h+thetaB+dB; % angle
        dq2(:,count) = ((dC*0.2/0.5+dB)*(1.5-h)*2*(h^2)-dB)/0.2;      % angular velocity
        d2q2(:,count) = (dC*0.2/0.5+dB)*(1-h)*3*h/(0.2^2);            % angular acceleration
        count=count+1; 
    end
    
    % path C' to C
    count = 1;
    for t = 0.2 : SamplingTime : 0.5
        h = t/0.5;
        q3(:,count) = dC*h+thetaB;      % angle
        dq3(:,count) = dC/0.5;          % angular velocity
        d2q3(:,count) = zeros(6,1);     % angular acceleration
        count=count+1;
    end
    % Combine all paths
    q = {q1,q2,q3};
    dq = {dq1,dq2,dq3};
    d2q = {d2q1,d2q2,d2q3};
    
end