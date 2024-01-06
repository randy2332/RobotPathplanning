function [px,py,pz,a] = CartesianMotion(A,B,C,SamplingTime)
  

    count = 1;

    for t = -0.5 : SamplingTime : -0.2  % path AA'
        h = (t+0.5)/0.5;
        LD1 = LinearDrive(A,B,h);	% drive matrix
        m_A_Ap(:,:,count) = A*LD1;  % [n o a p] matrix

        x1(:,count) = m_A_Ap(1,4,count);      
        y1(:,count) = m_A_Ap(2,4,count);   
        z1(:,count) = m_A_Ap(3,4,count);    
        a1(:,count) = m_A_Ap(1:3,3,count);
        
        count = count+1;
    end
    Ap = m_A_Ap(:,:,count-1);   % position A'
    
    count = 1;
    for t = (-0.2+SamplingTime) : SamplingTime : (0.2-SamplingTime)   % path A'C'
        h = (t+0.2)/(2*(0.2));
        TD = TransitionDrive(Ap,B,C,h);	% drive matrix
        m_Ap_Cp(:,:,count) = B*TD;     % [n o a p] matrix

        x2(:,count) = m_Ap_Cp(1,4,count);
        y2(:,count) = m_Ap_Cp(2,4,count);
        z2(:,count) = m_Ap_Cp(3,4,count);
        a2(:,count) = m_Ap_Cp(1:3,3,count);
        
        count = count+1;
    end
    
    count=1;
    for t = 0.2 : SamplingTime : 0.5  % path C'C
        h = t/(0.5);
        LD3 = LinearDrive(B,C,h);    % drive matrix
        m_Cp_C(:,:,count) = B*LD3;  % [n o a p] matrix
   
        x3(:,count) = m_Cp_C(1,4,count);       
        y3(:,count) = m_Cp_C(2,4,count);
        z3(:,count) = m_Cp_C(3,4,count);    
        a3(:,count) = m_Cp_C(1:3,3,count);
        
        count = count+1;
    end
    % combine path segment
    px = {x1,x2,x3};             
    py = {y1,y2,y3};
    pz = {z1,z2,z3};
    a = {a1,a2,a3};
   
end


    