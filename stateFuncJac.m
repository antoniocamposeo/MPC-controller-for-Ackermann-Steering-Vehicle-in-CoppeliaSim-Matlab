function [A,Bmv] = stateFuncJac(x,u)
    % Parameters
    
    l = 0.6;
    
    % States
    x1 = x(1); % 
    x2 = x(2); % 
    x3 = x(3); % 
    x4 = x(4); % 
    
    % Inputs
    u1 = u(1); % 
    u2 = u(2); % 
    %u3 = u(3); % 
    
    
    A = [1,0,sin((x3))*(u1),0;
        0,1,cos((x3))*(u1),0;
        0,0,1,((u1)*(tan((x4))^2 + 1))/(l);0,0,0,0];
    Bmv =[cos((x3)),0;
        -sin((x3)),0;
        tan((x4))/(l),0;0,1];
end

