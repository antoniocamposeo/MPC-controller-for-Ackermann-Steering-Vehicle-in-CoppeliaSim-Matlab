function dxdt = stateFunc(x,u)
    % Parameters
    l = 0.755; % wheel base %   
        
   
    
    % States
    x1 = x(1); % 
    x2 = x(2); % 
    x3 = x(3); % 
    x4 = x(4); % 
    
    % Inputs
    u1 = u(1); % 
    u2 = u(2); % 
    %u3 = u(3); % 
    

    dxdt = zeros(length(x),1);
    dxdt(1) =(x1 + (u1)*cos(x3)*0.2);
    dxdt(2) =x2 + (u1)*sin(x3)*0.2;
    dxdt(3) =x3 + ((u1)*tan(x4)/l)*0.2;
    dxdt(4) = x4 + (u2)*0.1;
    
end

