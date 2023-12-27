clear 
clc

close all 
%%  INSERIRE PARAMETRI 
l = 0.6
%%  NUMERO DI VARIABILI 
n_var = 4 ;
n_input = 2; 
n_input_mv = 2;
n_input_md = 0;
n_output = 2;
%% 
x = zeros(n_var,1);
u = zeros(n_input,1);
y = zeros(n_output,1);

x1 = x(1); % 
x2 = x(2); % 
x3 = x(3); % 
x4 = x(4); % 


% Inputs
u1 = u(1);%*100*0.1; % 
u2 = u(2); % 
%u3 = u(3); % 
   

y1 = y(1);
y2 = y(2);
% y3 = y(3);
% y4 = y(4);
% y5 = y(5);
% 


%% jacobian matrix 
syms x1 x2 x3 x4 u1 u2 l

dxdt_1 = x1 + (u1)*cos(x3);
dxdt_2 = x2 + (u1)*sin(x3);
dxdt_3 = x3 + (u1)*tan(x4)/l;
dxdt_4 = u2;
    
y_1 = x1;
y_2 = x2;
% y_3 = x2;
% y_4 = x3;
% y_5 = x4;
%%

jacob_state = zeros(n_var,n_var);
jacob_input = zeros(n_var,n_input_mv);
jacob_output = zeros(n_output,n_var);
jacob_state = symmatrix(jacob_state);
jacob_input = symmatrix(jacob_input);
jacob_output = symmatrix(jacob_output);

% number of equation 
dxdt = [dxdt_1,dxdt_2,dxdt_3,dxdt_4]';
y_dt = [y_1,y_2]'; 
x = [x1,x2,x3,x4];
u = [u1,u2];

%% Jacobian
% for state

for j = 1:n_var
    for i = 1:n_var
        jacob_state(j,i)= diff(dxdt(j),x(i));
    
    end
end

% for input 
for j = 1:n_var
    for i =1:n_input_mv
        jacob_input(j,i)= diff(dxdt(j),u(i));
    end
end

% for output 
for j = 1:n_output
    for i =1:n_var
        jacob_output(j,i)= diff(y_dt(j),x(i));
    end
end


%% Convert into string
A = symmatrix2sym(jacob_state);
Bmv = symmatrix2sym(jacob_input);
C = symmatrix2sym(jacob_output);

str_A = "[";

for i = 1:n_var
    k = 1;
    for j = 1:n_var
        if k == n_var
            str_A = str_A + string(A(i,j));
            str_A = str_A + ";";
        else
            str_A = str_A + string(A(i,j));
            str_A = str_A + ",";
        end
        k = k+1;
    end
    
end
str_A = str_A + "]";
str_A = erase(str_A,"conj");

str_Bmv = "[";
for i = 1:n_var
    k = 1;
    for j = 1:n_input_mv
        if k == n_input_mv
            str_Bmv = str_Bmv + string(Bmv(i,j));
            str_Bmv = str_Bmv + ";";
        else
            str_Bmv = str_Bmv + string(Bmv(i,j));
            str_Bmv = str_Bmv + ",";
        end
        k = k+1;
    end
end
str_Bmv = str_Bmv + "]";
str_Bmv = erase(str_Bmv,"conj");

str_C = "[";
for i = 1:n_output
    k = 1;
    for j = 1:n_var
        if k == n_var 
            str_C= str_C + string(C(i,j));
            str_C= str_C + ";";
        else
            str_C= str_C + string(C(i,j));
            str_C= str_C + ",";
    
        end
        k = k+1;
    end
end
str_C = str_C + "]";
str_C = erase(str_C,"conj");

