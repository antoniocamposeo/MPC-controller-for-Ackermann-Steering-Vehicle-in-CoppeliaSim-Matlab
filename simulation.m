
% History vectors init
x_history = [];
mv_history = [];
y_history = [];
c_history = [];
ref_history = [];
cost= [];
tvec = 0:Ts:Tf;

tic;
for i=0:length(tvec)-1 % Simulation loop
 

     [mv,~,info] = nlmpcmove(mpcobj,xk,mv,yref);
        


    xk = info.Xopt(2,:);
    y = info.Yopt(1,:);
    cost = [cost;info.Cost];
    
    % History vectors
    ref_history = [ref_history; yref];
    x_history   = [x_history; xk];
    y_history   = [y_history; y];
    mv_history  = [mv_history; mv'];
    c_history   = [c_history; toc];
    tic;

end

fprintf("Tempo di esecuzione %.2f secondi.\n", sum(c_history))

% Plot the closed-loop response.
plotter;