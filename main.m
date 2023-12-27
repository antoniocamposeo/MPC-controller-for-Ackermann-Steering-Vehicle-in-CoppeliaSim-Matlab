clc
close all 
clear 
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONNECTION COPPELIASIM %
%%%%%%%%%%%%%%%%%%%%%%%%%
% Connection trought RemoteApi
disp('Program started');
% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);
sim.simxSetFloatingParameter(clientID, sim.sim_floatparam_simulation_time_step, 0.1, sim.simx_opmode_oneshot_wait);
% sim.setStepping(true)
% get n_obj Manta object coppeliasim 
[rCode,obj_n]=sim.simxGetObjectHandle(clientID,'/nakedAckermannSteeringCar',sim.simx_opmode_oneshot_wait);
sim.simxSetObjectPosition(clientID, obj_n, -1, [0,0,+0.3],sim.simx_opmode_oneshot_wait);

%sim.simxGetStringSignal(clientID,'signal_out',sim.simx_opmode_streaming); %Tell V-REP to stream the data
% [~,EValue]=sim.simxGetStringSignal(clientID,'signal_out',sim.simx_opmode_blocking);
% E=sim.simxUnpackFloats(EValue)

%% 
%%%%%%%%%%%%%%%%%%
% CONTROLLER MPC %
%%%%%%%%%%%%%%%%%%

nx = 4;
ny = 2;
nu = 2;
mpcobj = nlmpc(nx,ny,nu);

% T= 1 , T =Ts*Hp
% Ts : 10% - 25% 
Ts = 0.2; % Sampling time 
Hp = 10; % Prediction Horizon
Hc = 3; % Control Horizion

mpcobj.Ts = Ts;
mpcobj.PredictionHorizon = Hp;
mpcobj.ControlHorizon = Hc;
mpcobj.Model.StateFcn = "stateFunc";
mpcobj.Model.OutputFcn = "outFunc";
%mpcobj.Jacobian.StateFcn = "stateFuncJac";
%mpcobj.Jacobian.OutputFcn = "outFuncJac";

% Dicrete Time control
mpcobj.Model.IsContinuousTime = false;
mpcobj.Model.NumberOfParameters = 0;

% Intial state
x0 = [0 0 pi/2 0];
xk = x0;
u0 = [0 0];
mv = u0;
validateFcns(mpcobj,xk,mv)

% Costraints 
mpcobj.ManipulatedVariables(1).Min = -5*20*pi/180; % from simulator 
mpcobj.ManipulatedVariables(1).Max =  5*20*pi/180;

mpcobj.ManipulatedVariables(2).Min = -deg2rad(45);
mpcobj.ManipulatedVariables(2).Max = deg2rad(45);
 
% Weights
mpcobj.Weights.OutputVariables = [40 25];
%mpcobj.Weights.ManipulatedVariablesRate(1) = 0.5;
%mpcobj.Weights.ManipulatedVariablesRate(2) = 0.5;
mpcobj.Weights.ManipulatedVariables = [0.3 2];

%%
%%%%%%%%%%%%%%%%%%%
% PATH GENERATION %
%%%%%%%%%%%%%%%%%%%
wpts = [0 0 2 
        0 2 7];
tpts = [0 5 15];
tvec = 0:0.2:15;
[q, qd, qdd, pp] = cubicpolytraj(wpts, tpts, tvec);
plot(tvec, q)
hold all
plot(tpts, wpts, 'x')
xlabel('t')
ylabel('Positions')
legend('X-positions','Y-positions')
hold off

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SIMULATION ON COPPELIASIM %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% States and reference
x0 = [0 0 pi/2 0];
xk = x0;
u0 = [0 0];
mv = u0;
ref_vet = q';

Tf = 15;
% tvec = 0:Ts:Tf;

err = []; % error
k = 1;
h = 1;
x_history = [];
mv_history = [];
y_history = [];
c_history = [];
ref_history = [];
cost= [];

if (clientID>-1)
    disp('Connected to remote API server');
    pause(2);
    t=clock;
    startTime=t(6); 
    currentTime=t(6);
    
    % start simulation
    sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);
    flag = 1;
    
    pause(0.5)
    tic;
    step = 1; % 5
    yref = ref_vet(1,:);
    % close loop
    for i = 1:length(tvec)
         
        [mv,~,info] = nlmpcmove(mpcobj,xk,mv,yref,[]);
        %disp(info.Cost) 
        % Send signal
        set_input(sim,clientID,mv);
        %disp(mv);
        % get orientation
        %[rCode,orientation] = sim.simxGetObjectOrientation(clientID,obj_n,-1,sim.simx_opmode_blocking);
        [rCode, orientation] = sim.simxGetObjectQuaternion(clientID,obj_n,-1,sim.simx_opmode_blocking);
        % sim.simxGetObjectQuaternion(clientID,obj_n,-1,sim.simx_opmode_blocking); 
        eul = quat2eul(orientation);
        x_3 = eul(3);
        disp(rad2deg(orientation));

        % get position 
        [rCode,position] = sim.simxGetObjectPosition(clientID,obj_n,-1,sim.simx_opmode_blocking);
        x_1 = position(1);
        x_2 = position(2);
        x_4 = mv(2);
        fprintf("%f, %f, %f\n",x_1,x_2,rad2deg(x_3));
        fprintf("%f, %f \n",yref(1),yref(2))
        % states 
        xk = double([x_1,x_2,x_3,x_4]);
        y = [x_1,x_2];
        
%         error= norm([(yref(1)- y(1)) (yref(2) - y(2))]);
%         disp(error)
        yref = ref_vet(i,:);

        t=clock;
        currentTime=t(6);
        % History vectors
        cost = [cost;info.Cost];
        ref_history = [ref_history; yref];
        x_history   = [x_history; xk];
        y_history   = [y_history; y];
        mv_history  = [mv_history; mv'];
        c_history   = [c_history; toc];
        tic;
    end
    
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);

    % Nown-blocking fashi send some data to CoppeliaSim in a noon:
    sim.simxAddStatusbarMessage(clientID,'Connection Ended!',sim.simx_opmode_oneshot);

    % Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID);

    % Now close the connection to CoppeliaSim:    
    sim.simxFinish(clientID);
    
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

disp('Program ended');
 
plotter;


