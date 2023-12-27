% Plotting
close all;

n_state = 4 ;
n_output = 2;

stateLabels = ["x1 [m]", "x2[m]", "x3[°]", "x4[m/s]"];
outputLabels = ["y1 [m]", "y2[m]"];

figure(1); % state Plots
for i=1:n_state
    subplot(n_state,1,i)
    plot(tvec,x_history(:,i));
    grid minor
    hold on
    xlim([0 Tf]);
    xlabel("Time [seconds]")
    ylabel(stateLabels(i))
end
hold on

figure(2); % output Plots
for i=1:n_output
    subplot(n_output,1,i)
    plot(tvec,y_history(:,i));
    grid minor
    hold on
    % Reference
    plot(tvec,ref_history(:,i));
    xlim([0 Tf]);
    xlabel("Time [seconds]")
    ylabel(outputLabels(i))
end
hold on

figure(3); % MV Plot
plot(tvec,mv_history)
grid minor
legend("u1","u2")
ylim([min(min(mv_history))-1 max(max(mv_history))+1]);
ylabel("Valve values [%]")
xlabel("Time [seconds]")
title("Manipulated Variables")
hold on 

figure(4) % Computational Time plot
plot(tvec,c_history);
xlim([0 Tf]);
grid minor
ylabel("Time to compute  [s]")
xlabel("Time of simulation [seconds]")
title("Computational Time")
hold on

figure(5) % Reference Plot
plot(ref_history(:,1),ref_history(:,2),'LineWidth',1);
hold on 
plot(y_history(:,1),y_history(:,2),'LineWidth',1);
grid on
xlim([0 Tf]);
ylabel("Y [m]")
xlabel("X []")
title("Reference")
hold on 



% figure(4) % Reference plot
% stairs(tvec,ref_history(:,3),'LineWidth',2);
% grid on
% ylim([0 4000]);
% xlim([0 Tf]);
% xticks([0 5 30 50 90 130 160 210 Tf])
% yticks([0 1000 2000 3500])
% ylabel("Height [m]")
% xlabel("Time [seconds]")
% title("Reference")