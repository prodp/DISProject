% perfs
tiledlayout(2,2)
nexttile
% plot 1
plot(clock0,instant0, 'Color', "#0072BD", 'LineStyle',  "-");
xlabel('time [steps]');
ylabel('instant performance');
title('Instantaneous performance');
hold off
% plot 2
nexttile
plot(clock0,overall0, 'Color', "#0072BD", 'LineStyle',  "-");
xlabel('time [steps]');
ylabel('overall performance');
title('Overall performance over time');
hold off

%plot 3
nexttile([1 2])
%nexttile
plot(clock0,cohesion0, 'Color', "#0072BD", 'LineStyle',  "-");
%xlabel('time [steps]');
%ylabel('cohesion [%]');
%title('Cohesion perf over time');
hold on


plot(clock0,orientation0, 'Color', "#D95319", 'LineStyle',  "-");
%xlabel('time [steps]');
%ylabel('orientation [%]');
%title('Orientation perf over time');

plot(clock0,velocity0, 'Color', "#7E2F8E", 'LineStyle',  "-");
%xlabel('time [steps]');
%ylabel('velocity [%]');
%title('Velocity perf over time');
legend(["cohesion", "orientation", "velocity"]);
title('Individual performance over time');
hold off