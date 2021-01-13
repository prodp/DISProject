% perfs
tiledlayout(2,2)
nexttile
% plot 1
plot(clock0,instant0, 'Color', "#0072BD", 'LineStyle',  "-");
hold on
plot(clock1,instant1, 'Color', "#0072BD", 'LineStyle',  "--");
xlabel('time [steps]');
ylabel('instant performance');
title('Instantaneous performance');
hold off
% plot 2
nexttile
plot(clock0,overall0, 'Color', "#0072BD", 'LineStyle',  "-");
hold on
plot(clock1, overall1, 'Color', "#0072BD", 'LineStyle',  "--");
xlabel('time [steps]');
ylabel('overall performance');
title('Overall performance over time');
hold off

%plot 3
nexttile([1 2])
%nexttile
plot(clock0,cohesion0, 'Color', "#0072BD", 'LineStyle',  "-");
hold on
plot(clock1,cohesion1, 'Color', "#0072BD", 'LineStyle',  "--");
%xlabel('time [steps]');
%ylabel('cohesion [%]');
%title('Cohesion perf over time');
hold on


plot(clock0,orientation0, 'Color', "#D95319", 'LineStyle',  "-");
hold on
plot(clock1,orientation1, 'Color', "#D95319", 'LineStyle',  "--");
%xlabel('time [steps]');
%ylabel('orientation [%]');
%title('Orientation perf over time');

plot(clock0,velocity0, 'Color', "#7E2F8E", 'LineStyle',  "-");
hold on
plot(clock1,velocity1, 'Color', "#7E2F8E", 'LineStyle',  "--");
%xlabel('time [steps]');
%ylabel('velocity [%]');
%title('Velocity perf over time');
legend(["cohesion1", "cohesion2", "orientation", "orientation2", "velocity", "velocity2"]);
title('Individual performance over time');
hold off