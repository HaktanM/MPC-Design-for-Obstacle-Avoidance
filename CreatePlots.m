function CreatePlots(N,T,X,U,state_offset,horizontol_position,vertical_position)

% Figure Formatting
set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex'); 

time = 0 : T : (N-1)*T;
figure
plot(time,X(1,:)+state_offset(1),'LineWidth',2);
hold all;

plot(time,X(2,:)+state_offset(2),'LineWidth',2);
plot(time,X(3,:)+state_offset(3),'LineWidth',2);
plot(time,X(4,:)+state_offset(4),'LineWidth',2);
plot(time,X(5,:)+state_offset(5),'LineWidth',2);

xlabel('time [sec]')
ylabel('States')
grid on;
legend({'$\dot{y}$','$\dot{\psi}$','$e_{\psi}$','$e_y$','$\delta$'},'FontSize',14)
set(findall(gcf,'Type','line'),'LineWidth',2)
set(findall(gcf,'-property','FontSize'),'FontSize',14);
title('Initial Condition Rejection with Disturbances')
set(findall(gcf,'Type','line'),'LineWidth',2)
set(findall(gcf,'-property','FontSize'),'FontSize',14);


time = 0 : T : (N-2)*T;
figure
plot(time,U,'LineWidth',2);
xlabel('time [sec]')
ylabel('u')
grid on;

title('Control Effort')
set(findall(gcf,'Type','line'),'LineWidth',2)
set(findall(gcf,'-property','FontSize'),'FontSize',14);




figure
plot(horizontol_position,vertical_position,'LineWidth',2);
hold on


[ObstacleLocation, ObstacleLength, ObstacleWidth] = defineObstacle;

x1 = ObstacleLocation;
y1 = -0.5*ObstacleWidth;

x2 = ObstacleLocation+ObstacleLength;
y2 = -0.5*ObstacleWidth;

x3 = ObstacleLocation+ObstacleLength; 
y3 = 0.5*ObstacleWidth;

x4 = ObstacleLocation;
y4 = 0.5*ObstacleWidth;

rectangle_x = [x1 x2 x3 x4 x1];
rectangle_y = [y1 y2 y3 y4 y1];

plot(rectangle_x,rectangle_y,'LineWidth',2)
fill(rectangle_x,rectangle_y,'r')

xlabel('Horizontal Position (m)')
ylabel('Vertical Position (m)')
grid on;
%xlim([horizontol_position(1) horizontol_position(length(horizontol_position)-1)])
ylim([-10 10])
set(findall(gcf,'Type','line'),'LineWidth',2)
set(findall(gcf,'-property','FontSize'),'FontSize',14);
title('Obstacle Avoidance')
set(findall(gcf,'Type','line'),'LineWidth',2)
set(findall(gcf,'-property','FontSize'),'FontSize',14);

end