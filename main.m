%% ROADGUARD AI — 2-D planning starter simulation
% Educational prototype only. Not for real-vehicle control.
clear; clc; close all;

% Grid convention: 0 = free cell, 1 = blocked cell.
gridSize = [30, 40];
occupancy = zeros(gridSize);

% Synthetic obstacles used only to demonstrate planning.
occupancy(8:24, 15) = 1;
occupancy(8, 15:27) = 1;
occupancy(20:30, 27) = 1;
occupancy(13, 15:27) = 0; % gap through the first barrier

startRC = [3, 3];
goalRC  = [27, 37];

pathRC = astarPlanner(occupancy, startRC, goalRC);
if isempty(pathRC)
    error('No path found. Adjust the synthetic obstacle map.');
end

% Simulated dynamic object: [x y vx vy], in grid-column/grid-row units.
objectState = [22, 8, -0.25, 0.12];
numSteps = 12;
predictedObject = motionPredictor(objectState, numSteps, 1.0);

% Demonstration collision check: path points are [x y], obstacles are [x y radius].
pathXY = [pathRC(:,2), pathRC(:,1)];
obstacles = [15, 16, 1.2; 27, 25, 1.2];
[collisionDetected, firstCollisionIndex] = trajectoryCollision(pathXY, obstacles, 0.75);

figure('Name','ROADGUARD AI Starter Simulation','Color','w');
imagesc(occupancy);
colormap(flipud(gray));
axis equal tight;
set(gca,'YDir','normal');
hold on;
plot(startRC(2), startRC(1), 'go', 'MarkerFaceColor','g', 'MarkerSize',8);
plot(goalRC(2), goalRC(1), 'bx', 'LineWidth',2, 'MarkerSize',10);
plot(pathXY(:,1), pathXY(:,2), 'b-', 'LineWidth',2);
plot(obstacles(:,1), obstacles(:,2), 'ro', 'MarkerSize',10, 'LineWidth',1.5);
plot(predictedObject(:,1), predictedObject(:,2), 'm--', 'LineWidth',1.5);
legend('Start','Goal','Planned path','Obstacle centres','Predicted object path', ...
    'Location','southoutside');
title('ROADGUARD AI — synthetic grid demonstration');
xlabel('Grid column / X');
ylabel('Grid row / Y');
hold off;

if collisionDetected
    fprintf('Collision check: possible collision at path point %d.\n', firstCollisionIndex);
else
    fprintf('Collision check: no collision detected on the supplied path.\n');
end
fprintf('Planned path contains %d grid points.\n', size(pathRC,1));
