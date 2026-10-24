function [collisionDetected, firstIndex, minClearance] = trajectoryCollision(pathXY, obstacles, safetyMargin)
%TRAJECTORYCOLLISION Check path points against circular obstacles.
%   pathXY: N-by-2 points [x y].
%   obstacles: M-by-3 [xCentre yCentre radius].
%   safetyMargin: extra clearance added to each radius.
%   Outputs collision flag, first colliding path-point index (0 if none),
%   and minimum clearance across all point/obstacle pairs.
%
% This checks sampled points only; it does not guarantee swept-path safety.

    validateattributes(pathXY, {'numeric'}, {'2d','ncols',2,'finite'});
    validateattributes(obstacles, {'numeric'}, {'2d','ncols',3,'finite'});
    validateattributes(safetyMargin, {'numeric'}, {'scalar','nonnegative','finite'});

    collisionDetected = false;
    firstIndex = 0;
    minClearance = inf;

    if isempty(pathXY) || isempty(obstacles)
        return;
    end
    if any(obstacles(:,3) < 0)
        error('Obstacle radii must be nonnegative.');
    end

    for i = 1:size(pathXY,1)
        delta = obstacles(:,1:2) - pathXY(i,:);
        distances = sqrt(sum(delta.^2,2));
        clearance = distances - obstacles(:,3) - safetyMargin;
        minClearance = min(minClearance, min(clearance));
        if any(clearance <= 0)
            collisionDetected = true;
            firstIndex = i;
            return;
        end
    end
end
