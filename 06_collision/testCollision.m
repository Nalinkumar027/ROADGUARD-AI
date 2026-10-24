%% Test trajectory collision checking
path = [0 0; 1 0; 2 0; 3 0];
obstacles = [2 0 0.25];

[hit, firstIndex, minClearance] = trajectoryCollision(path, obstacles, 0.1);
assert(hit);
assert(firstIndex == 3);
assert(minClearance <= 0);

[hit2, ~, ~] = trajectoryCollision(path, [2 3 0.25], 0.1);
assert(~hit2);

disp('testCollision passed.');
