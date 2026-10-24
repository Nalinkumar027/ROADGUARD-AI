%% Test A* planning on a small grid
grid = zeros(10,10);
grid(2:9,5) = 1;
grid(6,5) = 0; % one opening
path = astarPlanner(grid,[2 2],[9 9]);

assert(~isempty(path));
assert(isequal(path(1,:),[2 2]));
assert(isequal(path(end,:),[9 9]));
assert(all(arrayfun(@(i) grid(path(i,1),path(i,2)) == 0, 1:size(path,1))));
disp('testAstar passed.');
