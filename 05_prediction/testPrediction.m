%% Test the constant-velocity motion predictor
state = [2, 4, 0.5, -1];
predicted = motionPredictor(state, 3, 2);

assert(isequal(size(predicted), [4,2]));
assert(all(abs(predicted(1,:) - [2,4]) < 1e-9));
assert(all(abs(predicted(end,:) - [5,-2]) < 1e-9));

disp('testPrediction passed.');
