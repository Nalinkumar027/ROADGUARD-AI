function predictedXY = motionPredictor(state, numSteps, dt)
%MOTIONPREDICTOR Constant-velocity prediction for one tracked object.
%   state = [x y vx vy]; numSteps is a nonnegative integer; dt > 0.
%   Returns (numSteps+1)-by-2 positions, including the current position.
%   This is a baseline predictor, not a learned trajectory model.

    validateattributes(state, {'numeric'}, {'vector','numel',4,'finite'});
    validateattributes(numSteps, {'numeric'}, {'scalar','integer','nonnegative','finite'});
    validateattributes(dt, {'numeric'}, {'scalar','positive','finite'});

    state = double(state(:)');
    t = (0:numSteps)' * double(dt);
    predictedXY = [state(1) + state(3).*t, state(2) + state(4).*t];
end
