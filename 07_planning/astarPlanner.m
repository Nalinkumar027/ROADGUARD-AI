function pathRC = astarPlanner(occupancy, startRC, goalRC)
%ASTARPLANNER Find an 8-connected path through a binary occupancy grid.
%   occupancy: matrix where 0 is free and nonzero is blocked.
%   startRC, goalRC: [row column].
%   pathRC: N-by-2 [row column] path, or [] if no path exists.
%
% This simple implementation is intended for learning and prototyping.

    validateattributes(occupancy, {'numeric','logical'}, {'2d','nonempty'});
    validateattributes(startRC, {'numeric'}, {'vector','numel',2,'integer'});
    validateattributes(goalRC, {'numeric'}, {'vector','numel',2,'integer'});

    rows = size(occupancy,1);
    cols = size(occupancy,2);
    startRC = double(startRC(:)');
    goalRC = double(goalRC(:)');

    if ~inside(startRC, rows, cols) || ~inside(goalRC, rows, cols)
        error('Start and goal must be inside the occupancy grid.');
    end
    if occupancy(startRC(1),startRC(2)) ~= 0 || occupancy(goalRC(1),goalRC(2)) ~= 0
        error('Start and goal cells must be free.');
    end

    gScore = inf(rows,cols);
    fScore = inf(rows,cols);
    cameFrom = zeros(rows,cols,2);
    openSet = false(rows,cols);
    closedSet = false(rows,cols);

    gScore(startRC(1),startRC(2)) = 0;
    fScore(startRC(1),startRC(2)) = heuristic(startRC,goalRC);
    openSet(startRC(1),startRC(2)) = true;

    moves = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];

    while any(openSet(:))
        candidates = find(openSet);
        [~,localIdx] = min(fScore(candidates));
        currentLinear = candidates(localIdx);
        [cr,cc] = ind2sub([rows,cols],currentLinear);
        current = [cr,cc];

        if isequal(current,goalRC)
            pathRC = reconstruct(cameFrom,current,startRC,rows,cols);
            return;
        end

        openSet(cr,cc) = false;
        closedSet(cr,cc) = true;

        for k = 1:size(moves,1)
            neighbor = current + moves(k,:);
            nr = neighbor(1); nc = neighbor(2);

            if ~inside(neighbor,rows,cols) || occupancy(nr,nc) ~= 0 || closedSet(nr,nc)
                continue;
            end

            % Prevent diagonal movement through blocked corner cells.
            dr = moves(k,1); dc = moves(k,2);
            if dr ~= 0 && dc ~= 0
                if occupancy(cr+dr,cc) ~= 0 || occupancy(cr,cc+dc) ~= 0
                    continue;
                end
            end

            stepCost = sqrt(double(dr)^2 + double(dc)^2);
            tentativeG = gScore(cr,cc) + stepCost;
            if tentativeG < gScore(nr,nc)
                cameFrom(nr,nc,:) = current;
                gScore(nr,nc) = tentativeG;
                fScore(nr,nc) = tentativeG + heuristic(neighbor,goalRC);
                openSet(nr,nc) = true;
            end
        end
    end

    pathRC = [];
end

function tf = inside(rc, rows, cols)
    tf = rc(1) >= 1 && rc(1) <= rows && rc(2) >= 1 && rc(2) <= cols;
end

function h = heuristic(a,b)
    h = hypot(double(a(1)-b(1)), double(a(2)-b(2)));
end

function path = reconstruct(cameFrom,current,startRC,rows,cols)
    path = current;
    while ~isequal(current,startRC)
        parent = squeeze(cameFrom(current(1),current(2),:))';
        if numel(parent) ~= 2 || all(parent == 0)
            path = [];
            return;
        end
        current = parent;
        path = [current; path]; %#ok<AGROW>
        if size(path,1) > rows*cols
            path = [];
            return;
        end
    end
end
