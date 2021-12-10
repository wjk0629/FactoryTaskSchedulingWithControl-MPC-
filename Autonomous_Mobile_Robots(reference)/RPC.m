function [totdist, path] = RPC(pose, T)
    T = [pose; T];
    nStops = size(T,1);
    idxs = nchoosek(1:nStops,2);
    dist = hypot(T(idxs(:,1),1) - T(idxs(:,2),1), ...
                 T(idxs(:,1),2) - T(idxs(:,2),2) );
    lendist = length(dist);

    tsp = optimproblem;
    trips = optimvar('trips',lendist,1,'Type','integer','LowerBound',0,'UpperBound',1);
    tsp.Objective = dist'*trips;
    
    constrips = sum(trips) == nStops;
    tsp.Constraints.constrips = constrips;

    constr2trips = optimconstr(nStops,1);
    for stops = 1:nStops
        whichIdxs = (idxs == stops);
        whichIdxs = any(whichIdxs,2); % start or end at stops
        constr2trips(stops) = sum(trips(whichIdxs)) == 2;
    end
    tsp.Constraints.constr2trips = constr2trips;
    
    opts = optimoptions('intlinprog','Display','off','Heuristics','round-diving',...
                        'IPPreprocess','none');
    tspsol = solve(tsp,'options',opts);

    tours = detectSubtours(tspsol.trips,idxs);
    numtours = length(tours);
    % Index of added constraints for subtours
    k = 1;
    while numtours > 1 % repeat until there is just one subtour
        % Add the subtour constraints
        for ii = 1:numtours
            subTourIdx = tours{ii}; % Extract the current subtour
    %         The next lines find all of the variables associated with the
    %         particular subtour, then add an inequality constraint to prohibit
    %         that subtour and all subtours that use those stops.
            variations = nchoosek(1:length(subTourIdx),2);
            a = false(length(idxs),1);
            for jj = 1:length(variations)
                whichVar = (sum(idxs==subTourIdx(variations(jj,1)),2)) & ...
                           (sum(idxs==subTourIdx(variations(jj,2)),2));
                a = a | whichVar;
            end
            tsp.Constraints.(sprintf('subtourconstr%i',k)) = sum(trips(a)) <= length(subTourIdx)-1;
            k = k + 1;
        end
        % Try to optimize again
        [tspsol,fval,exitflag,output] = solve(tsp,'options',opts);

        % How many subtours this time?
        tours = detectSubtours(tspsol.trips,idxs);
        numtours = length(tours); % number of subtours
%         fprintf('# of subtours: %d\n',numtours);
    end

    segments = find(round(tspsol.trips));

    path = [];
    for ii = 1:length(segments)
        path = [path; idxs(segments(ii),1)];
        plot([T(idxs(segments(ii),1),1), T(idxs(segments(ii),2),1)], ...
             [T(idxs(segments(ii),1),2), T(idxs(segments(ii),2),2)], 'k--', ...
             'LineWidth', 2);
    end

    totdist = dist'*tspsol.trips;
end

function subTours = detectSubtours(x,idxs)
% Returns a cell array of subtours. The first subtour is the first row of x, etc.

%   Copyright 2014 The MathWorks, Inc. 

x = round(x); % correct for not-exactly integers
r = find(x); % indices of the trips that exist in the solution
substuff = idxs(r,:); % the collection of node pairs in the solution
unvisited = ones(length(r),1); % keep track of places not yet visited
curr = 1; % subtour we are evaluating
startour = find(unvisited,1); % first unvisited trip
    while ~isempty(startour)
        home = substuff(startour,1); % starting point of subtour
        nextpt = substuff(startour,2); % next point of tour
        visited = nextpt; unvisited(startour) = 0; % update unvisited points
        while nextpt ~= home
            % Find the other trips that starts at nextpt
            [srow,scol] = find(substuff == nextpt);
            % Find just the new trip
            trow = srow(srow ~= startour);
            scol = 3-scol(trow == srow); % turn 1 into 2 and 2 into 1
            startour = trow; % the new place on the subtour
            nextpt = substuff(startour,scol); % the point not where we came from
            visited = [visited,nextpt]; % update nodes on the subtour
            unvisited(startour) = 0; % update unvisited
        end
        subTours{curr} = visited; % store in cell array
        curr = curr + 1; % next subtour
        startour = find(unvisited,1); % first unvisited trip
    end
end