function allocations = taskAllocation(robots, targets)
    bids = zeros(size(robots,1),1);
    tmptargets = zeros(size(robots,1),2);
    for i = 1:size(robots,1)
        allocations(i).bid=0;
        allocations(i).task=[];
    end
    while ~isempty(targets)
        for i = 1:size(robots,1)
            [target, bid] = robotBid(robots(i,:), allocations(i).task, targets);
            bids(i) = bid;
            tmptargets(i,:) = target;
        end

%         [winner_bid, idx] = min(bids);
        winner_bid = min(bids);
        idx = find(bids == winner_bid);
        idx = idx(floor( rand*size(idx,1) + 1 ));

        allocations(idx).bid = winner_bid;
        allocations(idx).task = [allocations(idx).task; tmptargets(idx, :)];
        
        targets = setxor(targets,tmptargets(idx, :),'rows');
    end
end