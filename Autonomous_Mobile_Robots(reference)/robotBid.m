function [target, bid] = robotBid(pose, assigned, unassigned)
    tot_dist = 0;
    for i=1:size(assigned,1)
        tot_dist = tot_dist + norm(pose-assigned(i,:));
        pose = assigned(i,:);
    end
    min_d = 1e6; 
    for i = 1:size(unassigned,1)
        d = norm(pose-unassigned(i,:));
        if(d<min_d)
            min_d = d;
            target = unassigned(i,:);
        end
    end
    bid = min_d + tot_dist;
end