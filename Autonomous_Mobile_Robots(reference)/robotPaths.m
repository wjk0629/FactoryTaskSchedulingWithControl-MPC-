function robotPaths(robots, tasks)
    c = distinguishable_colors(size(robots,1));
    allocations = taskAllocation(robots, tasks);
    totalobj  = 0;
    
    figure; hold on;
    
    for i=1:size(allocations,2)
        plot( robots(i,1), robots(i,2), 'o', 'Color', c(i,:), ...
            'MarkerSize', 12);
        if(~isempty(allocations(i).task))
            plot( [robots(i,1); allocations(i).task(:,1)], ...
                  [robots(i,2); allocations(i).task(:,2)], ...
                   'Color', c(i,:), 'LineWidth', 2);        
               totalobj = totalobj +allocations(i).bid;
        end
    end
    
    for i=1:size(tasks,1)
        plot(tasks(i,1), tasks(i,2), 'S', 'Color', 'k' );
    end
    xlabel('X [m]');
    ylabel('Y [m]');
    axis equal
    title(['Team objective: ' num2str( totalobj )])
end