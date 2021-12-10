%%
close all

n = [0, 0; 1, 0];
m = [4, 4; 8, 4;8, 8];

figure; hold on;
    for i=1:size(n,1)
        %plot( n(i,1), n(i,2), 'o', 'Color', 'b', ...,'MarkerSize', 12);
        
    end
    %figure 1
    for i=1:size(m,1)
        %plot(m(i,1), m(i,2), 'S', 'Color', 'k' );
        text(m(i,1), m(i,2), num2str(i) );
    end
    xlabel('X [m]');
    ylabel('Y [m]');
    axis equal

    [totalobj , path] = RPC(n(1,:), m);
    title(['Robot 2 RPC: ' num2str( totalobj )])

%% figure 2
% robotPaths(n,m);

%% figure 3
n = [0, 0; 1, 0];
m = [4, 4; 4, 8; 4, 12; 8, 4; 8, 8; 8, 12];

robotPaths(n,m);

%% figure 4
% n = randi(1000, 10, 2);
% m = randi(2000, 20, 2);
% 
% robotPaths(n,m);