close all
clear 
clc

% 인풋으로 필요 : 로봇 위치 벡터, 타겟 위치 벡터, 총 시간? 
% 파라미터 설정 : 로봇 속도,,타겟이 콜을 할 시간..?

%% number of robots , nodes , and total time set
R = 3; % number of robots
N = 5; % number of nodes(targets)
ts = 0.1; T = 80; e=0.1; % time set to simulation
g = [15 15 0]; 
Simlength=(T/ts);  
%% robot initial pose(x,y,theta) and velo(v,w)
p0 = zeros(R,4); % robot initial pose and status(x,y,theta,status)
v0 = zeros(R,2); % robot initial velo (v,w)
%v0(1,:) = [1 0];
for i = 1:R    
    while 1
        p0(i,:) = [rand, rand, atan2(v0(i,2), v0(i,1)), 0];
        d = getDistance(p0(1:i-1,1:2), p0(i,1:2));
        if( all(d>0.2)) % robot의 초기 위치는 서로 20cm이상 떨어지게
            break
        end
    end
end    
    
%% node initial position ( random )
node_distance = 1; % distance of node to node >= 1m 나중에 적용할 것
n0 = zeros(N,3);
% for i = 1:N
%     n0(i,:) = [rand*10, rand*10];
% end
for i = 1:N
    while 1
        n0(i,:) = [rand*10+2, rand*10+2, 0];
        d = getDistance(n0(1:i-1,1:2), n0(i,1:2));
        d_g = getDistance(n0(1:i-1,1:2),g(1:2));
        if( all(d>node_distance))
            if( all(d_g>node_distance))
                
                break
            end
            n0(i-1,:) = [rand*10+1, rand*10+1, 0];
            n0(i,:) = [rand*10+1, rand*10+1, 0];
        end
    end
end


%% plot initial
figure;
p = p0; v=v0; n = n0;
q = [p(1:R,1:R) zeros(R,4)];
r_plot_pos = plot(p(:,1), p(:,2), 'bo', 'MarkerSize', 4 ); 
hold on; xlabel('x [m]'); ylabel('y [m]');
r_plot_vel = quiver(p(:,1), p(:,2), v(:,1), v(:,2), 0.1, 'b--'); 
% quiver 하면 안보인다...(초기속도 0이니까..)
title('factory simulation - start . . .');
axis equal
axis([-2 16 -2 16]);
n_plot = plot(n0(:,1),n0(:,2),'go','MarkerSize',8);
g_plot = plot(g(1),g(2),'ro','MarkerSize',12);
%pause

index=1;
t_record = zeros(length(n),1);
p_vec = zeros(size(p,1), size(p,2), ceil(T/ts)); % p에 벡터 할당
p_vec(:,:,index) = p;
n_vec = zeros(size(n,1), size(n,2), ceil(T/ts));
n_vec(:,:,index) = n;
Record = VideoWriter('test.avi');
open(Record);
for t = 1:Simlength
    index = index + 1;
    %node call set
    [n,t_record] = nodeCall(n,t_record);
    %robot velocity set
    [p, v, n] = RobotTaskAllocation_test(p, v, n, g);
    %[p, v, n] = RobotTaskAllocation(p, v, n, g);
    
%     for j=1:R
%         V = v(j,1);
%         if v(j,2) < 0
%             disp('minus!!!!!!!!!!!!')
%         end
%            
%         W = v(j,2);
%         x_k = integratePosi(p(j,:),[V;W]*ts);
%         %x_k = integrateOdom(p(j,:), [V;W]*ts);
%         p(j,1:3) = x_k(1:3,end)';
%     end
    
    p
    
    delete(r_plot_pos); delete(r_plot_vel); delete(n_plot);
    r_plot_pos = plot(p(:,1), p(:,2), 'bo', 'MarkerSize', 4); 
    % plot(p(:,1), p(:,2), 'g.'); % 궤적 표시하는거 정신없어서 지움 
    r_plot_vel = quiver(p(:,1), p(:,2), v(:,1).*cos(p(:,3)), v(:,1).*sin(p(:,3)), 0.3, 'b--');
    n_plot = [];
    delete(n_plot);
    for i = 1:N
        if n(i,3) == 0 || n(i,3) == -2
            n_plot(i) = plot(n0(i,1),n0(i,2),'go','MarkerSize',8);
        else
            n_plot(i) = plot(n0(i,1),n0(i,2),'g*','MarkerSize',8);
        end
    end
    title(['Factory simulation - ' num2str(t*ts) '[sec]'])
    p_vec(:,:,i) = p;
    n_vec(:,:,i) = n;
    drawnow;
    frame = getframe(gca);
    writeVideo(Record,frame);
end

close(Record);