%%

close all
clear 
clc

N = 3;

% v0 = [(rand(N,1)-0.5)*1, (rand(N,1)-0.5)*1];
v0 = zeros(N,2);
v0(1,:) = [1 0];

p0 = zeros(N,3);

for i = 1:N        
    if(norm(v0(i,:))~=0)
        v0(i,:) = v0(i,:)/norm(v0(i,:));
    end
    while 1
        p0(i,:) = [rand*10, rand*10, atan2(v0(i,2), v0(i,1))];
        d = getDistance(p0(1:i-1,1:2), p0(i,1:2));
        if( all(d>0.5))
            break
        end
    end
end

p = p0; v=v0;

figure;
h1 = plot(p(:,1), p(:,2), 'bo', 'MarkerSize', 14 ); 
hold on; xlabel('x [m]'); ylabel('y [m]');
h2 = quiver(p(:,1), p(:,2), v(:,1), v(:,2), 0.1, 'b--');
title('flocking simulation - full info');
axis equal
axis([0 10 0 10]);
pause

ts = 0.05; T = 100; e=0.1;
i=1;
p_vec = zeros(size(p,1), size(p,2), ceil(T/ts));
p_vec(:,:,i) = p;

for t = 0:ts:T
    i=i+1;
    v = flockingVelocity(p, v, 1);
    % integration
%     p = p(:,1:2) + ts*v;
    for j=1:N
        [V, W] = feedbackLin(v(j,1), v(j,2), p(j,3), e);
        [V, W] = limitCmds(V,W,1,0.13);
        x_k = integrateOdom(p(j,:), [V;W]*ts);
        p(j,:) = x_k(:,end)';
    end
    
    delete(h1); delete(h2);
    h1 = plot(p(:,1), p(:,2), 'bo', 'MarkerSize', 14); 
    plot(p(:,1), p(:,2), 'g.'); 
    h2 = quiver(p(:,1), p(:,2), v(:,1), v(:,2), 1, 'b--');
    title(['flocking simulation - full info ' num2str(t) '[sec]'])
    p_vec(:,:,i) = p;
    
%     for j=size(p_vec,3):-1:
%     end
    
    drawnow; %pause(0.1); 
%     cla
end

p = p0; v=v0;

figure;
h1 = plot(p(:,1), p(:,2), 'bo', 'MarkerSize', 14); 
hold on; xlabel('x [m]'); ylabel('y [m]');
h2 = quiver(p(:,1), p(:,2), v(:,1), v(:,2), 0.5, 'b--');
title('flocking simulation - info from 5 closest');
axis equal
axis([0 10 0 10]);

i=1;

for t = 0:ts:T
    i=i+1;
    v = flockingVelocity(p, v, 2);
    % integration
%     p = p(:,1:2) + ts*v;
    for j=1:N
        [V, W] = feedbackLin(v(j,1), v(j,2), p(j,3), e);
        [V, W] = limitCmds(V,W,1,0.13);
        x_k = integrateOdom(p(j,:), [V;W]*ts);
        p(j,:) = x_k(:,end)';
    end
    
    delete(h1); delete(h2);
    h1 = plot(p(:,1), p(:,2), 'bo', 'MarkerSize', 14); 
    plot(p(:,1), p(:,2), 'g.'); 
    h2 = quiver(p(:,1), p(:,2), v(:,1), v(:,2), 1, 'b--');
            
    pause(0.1); 
%     cla
end


p = p0; v=v0;

figure;
h1 = plot(p(:,1), p(:,2), 'bo', 'MarkerSize', 14);
hold on; xlabel('x [m]'); ylabel('y [m]');
h2 = quiver(p(:,1), p(:,2), v(:,1), v(:,2), 0.5, 'b--');
title('flocking simulation - info mod 4');
axis equal
axis([0 10 0 10]);

i=1;

for t = 0:ts:T
    i=i+1;
    v = flockingVelocity(p, v, 3);
    % integration
%     p = p(:,1:2) + ts*v;
    for j=1:N
        [V, W] = feedbackLin(v(j,1), v(j,2), p(j,3), e);
        [V, W] = limitCmds(V,W,1,0.13);
        x_k = integrateOdom(p(j,:), [V;W]*ts);
        p(j,:) = x_k(:,end)';
    end
    
    delete(h1); delete(h2);
    h1 = plot(p(:,1), p(:,2), 'bo', 'MarkerSize', 14); 
    plot(p(:,1), p(:,2), 'g.'); 
    h2 = quiver(p(:,1), p(:,2), v(:,1), v(:,2), 1, 'b--');
    
    pause(0.1); 
%     cla
end

