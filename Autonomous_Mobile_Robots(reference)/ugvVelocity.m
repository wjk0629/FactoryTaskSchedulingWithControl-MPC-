function v_o = ugvVelocity(p, v, n, info)
% info == 1 : 접근
% info == 2 : 
    closeEnough = 0.5; % if close to node within 0.5m : mission clear 
    krep = 10; kalign = 1; kcoh = 1;
    N = size(n,1);
    V = 0.5; % robot velocity
    R = size(p,1); % ini robot position 0
    v_o = zeros(R, 2); % ini robot velo 0
    all_vec = 1:R;
    
    for i = 1:N % 모든 노드를 검색해서
        if n(i,3) == 1 % 한 노드에 콜이 들어왔다
            d = getDistance(p(:,1:2), n(i,1:2)); % 거리를 구한다
            robot_choiced = find(min(d)); % 젤 가까운 로봇을 고른다
            if p(robot_choiced,4) < 3 % 그 로봇이 capable 하다면
                %보낸다
            else % capable하지 않다면
                d(robot_choiced) = [];
            end
            
                
        
        if(info == 1)
            jj = [1:i-1 i+1:R];
        elseif(info==2)
            [~, i_sort] = sort(d, 'descend');
            jj = all_vec(i_sort(1:5));
        elseif(info==3)
            modi = mod(i,4);
            jj = find(mod(all_vec, 4) == modi);
            jj = setxor(jj, i);
        end
        
        repulsion = zeros(1,2);
        alignment = zeros(1,2);
        cohesion =  zeros(1,2);
        for j=jj
            r = p(j,1:2) - p(i,1:2);
            if(d(j) < closeEnough)
                % separation
                repulsion = repulsion - krep*r/d(j);
            else
                % alignment
                alignment = alignment + kalign*v(j,1:2);
                % cohesion
                cohesion = cohesion + kcoh*p(j,1:2);
            end
        end            
        alignment = alignment/length(jj) - v(i,:);
        cohesion = cohesion/length(jj) - p(i,1:2);
        v_o(i,:) = repulsion + alignment + cohesion;
        v_o(i,:) = V * v_o(i,:)/norm(v_o(i,:));
    end
end



