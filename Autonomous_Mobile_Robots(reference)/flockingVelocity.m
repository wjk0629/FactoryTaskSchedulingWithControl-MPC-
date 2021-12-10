function v_o = flockingVelocity(p, v, info)
% info == 1 : Á¢±Ù
% info == 2 : 
    closeEnough = 1;
    krep = 10; kalign = 1; kcoh = 1;
    V = 1;
    N = size(p,1);
    v_o = zeros(N, 2);
    all_vec = 1:N;
    
    for i = 1:N
        d = getDistance(p(:,1:2), p(i,1:2));
        
        if(info == 1)
            jj = [1:i-1 i+1:N];
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



