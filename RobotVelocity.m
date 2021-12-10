function [p_o, v_o, n_o] = RobotVelocity(p, v, n, g)

    closeEnough = 0.5;
    V = 1;
    R = size(p,1);
    N = size(n,1);
    v_o = v;
    p_o = p;
    n_o = n;
    for i = 1:N
        if (n(i,3) == 1)
                % 나중에 getDistance를 Astar로 바꿀것           
                d_n = getDistance(p(:,1:2), n(i,1:2));
                d_g = getDistance(p(:,1:2), [10 10]);
                distance_score = sort(d_n);
                for k = 1:R
                    b = find(d_n == distance_score(k));
                    a = d_n(b);
                    
                    if (p(k,4) == i && a > closeEnough)
                        v_o(k,:) = kinematicVelo(p(k,:),n(i,:));
                        break;
                    
                                            
                    elseif (p(b,4) == 0 && a > closeEnough)
                        % go to node
                        % 여기에 MPC 들어가야함
                        v_o(b,:) = kinematicVelo(p(b,:),n(i,:));
                        p_o(b,4) = i;
                        break;
                        
                    elseif (p(b,4) == i && a <= closeEnough)
                        % go to goal
                        v_o(b,:) = kinematicVelo(p(b,:),g);
                        p_o(b,4) = -1;
                        n_o(i,3) = 0;
                    elseif (p(b,4) == -1 && d_g(b) <= closeEnough)
                        %go to base
                        v_o(b,:) = kinematicVelo(p(b,:),[0 0 0]);
                        p_o(b,4) = 0;
                    end
                end
        end
        
        if (n(i,3) == 0)
            for k = 1:R
                if p(k,4) == i && a > closeEnough
                    v_o(k,:) = kinematicVelo(p(k,:),n(i,:));
                elseif (p(k,4) == -1 && d_g(b) > closeEnough)
                    % keep going to goal
                    v_o(b,:) = kinematicVelo(p(b,:),g);
                end
            end
        end
        
    end
end



