function [p_o, v_o, n_o] = RobotTaskAllocation(p, v, n, g)

    closeEnough = 0.5;
    R = size(p,1);
    N = size(n,1);
    v_o = v;
    p_o = p;
    n_o = n;
    robot_procedure = [];
    for i = 1:R
        if (p(i,4) ~= 0 && p(i,4) ~= -1)
            distance_robotToNode = getDistance(p(:,1:2), n(p(i,4),1:2));
            if (distance_robotToNode(i) > closeEnough)
                v_o(i,:) = kinematicVelo(p(i,:),n(p(i,4),:));
            elseif (distance_robotToNode(i) <= closeEnough)
                v_o(i,:) = kinematicVelo(p(i,:),g(1:2));
                p_o(i,4) = -1;
                %n_o(i,3) = 0; % 이게 문제..
            end
        end
        if (p(i,4) == -1)
            distance_robotToGoal = getDistance(p(:,1:2), g(1:2));
            if (distance_robotToGoal(i) > closeEnough)
                v_o(i,:) = kinematicVelo(p(i,:),g(1:2));
            elseif (distance_robotToGoal(i) <= closeEnough)
                v_o(i,:) = kinematicVelo(p(i,:),[0 0]);
                p_o(i,4) = 0;
                n_o(i,3) = 0;
            end
        end
    end
    

    for j = 1:N
        % 처음 call이 들어온 노드라면
        if (n(j,3) == 1)
            % 나중에 getDistance를 Astar로 바꿀것           
            distance_robotToNode = getDistance(p(:,1:2), n(j,1:2));
            %distance_robotToGoal = getDistance(p(:,1:2), g(1:2));
            distance_score = sort(distance_robotToNode);
            for k = 1:R
                robot_procedure = [robot_procedure find(distance_robotToNode == distance_score(k))];
            end
                            
            for k = 1:length(robot_procedure)
                if (p(robot_procedure(k),4) == 0)
                    v_o(robot_procedure(k),:) = kinematicVelo(p(robot_procedure(k),:),n(j,:));
                    p_o(robot_procedure(k),4) = j;
                    n_o(j,3) = -1;
                    break;
                end
            end
            
            
        end
        % 어떤 로봇이 찜한 노드라면
%         if (n(i,3) == -1)
%             for j = 1:R
%                 distance_robotToNode = getDistance(p(:,1:2), n(i,1:2));
%                 distance_robotToGoal = getDistance(p(:,1:2), g(1:2));
%                 if (p(j,4) == i && distance_robotToNode(j) > closeEnough)
%                     v_o(j,:) = kinematicVelo(p(j,:),n(i,:));
%                 elseif (p(j,4) == i && distance_robotToNode(j) <= closeEnough)
%                     v_o(j,:) = kinematicVelo(p(j,:),g(1:2));
%                     p_o(j,4) = -1;
%                     %n_o(i,3) = 0; % 이게 문제..
%                 elseif (p(j,4) == -1 && distance_robotToGoal(j) > closeEnough)
%                     v_o(j,:) = kinematicVelo(p(j,:),g(1:2));
%                 elseif (p(j,4) == -1 && distance_robotToGoal(j) <= closeEnough)
%                     v_o(j,:) = kinematicVelo(p(j,:),[0 0]);
%                     p_o(j,4) = 0;
%                     n_o(i,3) = 0;
%                 end
%             end
%         end
        
    end
end



