function [route, cost] = AStarPathPlanning(map, start, goal)
    % Initialization
    [rows, cols] = size(map);
    openList = [];
    closedList = false(rows, cols);
    gCost = inf(rows, cols);
    fCost = inf(rows, cols);
    gCost(start(1), start(2)) = 0;
    fCost(start(1), start(2)) = heuristic(start, goal);
    openList = [openList; start, fCost(start(1), start(2))];

    % Direction Vector (上下左右及对角线)
    directions = [0 1; 1 0; 0 -1; -1 0; -1 -1; -1 1; 1 -1; 1 1];

    while ~isempty(openList)
        % 选择 fCost 最小的节点
        [~, idx] = min(openList(:, 3));
        current = openList(idx, 1:2);
        openList(idx, :) = [];

        if all(current == goal)
            break;
        end

        closedList(current(1), current(2)) = true;

        % 处理邻居节点
        for i = 1:size(directions, 1)
            neighbor = current + directions(i, :);
            if neighbor(1) > 0 && neighbor(1) <= rows && neighbor(2) > 0 && neighbor(2) <= cols
                if map(neighbor(1), neighbor(2)) == 1 && ~closedList(neighbor(1), neighbor(2))
                    tentative_gCost = gCost(current(1), current(2)) + norm(current - neighbor);
                    if tentative_gCost < gCost(neighbor(1), neighbor(2))
                        gCost(neighbor(1), neighbor(2)) = tentative_gCost;
                        fCost(neighbor(1), neighbor(2)) = tentative_gCost + heuristic(neighbor, goal);
                        openList = [openList; neighbor, fCost(neighbor(1), neighbor(2))];
                        parent(neighbor(1), neighbor(2), :) = current;
                    end
                end
            end
        end
    end

    % 重建路径
    route = [];
    current = goal;
    while ~all(current == start)
        route = [current; route];
        current = squeeze(parent(current(1), current(2), :))';
    end
    route = [start; route];
    cost = gCost(goal(1), goal(2));
end

function h = heuristic(node, goal)
    h = norm(node - goal);
end
