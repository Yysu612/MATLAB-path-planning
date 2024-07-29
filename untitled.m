% Set up the map
start = [1, 1];
rows = 60;
cols = 60;

% generate the maze
map = generateMaze(rows, cols);

% randomly select nodes goal1 and goal2 while make sure they are tangible
goal1 = generateRandomPoint(map);
goal2 = generateRandomPoint(map);

% excute first path planning
[route1, cost1] = AStarPathPlanning(map, start, goal1);

% excute second path planning
[route2, cost2] = AStarPathPlanning(map, goal1, goal2);

% combine the path
route = [route1; route2(2:end, :)];
cost = cost1 + cost2;

% plot the result
figure;
imshow(map, 'InitialMagnification', 'fit');
hold on;
plot(route1(:,2), route1(:,1), 'g', 'LineWidth', 2); % paint the first path green
plot(route2(:,2), route2(:,1), 'r', 'LineWidth', 2); % paint the second path red
plot(start(2), start(1), 'bo', 'MarkerFaceColor', 'b'); % begining
text(start(2), start(1), ' beginning', 'Color', 'blue', 'FontSize', 10);
plot(goal1(2), goal1(1), 'ro', 'MarkerFaceColor', 'r'); % first terminal
text(goal1(2), goal1(1), ' goal1', 'Color', 'red', 'FontSize', 10);
plot(goal2(2), goal2(1), 'mo', 'MarkerFaceColor', 'm'); % second terminal
text(goal2(2), goal2(1), ' goal2', 'Color', 'magenta', 'FontSize', 10);
title('Results of Path Planning');
hold off;

% Path Planning function
function [route, cost] = AStarPathPlanning(map, start, goal)
    % Initialize
    [rows, cols] = size(map);
    openList = [];
    closedList = false(rows, cols);
    gCost = inf(rows, cols);
    fCost = inf(rows, cols);
    gCost(start(1), start(2)) = 0;
    fCost(start(1), start(2)) = heuristic(start, goal);
    openList = [openList; start, fCost(start(1), start(2))];

    % Orientation Vector (Perpendicular axis and diagnoal)
    directions = [0 1; 1 0; 0 -1; -1 0; -1 -1; -1 1; 1 -1; 1 1];

    while ~isempty(openList)
        % Select minimal fCost node
        [~, idx] = min(openList(:, 3));
        current = openList(idx, 1:2);
        openList(idx, :) = [];

        if all(current == goal)
            break;
        end

        closedList(current(1), current(2)) = true;

        % Process adjacent nodes
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

    % Polishing the Path
    route = [];
    current = goal;
    while ~all(current == start)
        route = [current; route];
        current = squeeze(parent(current(1), current(2), :))';
    end
    route = [start; route];
    cost = gCost(goal(1), goal(2));
end

% Heuristic function
function h = heuristic(node, goal)
    h = norm(node - goal);
end

% Generate maze function
function map = generateMaze(rows, cols)
    map = ones(rows, cols);
    % Apply random maze generating algorithm
    % Apply a obstacle generator
    for i = 1:rows
        for j = 1:cols
            if rand < 0.3 % 30% possibility to become obstacles
                map(i, j) = 0;
            end
        end
    end
    % Ensure begining and final goal nodes are tangible
    map(1, 1) = 1;
    map(rows, cols) = 1;
end

% Generate random point function
function point = generateRandomPoint(map)
    [rows, cols] = size(map);
    while true
        point = [randi(rows), randi(cols)];
        if map(point(1), point(2)) == 1
            break;
        end
    end
end
