function map = generateMaze(rows, cols)
    map = ones(rows, cols);
    % 使用随机生成迷宫的算法
    % 这里使用一个简单的随机障碍物生成器
    for i = 1:rows
        for j = 1:cols
            if rand < 0.3 % 30%的几率设置为障碍物
                map(i, j) = 0;
            end
        end
    end
    % 保证入口和出口是通道
    map(1, 1) = 1;
    map(rows, cols) = 1;
end
