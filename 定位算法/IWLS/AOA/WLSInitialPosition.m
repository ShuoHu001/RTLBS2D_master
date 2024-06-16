function [init_position] = WLSInitialPosition(beacons, aoa, weight)
    % WLSInitialPosition 使用加权最小二乘法计算初始位置
    % beacons 传感器位置
    % aoa 角度值
    % weight 权重矩阵
    N = size(beacons, 1);
    A = zeros(N, 2);
    b = zeros(N, 1);

    for i = 1:N
        A(i, :) = [sin(aoa(i)), -cos(aoa(i))];
        b(i) = beacons(i, 1) * sin(aoa(i)) - beacons(i, 2) * cos(aoa(i));
    end

    % 加权最小二乘求解
    W = weight; % 权重矩阵
    init_position = (A' * W * A) \ (A' * W * b);
end