function [position_estimate] = AOA_ElasticNet(beacons, aoa, lambda1, lambda2, weights)
    % AOA_ElasticNet_Weighted 使用带权重的弹性网络回归进行定位
    % beacons: 基站位置 (m x 2 矩阵)
    % aoa: 基站到目标的到达角度 (m x 1 向量，弧度)
    % lambda1: L1正则化项的权重
    % lambda2: L2正则化项的权重
    % weights: 每个样本的权重 (m x 1 向量)
    % 返回值: 估计位置 (1 x 2 向量)

    % 基站数量
    m = size(beacons, 1);

    % 构建矩阵A和向量b
    A = zeros(m, 2);
    b = zeros(m, 1);
    for i = 1:m
        phi = aoa(i);
        x_i = beacons(i, 1);
        y_i = beacons(i, 2);
        A(i, :) = [sin(phi), -cos(phi)];
        b(i) = x_i * sin(phi) - y_i * cos(phi);
    end

    % 构建权重矩阵 W
    W = diag(weights);

    % 初始化位置估计
    w_init = [0; 0];  % 初始值可以设为零向量

    % 设置优化选项
    options = optimoptions('fminunc', 'Algorithm', 'quasi-newton', 'Display','iter');

    % 定义损失函数
    loss_func = @(w) (A * w - b)' * W * (A * w - b) + lambda1 * norm(w, 1) + lambda2 * norm(w, 2)^2;

    % 使用fminunc进行优化
    position_estimate = fminunc(loss_func, w_init, options);
end