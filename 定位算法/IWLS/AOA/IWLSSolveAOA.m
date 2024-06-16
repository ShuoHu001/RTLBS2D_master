function [position_estimate] = IWLSSolveAOA(beacons, aoa, init_weight, max_iter, tol)
    % IWLSSolveAOA 使用加权最小二乘法计算初始位置，并使用迭代加权最小二乘法进行AOA定位
    % beacons: 基站位置 (Nx2 矩阵)
    % aoa: 基站到目标的到达角度 (Nx1 向量，弧度)
    % max_iter: 最大迭代次数
    % tol: 收敛阈值

    % 使用加权最小二乘法计算初始位置
    init_position = WLSInitialPosition(beacons, aoa, init_weight);
    W=init_weight;
    % 初始化
    position_estimate = init_position;
    N = size(beacons, 1);


    for iter = 1:max_iter
        % 计算当前估计位置到各基站的方向
        dx = position_estimate(1) - beacons(:, 1);
        dy = position_estimate(2) - beacons(:, 2);
        angles = atan2(dy, dx);
        angles = mod(angles, 2*pi); % 确保角度在0到2π范围内

        % 计算残差
        residuals = aoa - angles;
        residuals = mod(residuals + pi, 2*pi) - pi; % 确保残差在[-π, π]范围内
        residuals = diag(residuals.*(1.0 ./ W));               %残差乘以权重矩阵
        % 计算加权矩阵
        newWeight=1 ./ (1 + (residuals).^2);
        W = 0.1*W.*diag(newWeight); % 简单示例，权重与残差成反比,放缩权重矩阵


        % 构建雅可比矩阵
        distances = sqrt(dx.^2 + dy.^2);
        J = [-dy ./ distances, dx ./ distances];

        % 迭代更新位置估计
        delta = pinv(J' * W * J+1e-5*eye(size(J, 2))) * (J' * W * residuals);
        % if isnan(delta)
        %     break;
        % end
        position_estimate = position_estimate + delta;
        disp([iter,norm(delta)]);
        % 检查收敛
        if norm(delta) < tol
            break;
        end
    end
end
