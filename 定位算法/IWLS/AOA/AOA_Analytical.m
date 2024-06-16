function [position_estimate] = AOA_Analytical(beacons, aoa)
    % AOA_Analytical 使用AOA解析解方法进行定位
    % beacons: 基站位置 (2x2 矩阵)
    % aoa: 基站到目标的到达角度 (2x1 向量，弧度)
    % 返回值: 估计位置 (1x2 向量)

    % 提取基站位置
    x1 = beacons(1, 1);
    y1 = beacons(1, 2);
    x2 = beacons(2, 1);
    y2 = beacons(2, 2);

    % 提取到达角度
    theta1 = aoa(1);
    theta2 = aoa(2);

    % 构建线性方程
    % 目标位置为 (x, y)
    A = [-sin(theta1), cos(theta1);
         -sin(theta2), cos(theta2)];
    b = [-sin(theta1) * x1 + cos(theta1) * y1;
         -sin(theta2) * x2 + cos(theta2) * y2];

    % 解析求解
    position_estimate = A\b;
end