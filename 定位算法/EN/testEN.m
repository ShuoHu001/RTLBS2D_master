% 示例数据
beacons = [181.42,139.278; 181.42,33; -1.22,139.278; -1.22,33;-1.22,179.188;280.64,179.188]; % 基站位置
aoa = [3.55926; 2.73992; 5.56569; 0.714025;5.56872;3.85169]; % 到达角度 (弧度)
weight=[0.98407;1;0.999868;1;0.889814;0.889814];

% 正则化权重
lambda1 = 0.9;
lambda2 = 0;

% 调用AOA弹性网络回归函数
position_estimate = AOA_ElasticNet(beacons, aoa, lambda1, lambda2, weight);
disp('估计的目标位置:');
disp(position_estimate);