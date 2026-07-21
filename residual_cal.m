function [residual,jaco] = residual_cal(posi_1,posi_2,dis)

%计算节点之间的残差和雅可比矩阵
%注意：这里的posi需要是以m为单位
H = zeros(1,6);
r = sqrt((posi_1 - posi_2)' * (posi_1 - posi_2));
H(1,1) =   (posi_1(1,1) - posi_2(1,1))/r;
H(1,2) =   (posi_1(2,1) - posi_2(2,1))/r;
H(1,3) =   (posi_1(3,1) - posi_2(3,1))/r;
H(1,4) = - (posi_1(1,1) - posi_2(1,1))/r;
H(1,5) = - (posi_1(2,1) - posi_2(2,1))/r;
H(1,6) = - (posi_1(3,1) - posi_2(3,1))/r;

residuals = r - dis;
residual = 1/sqrt(0.04) * residuals;      %乘以误差矩阵的平方根

jaco = 1/sqrt(0.04) * H;

end