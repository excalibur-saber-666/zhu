function [dis_measure] = simu_dis(dis_true,dis_err)

[m,n] = size(dis_true);
dis_measure = dis_true + dis_err * randn(m,n);