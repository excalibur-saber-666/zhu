function [dis_measure,uav_link_num] = psedu_cal(dis_true,dis_err,uav_link_num,uav_num,high_num)

dis_measure = simu_dis(dis_true,dis_err);  %伪距生成
    
for i = 1:uav_num-high_num     %
    dis_measure(i:uav_num-high_num,i) = zeros(uav_num-high_num-i+1,1);   %把矩阵上三角化
end
dis_measure(1:uav_num-high_num,1:uav_num-high_num) = dis_measure(1:uav_num-high_num,1:uav_num-high_num) + dis_measure(1:uav_num-high_num,1:uav_num-high_num)';    %这样两个节点之间的测距值就统一了
    
for i = 1:uav_num-high_num
    for j = 1:uav_num
        if dis_true(i,j) <= 500 && i ~= j
             uav_link_num{1,i} = [uav_link_num{1,i},j];
        end
    end
end