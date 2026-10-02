function [posi_w_all,posi_L_all,dis_true] = distance_cal(posi_w_enu_all,posi_L_enu_all,posi_ini,uav_num,high_num)

posi_w_all = zeros(3,uav_num - high_num);
posi_L_all = zeros(3,high_num);
dis_true   = zeros(uav_num - high_num,uav_num);

for i = 1:uav_num - high_num
    posi_w_all(:,i) = posical_xyz(posi_w_enu_all(:,i),posi_ini);
end
for i = 1:high_num
    posi_L_all(:,i) = posical_xyz(posi_L_enu_all(:,i),posi_ini);
end

for i = 1:uav_num-high_num           %先求出真实的距离
    for j = 1:uav_num-high_num
        dis_true(i,j) = dis_cal(posi_w_all(:,i),posi_w_all(:,j));
    end
    for j = 1:high_num
            dis_true(i,uav_num-high_num+j) = dis_cal(posi_w_all(:,i),posi_L_all(:,j));
    end
end