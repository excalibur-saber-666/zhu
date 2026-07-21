%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%   基于因子图和卡尔曼滤波的混合式协同导航方法
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
for monte =1:1
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------固定参数设置----------------%
deg_rad=0.01745329252e0;% Transfer from angle degree to rad
g=9.7803698;         %重力加速度    （单位：米/秒/秒）
Re=6378137.0;           %地球半径（米） 
f=1/298.257;            %地球的椭圆率

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------仿真时间设置---------------%
%%
t = 0;
T = 0.02;  %惯导更新频率
T_D = 1;   %每次进入优化的时间以及卡尔曼滤波周期
T_M = 0;
t_stop = 600;  %仿真总时长
%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------初始位置生成----------%
%%
uav_num = 5; high_num =3;
% [x,y,z]=sampling(0,600,0,600,100,1000,uav_num,100);
load ('posi_e_all.dat');load ('posi_n_all.dat');load ('posi_u_all.dat');

% figure_num = 1;
% figure(figure_num);
% plot3(x,y,z,'o');grid;
% 
% save posi_e_all.dat x -ASCII;
% save posi_n_all.dat y -ASCII;
% save posi_u_all.dat z -ASCII;
posi_ini = [118;32;200.0];    %代表[0;0;0]所在的经纬高
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----------------转换为经纬度表示--------------%
posi_w_all = zeros(3,uav_num - high_num); posi_w_enu_all = zeros(3,uav_num - high_num);
posi_L_all = zeros(3,high_num); posi_L_enu_all = zeros(3,high_num);
posi_w_graph = zeros(3,uav_num - high_num); posiN_w_graph = zeros(3,uav_num - high_num);
posi_L_graph = zeros(3,high_num);

for i = 1:(uav_num - high_num)
        posi_w_i = [posi_e_all(1,i);posi_n_all(1,i);posi_u_all(1,i)];
        posi_w_enu = posical_enu(posi_w_i,posi_ini);
        
        posi_w_all(:,i) = posi_w_i;
        posi_w_enu_all(:,i) = posi_w_enu;
end
for i = 1:high_num
   posi_L_all(:,i) =  [posi_e_all(1,uav_num - high_num + i);posi_n_all(1,uav_num - high_num + i);posi_u_all(1,uav_num - high_num + i)];
   posi_L_enu_all(:,i) = posical_enu(posi_L_all(:,i),posi_ini);
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%------------------------声明导航参数----------------%
%需要注意的是：如果变量包含的是所有无人机的状态，那么僚机在前，长机在后
veloB_all = zeros(3,uav_num);         
velo_all = zeros(3,uav_num);
atti_all = zeros(3,uav_num);    %所有机群速度和姿态真值

veloN_all = zeros(3,uav_num - high_num); attiN_all = zeros(3,uav_num - high_num);  %僚机群速度和姿态解算值
WnbbA_old = zeros(3,uav_num - high_num);  %角速度积分输出
atti_rate_all = zeros(3,uav_num);  %横滚速率、俯仰速率
acceB_all = zeros(3,uav_num);  %加速度
%如果初始速度和运动轨迹都一样，那么角速度和加速度的真值可以设置成一样的
%%IMU输出%%
Wibb=zeros(3,1);    %机体系陀螺仪输出   （单位：度/秒）
Fb=zeros(3,1);      %机体系加速度计输出 （单位：米/秒/秒）

Wibb_noise = zeros(3,uav_num-high_num); %带噪声的陀螺仪输出
Fb_noise = zeros(3,uav_num-high_num);  %带噪声的加速度计输出

Gyro_b=zeros(3,uav_num-high_num);  % 陀螺随机常数（弧度/秒）
Gyro_r=zeros(3,uav_num-high_num);  % 陀螺一阶马尔可夫过程（弧度/秒）
Gyro_wg=zeros(3,uav_num-high_num); %陀螺白噪声（弧度/秒）
Acc_r =zeros(3,uav_num-high_num);  % 加速度一阶马尔可夫过程（米/秒/秒）

%%GPS仿真输出%%
posiG_w_all = zeros(3,uav_num-high_num);
posiG_L = zeros(3,high_num); %高精度锚机的GPS输出
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------卡尔曼滤波参数-----------------%
Acc_modi_all = zeros(3,uav_num-high_num);  %加速度计误差修正值
Gyro_modi_all = zeros(3,uav_num-high_num);     %陀螺误差修正值
for i = 1:uav_num-high_num
    Xc = zeros(18,1);    %系统的状态量
    Xc_all{1,i} = Xc;
    PK = zeros(18,18);   %状态协方差阵
    PK_all{1,i} = PK;
    Xerr = zeros(1,18);  %状态估计量的误差值
    Xerr_all{1,i} = Xerr;
end

%%
%-------------------因子图参数--------------------%
cov_graph = zeros(3,uav_num-high_num);

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------IMU参数赋值---------%
for i = 1:uav_num-high_num
    [Gyro_b(:,i),Gyro_r(:,i),Gyro_wg(:,i),Acc_r(:,i)]=imu_err_random(t,T,Gyro_b(:,i),Gyro_r(:,i),Gyro_wg(:,i),Acc_r(:,i));
end

%--------------初始导航参数赋值--------------%
kc = 0;
for i = 1:uav_num
    veloB_all(2,i) = 5;   %设置初始速度，机头方向
    atti_all(:,i) = [0;0;90];  %设置初始航向
end
attiN_all = atti_all(:,1:uav_num-high_num);

old_veloB_all = veloB_all;
old_atti_all = atti_all;

for i = 1:uav_num
    velo_all(:,i) = veloN0(atti_all(:,i),veloB_all(:,i));  %所有无人机在东北天坐标系下的速度真值
end
for i = 1:uav_num-high_num
    veloN_all(:,i) = veloN0(attiN_all(:,i),veloB_all(:,i));
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
w_flag = 1; L_flag = 2;   % w_flag代表的僚机
for i = 1:uav_num-high_num
    posiG_w_all(:,i) = simu_gps(posi_w_enu_all(:,i),w_flag); 
end

posiN_w_all = posi_w_enu_all;    %首先赋值为真值
%卡尔曼滤波器的初始化
for i = 1:uav_num-high_num
    Xc = Xc_all{1,i};
    PK = PK_all{1,i};
    Xerr = Xerr_all{1,i};
    [Xc,PK,Xerr] = kalm_factor_init(posiN_w_all(:,i),Xc,PK,Xerr);
    Xc_all{1,i} = Xc;
    PK_all{1,i} = PK;
    Xerr_all{1,i} = Xerr;
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = 0; 
%数据记录

TraceData = zeros(t_stop/T,uav_num*3+1);    %记录所有无人机的真实位置，前十项为时间和长机位置，后面是僚机位置
TraceData_xyz = zeros(t_stop/T,uav_num*3+1);  %记录所有僚机的真实位置，以[0；0；0]为原点
SinsData = zeros(t_stop/T,(uav_num-high_num)*3 + 1);  %记录所有僚机的惯导解算位置
SinsData_xyz = zeros(t_stop/T,(uav_num-high_num)*3 + 1);  %记录所有僚机的惯导解算位置，以[0；0；0]为原点
AttiData = zeros(t_stop/T,uav_num*3+1);  %记录所有无人机的真实姿态
AttiNData = zeros(t_stop/T,(uav_num-high_num)*3 + 1);  %记录所有僚机的解算姿态
GPSData = zeros(t_stop/T_D,(uav_num-high_num)*3 + 1);
VeloData = zeros(t_stop/T,uav_num*3+1);  %记录所有无人机的真实速度
VeloNData = zeros(t_stop/T,(uav_num-high_num)*3 + 1);  %记录所有僚机的解算速度
GPSerrData = zeros(t_stop/T_D,(uav_num-high_num)*3 + 1);
GraphData = zeros(t_stop/T_D,(uav_num-high_num)*3 + 1);   %记录僚机的因子图解算值

TraceData(1,1) = t;
for i = 1:high_num
    TraceData(1,3*i-1:3*i+1) = [posi_L_enu_all(:,i)'];
end
TraceData_xyz(1,1) = t; SinsData_xyz(1,1) = t;
SinsData(1,1) = t; AttiNData(1,1) = t; VeloNData(1,1) = t;
for i = 1:uav_num-high_num
    TraceData(1,(3*(i+high_num)-1):(3*(i+high_num)+1)) = posi_w_enu_all(:,i)';
    SinsData(1,(i-1)*3+2:(i-1)*3+4) = posiN_w_all(:,i)';
    AttiNData(1,(i-1)*3+2:(i-1)*3+4) = attiN_all(:,i)';
    VeloNData(1,(i-1)*3+2:(i-1)*3+4) = veloN_all(:,i)';
    TraceData_xyz(1,(i-1)*3+2:(i-1)*3+4) = posi_w_all(:,i)';
    SinsData_xyz(1,(i-1)*3+2:(i-1)*3+4) = posi_w_all(:,i)';
    GraphData(1,(i-1)*3+2:(i-1)*3+4) = posi_w_all(:,i)';
end

AttiData(1,1) = t; VeloData(1,1) = t;
for i=1:uav_num
    AttiData(1,(i-1)*3+2:(i-1)*3+4) = atti_all(:,i)';
    VeloData(1,(i-1)*3+2:(i-1)*3+4) = velo_all(:,i)';
end
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dis_true = zeros(uav_num-high_num,uav_num);    %
for i = 1:uav_num-high_num
    for j = 1:uav_num-high_num
        dis_true(i,j) = dis_cal(posi_w_all(:,i),posi_w_all(:,j));
    end
    for j = 1:high_num
        dis_true(i,uav_num-high_num+j) = dis_cal(posi_w_all(:,i),posi_L_all(:,j));
    end
end

%仿真正式开始

k_sins = 1;
kc=1;
kc_kal = 0;
while t <=t_stop
    
    t = t + T;
    
     if(t>=kc*50-T && t<kc*50)
        kc=kc+1;
        disp(t);
     end
     %控制显示
     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%
    %集群航迹发生
    old_veloB_all = veloB_all;
    old_atti_all = atti_all;
    [t,atti_all(:,1),atti_rate_all(:,1),veloB_all(:,1),acceB_all(:,1)]=trace(t,T,atti_all(:,1),atti_rate_all(:,1),veloB_all(:,1),acceB_all(:,1));
    [velo_all(:,1)] = veloN0(atti_all(:,1),veloB_all(:,1));
    [Wibb,Fb] = IMUout(T,posi_w_enu_all(:,1),atti_all(:,1),atti_rate_all(:,1),veloB_all(:,1),acceB_all(:,1),old_veloB_all(:,1),old_atti_all(:,1));
    for i = 1:uav_num-1   %将航迹发生器参数全部赋值
        atti_all(:,i+1) = atti_all(:,1);
        atti_rate_all(:,i+1) = atti_rate_all(:,1);
        veloB_all(:,i+1) = veloB_all(:,1);
        acceB_all(:,i+1) = acceB_all(:,1);
        velo_all(:,i+1) =  velo_all(:,1);
    end
    
    for i = 1:uav_num
        if i <=uav_num-high_num
            [posi_w_enu_all(:,i)] = posi_out(T,posi_w_enu_all(:,i),veloB_all(:,i),atti_all(:,i));
        else
            [posi_L_enu_all(:,i-uav_num +high_num)] = posi_out(T,posi_L_enu_all(:,i-uav_num +high_num),veloB_all(:,i-uav_num +high_num),atti_all(:,i-uav_num +high_num));
        end
    end
    
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %IMU误差生成及惯导解算
    for i = 1:uav_num -high_num
        [Gyro_b(:,i),Gyro_r(:,i),Gyro_wg(:,i),Acc_r(:,i)]=imu_err_random(t,T,Gyro_b(:,i),Gyro_r(:,i),Gyro_wg(:,i),Acc_r(:,i)); 
        Wibb_noise(:,i) = Wibb + Gyro_b(:,i)/deg_rad + Gyro_r(:,i)/deg_rad + Gyro_wg(:,i)/deg_rad;
        Fb_noise(:,i) =  Fb + Acc_r(:,i);
    end
    
    for i = 1:uav_num-high_num
        [attiN_all(:,i),WnbbA_old(:,i)]=atti_cal_cq_modi(T,Wibb_noise(:,i)-Gyro_modi_all(:,i)/deg_rad,attiN_all(:,i),veloN_all(:,i),posiN_w_all(:,i),WnbbA_old(:,i));
            %姿态角求解
        [veloN_all(:,i)] = velo_cal(T,Fb_noise(:,i)-Acc_modi_all(:,i),attiN_all(:,i),veloN_all(:,i),posiN_w_all(:,i));
            %比力变换
        [posiN_w_all(:,i)] = posi_cal(T,veloN_all(:,i),posiN_w_all(:,i));
        %位置计算
    end
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    T_M = T_M + T; k_flag = 0;
    %进入卡尔曼滤波阶段,首先进行时间更新
    if T_M >= T_D
        T_M = 0.0; k_flag = 1;
        
        w_flag = 1; L_flag = 2;   % w_flag代表的僚机
        for i = 1:uav_num-high_num
            posiG_w_all(:,i) = simu_gps(posi_w_enu_all(:,i),w_flag); 
        end
        for i = 1:high_num
           posiG_L(:,i) = simu_gps(posi_L_enu_all(:,i),L_flag); 
        end
        
    %%%%%%%%%%%
    %%
    %时间更新
    for i = 1:uav_num-high_num
        Xc = zeros(18,1);    %系统的状态量
        Xc_all{1,i} = Xc;
    end
    for i = 1:uav_num-high_num
        [Xc_all{1,i},PK_all{1,i},Xerr_all{1,i}] = kalm_factor_time_update(t,T_D,Fb_noise(:,i),attiN_all(:,i),veloN_all(:,i),posiN_w_all(:,i),...
                                          Xc_all{1,i},PK_all{1,i},Xerr_all{1,i});
    end

    %%
    %%%%%%%%%%%
    %伪距生成
    for i = 1:uav_num-high_num
        uav_link_num{1,i} = [];   %记录每个辅机相连接的无人机标号
    end
    dis_err = 0.2;   %测距噪声设置
    %
    [posi_w_all,posi_L_all,dis_true] = distance_cal(posi_w_enu_all,posi_L_enu_all,posi_ini,uav_num,high_num);  %距离真值计算
    
    [dis_measure,uav_link_num] = psedu_cal(dis_true,dis_err,uav_link_num,uav_num,high_num);
    
    %%
    %建立因子图
    Xerr_gps_low = [10;10;20]; Xerr_gps_high = [0.2;0.2;0.5];
    for i = 1:uav_num-high_num     %首先需要将位置转换到直角坐标系
        posi_w_graph(:,i) = posical_xyz(posiG_w_all(:,i),posi_ini);
    end
    for i = 1:high_num
        posi_L_graph(:,i) = posical_xyz(posiG_L(:,i),posi_ini);
    end

    graph_1 = factor_graph_centralization(posi_L_graph,Xerr_gps_high);   %首先建立全局因子图
    
    %加入变量节点和先验因子
    k_number = high_num+1;   %前面三架长机，因此初始值设为4
    for i = 1 : uav_num-high_num
        graph_1.para_add(posi_w_graph(:,i),Xerr_gps_low,k_number);
        k_number = k_number + 1;
    end
    
    %加入测距因子节点
    H = zeros(1,6);
    for i = 1 : uav_num-high_num
        for j = 1:size(uav_link_num{1,i},2)
           if  uav_link_num{1,i}(1,j)>i && uav_link_num{1,i}(1,j)<=uav_num-high_num %节点为僚机
               %首先将因子节点的值的雅可比矩阵等
               posi_1 = posi_w_graph(:,i);
               posi_2 = posi_w_graph(:,uav_link_num{1,i}(1,j));
               r = sqrt((posi_1 - posi_2)' * (posi_1 - posi_2));
               H(1,1) = (posi_1(1,1) - posi_2(1,1))/r;
               H(1,2) =   (posi_1(2,1) - posi_2(2,1))/r;
               H(1,3) =   (posi_1(3,1) - posi_2(3,1))/r;
               H(1,4) = - (posi_1(1,1) - posi_2(1,1))/r;
               H(1,5) = - (posi_1(2,1) - posi_2(2,1))/r;
               H(1,6) = - (posi_1(3,1) - posi_2(3,1))/r;
               residuals = r - dis_measure(i,uav_link_num{1,i}(1,j));
               
               residuals = 1/dis_err * residuals;   %将残差向量白化
               jaco = 1/dis_err * H;
               
               k1 = i + high_num;
               k2 = uav_link_num{1,i}(1,j) + high_num;
               graph_1.factor_add(jaco,residuals,k1,k2,dis_measure(i,uav_link_num{1,i}(1,j)));
           end
           
           if uav_link_num{1,i}(1,j)>i && uav_link_num{1,i}(1,j) > uav_num-high_num %节点为长机
               posi_1 = posi_w_graph(:,i);
               posi_2 = posi_L_graph(:,uav_link_num{1,i}(1,j) - uav_num + high_num);
               r = sqrt((posi_1 - posi_2)' * (posi_1 - posi_2));
               H(1,1) = (posi_1(1,1) - posi_2(1,1))/r;
               H(1,2) =   (posi_1(2,1) - posi_2(2,1))/r;
               H(1,3) =   (posi_1(3,1) - posi_2(3,1))/r;
               H(1,4) = - (posi_1(1,1) - posi_2(1,1))/r;
               H(1,5) = - (posi_1(2,1) - posi_2(2,1))/r;
               H(1,6) = - (posi_1(3,1) - posi_2(3,1))/r;
               residuals = r - dis_measure(i,uav_link_num{1,i}(1,j));
               
               residuals = 1/dis_err * residuals;   %将残差向量白化
               jaco = 1/dis_err * H;
               
               k1 = i + high_num;
               k2 = uav_link_num{1,i}(1,j) - uav_num + high_num;
               graph_1.factor_add(jaco,residuals,k1,k2,dis_measure(i,uav_link_num{1,i}(1,j)));
           end
        end
    end
    %%
    %因子图解算
    graph_1.Gauss_Newton(posi_L_graph,posi_w_graph,Xerr_gps_high,Xerr_gps_low,high_num);
    graph_1.covariance();     %计算位置协方差
    for  i = 1:uav_num-high_num
        posi_w_graph(:,i) =  graph_1.parameters(i*3+3*high_num-2:i*3+3*high_num,1);
        posiN_w_graph(:,i) = posical_enu(posi_w_graph(:,i),posi_ini);
    end
    
    for i = 1:uav_num-high_num   %提取出协方差
        covariance = diag(graph_1.P_all);
        cov_graph(:,i) = sqrt(covariance(i*3+3*high_num-2:i*3+3*high_num,1));
    end
    %%
    %进行量测更新    
    for i = 1:uav_num-high_num    %惯性/因子图混合滤波用
        [Xc_all{1,i},PK_all{1,i},Xerr_all{1,i}] = kalm_factor_measure_update(t,posiN_w_all(:,i),posiN_w_graph(:,i),cov_graph(:,i),...
                                          Xc_all{1,i},PK_all{1,i},Xerr_all{1,i},k_flag);
    end
    
    %修正
    for i = 1:uav_num-high_num
        [attiN_all(:,i),veloN_all(:,i),posiN_w_all(:,i)] = kalm_modi(attiN_all(:,i),veloN_all(:,i),posiN_w_all(:,i),Xc_all{1,i});
        %进行滤波修正
        
        Gyro_modi_all(1,i) = Xc_all{1,i}(10,1) + Xc_all{1,i}(13,1);
        Gyro_modi_all(2,i) = Xc_all{1,i}(10,1) + Xc_all{1,i}(13,1);
        Gyro_modi_all(3,i) = Xc_all{1,i}(10,1) + Xc_all{1,i}(13,1);
        %陀螺修正量
        
        Acc_modi_all(1,i) = Xc_all{1,i}(16,1);
        Acc_modi_all(2,i) = Xc_all{1,i}(17,1);
        Acc_modi_all(3,i) = Xc_all{1,i}(18,1);
    end
    
    %记录GPS数据
    kc_kal = kc_kal + 1;
    GPSData(kc_kal,1) = t;GPSerrData(kc_kal,1) = t;
    for i = 1:uav_num-high_num
        posiG_xyz_temp = posical_xyz(posiG_w_all(:,i),posi_ini);
        GPSData(kc_kal,(i-1)*3+2:(i-1)*3+4) = posiG_xyz_temp';
        
        GraphData(kc_kal,(i-1)*3+2:(i-1)*3+4) = posi_w_graph(:,i);
        
        posi_w_temp = posical_xyz(posi_w_enu_all(:,i),posi_ini);
        GPSerrData(kc_kal,(i-1)*3+2:(i-1)*3+4) = posiG_xyz_temp' - posi_w_temp';
    end
    
    end
    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %记录数据
    k_sins = k_sins + 1;
    TraceData(k_sins,1) = t;
    for i = 1:high_num
        TraceData(k_sins,3*i-1:3*i+1) = posi_L_enu_all(:,i)';
    end
    SinsData(k_sins,1) = t; AttiNData(k_sins,1) = t; AttiData(k_sins,1) = t;
    TraceData_xyz(k_sins,1) = t; SinsData_xyz(k_sins,1) = t;
    VeloData(k_sins,1) = t; VeloNData(k_sins,1) = t; 
    
    for i = 1:uav_num-high_num
        TraceData(k_sins,(3*(i+high_num)-1):(3*(i+high_num)+1)) = posi_w_enu_all(:,i)';
        SinsData(k_sins,(i-1)*3+2:(i-1)*3+4) = posiN_w_all(:,i)';
        AttiNData(k_sins,(i-1)*3+2:(i-1)*3+4) = attiN_all(:,i)';
        VeloNData(k_sins,(i-1)*3+2:(i-1)*3+4) = veloN_all(:,i)';
        
        posi_w_all(:,i) = posical_xyz(posi_w_enu_all(:,i),posi_ini);    %计算出基于原点的僚机坐标值(真值)
        TraceData_xyz(k_sins,(i-1)*3+2:(i-1)*3+4) = posi_w_all(:,i)';
        
        posi_w_temp = posical_xyz(posiN_w_all(:,i),posi_ini);
        SinsData_xyz(k_sins,(i-1)*3+2:(i-1)*3+4) = posi_w_temp;
    end
    for i = 1:uav_num
        AttiData(k_sins,(i-1)*3+2:(i-1)*3+4) = atti_all(:,i)';
        VeloData(k_sins,(i-1)*3+2:(i-1)*3+4) = velo_all(:,i)';
    end
    
end
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%计算误差
%计算导航经纬度的误差（度－－》米）
Re=6378137.0;           %地球半径（米） 
f=1/298.257;            %地球的椭圆率
long = TraceData(1,2) * deg_rad;
lati = TraceData(1,3) * deg_rad;
heig = TraceData(1,4);
Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
Rn=Re*(1+f*sin(lati)*sin(lati));
rows = length(TraceData_xyz(:,1));
SINSerr = zeros(rows,(uav_num-high_num)*3 + 1);
SINSerr(:,1) = SinsData_xyz(:,1);
SINSerr(:,2:(uav_num-high_num)*3 + 1) = SinsData_xyz(:,2:(uav_num-high_num)*3 + 1) - TraceData_xyz(:,2:(uav_num-high_num)*3 + 1);

fname = ['SINSerr_hybrid',num2str(monte),'.dat'];
save(fname,'SINSerr','-ASCII');
fname_2 = ['TraceData_hybrid',num2str(monte),'.dat'];
save(fname_2,'TraceData','-ASCII');
fname_3 = ['AttiData_hybrid',num2str(monte),'.dat'];
save(fname_3,'AttiData','-ASCII');
fname_4 = ['AttiNData_hybrid',num2str(monte),'.dat'];
save(fname_4,'AttiNData','-ASCII');
fname_5 = ['VeloData_hybrid',num2str(monte),'.dat'];
save(fname_5,'VeloData','-ASCII');
fname_6 = ['VeloNData_hybrid',num2str(monte),'.dat'];
save(fname_6,'VeloNData','-ASCII');

end
%绘制图形
C = rand(uav_num,3);

figure_num = 0;
figure_num = figure_num + 1;
figure(figure_num);
for i = 1:uav_num
    plot3(TraceData(:,(i-1)*3+2),TraceData(:,(i-1)*3+3),TraceData(:,(i-1)*3+4),'color',C(i,:)); hold on;
end
grid;

figure_num = figure_num + 1;
figure(figure_num);
for i = 1:uav_num-3
    plot(SinsData(:,(i-1)*3+2),SinsData(:,(i-1)*3+3),'color',C(i,:));hold on
end
grid;

var_e = sqrt(sum(SINSerr(:,2).^2)/(length(SINSerr(:,1))-1));
var_n = sqrt(sum(SINSerr(:,3).^2)/(length(SINSerr(:,1))-1));
var_u = sqrt(sum(SINSerr(:,4).^2)/(length(SINSerr(:,1))-1));

EAST_ERR_1 = sqrt(SINSerr(:,2).^2 + SINSerr(:,3).^2 + SINSerr(:,4).^2);

max_error = Inf;

error_L = [];

% 遍历单位组合 (单位编号从 1 到 8)
for i = 1:uav_num-high_num
    % 计算单位 i 和单位 j 的经纬高误差
    idx_i = (i - 1) * 3 + 2; % 单位 i 的经纬高起始列

    rmse_E_ree_L = sqrt(sum(((SINSerr(:, idx_i))).^2) / rows);
    rmse_N_ree_L = sqrt(sum(((SINSerr(:, idx_i + 1))).^2) / rows);
    rmse_U_ree_L = sqrt(sum((SINSerr(:, idx_i + 2)).^2) / rows);

    % 计算定位误差
    error = sqrt(rmse_E_ree_L^2 + rmse_N_ree_L^2 + rmse_U_ree_L^2);

    % 保存误差值
    error_L = [error_L; error];
end

% 获取所有误差中的最大值
max_error = max(error_L)
% 找到最大误差值的索引，即对应的飞机编号
plane_idx = find(error_L == max_error);

% 显示最大误差和对应的飞机编号
disp(['最大定位误差值为: ', num2str(max_error),'m']);
disp(['最大误差对应的飞机编号是: ', num2str(plane_idx)]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%最后一号长机-SINS位置误差
% plane_idx = 2;
figure_num = figure_num+1;
figure(figure_num);
subplot(3,1,1);plot(SINSerr(:,1),(SINSerr(:,plane_idx*3-1)));grid;hold on;
titleText = sprintf('%d号飞行器-SINS位置误差', plane_idx);
% 设置标题
title(titleText, 'FontSize', 12);
ylabel('经度误差(m)','FontSize',10);
subplot(3,1,2);plot(SINSerr(:,1),(SINSerr(:,plane_idx*3)));grid;hold on;
ylabel('纬度误差(m)','FontSize',10);
subplot(3,1,3);plot(SINSerr(:,1),SINSerr(:,plane_idx*3+1));
% set(gca,'YLim',[-40 40]);
ylabel('高度误差(m)','FontSize',10);
xlabel('时间t(s)','FontSize',10);
grid;
