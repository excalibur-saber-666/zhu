%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  输入 ：无人机位置坐标 PosiUavA,PosiUavB,PosiUavC
%         测距信息 DistanceSensor
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [PosiUser] = LocationUserLM(PosiUav,DistanceSensor,PosiUser)


UavNum = length(PosiUav);
data_1 = PosiUav;
obs_1 = DistanceSensor.*DistanceSensor;

% PosiUser = PosiUav(1,:)';
PosiUser = zeros(3,1);
x0 = PosiUser(1,1);
y0 = PosiUser(2,1);
z0 = PosiUser(3,1);
y_init = zeros(1,UavNum);
for i = 1:UavNum
    y_init(1,i)=(x0-PosiUav(i,1))^2+(y0-PosiUav(i,2))^2+(z0-PosiUav(i,3))^2;
end

% 数据个数
Ndata = UavNum;
% 参数维数
Nparams=3;
% 迭代最大次数
n_iters=30;
% LM算法的阻尼系数初值
lamda=1;

% step1: 变量赋值
updateJ=1;
% % % % a_est=a0;
% % % % b_est=b0;
x_est = x0;
y_est = y0;
z_est = z0;

% step2: 迭代
for it=1:n_iters
    if updateJ==1
        % 根据当前估计值，计算雅克比矩阵, 并根据当前参数，得到函数值
        J=zeros(Ndata,Nparams);
        fx = zeros(Ndata,1);
        for i=1:Ndata
            J(i,:) = [ 2*x_est - 2*data_1(i,1), 2*y_est - 2*data_1(i,2), 2*z_est - 2*data_1(i,3)];
            fx(i,1) = (x_est-data_1(i,1))^2+(y_est-data_1(i,2))^2+(z_est-data_1(i,3))^2;  %   f=(x-xi)^2+(y-yi)^2+(z-zi)^2;  
        end
           
        % 计算误差
        d=obs_1-fx;
        % 计算（拟）海塞矩阵
        H=J'*J;
        % 若是第一次迭代，计算误差
        if it==1
            e=dot(d,d);
        end
    end

    % 根据阻尼系数lamda混合得到H矩阵
    H_lm=H+(lamda*eye(Nparams,Nparams));
    % 计算步长dp，并根据步长计算新的可能的\参数估计值
    dp=inv(H_lm)*(J'*d(:));
    g = J'*d(:);
% % % %     a_lm=a_est+dp(1);
% % % %     b_lm=b_est+dp(2);
    x_lm=x_est+dp(1);
    y_lm=y_est+dp(2);
    z_lm=z_est+dp(3);
    
    % 计算新的可能估计值对应的y和计算残差e
    fx_est_lm =  zeros(Ndata,1);
    for i=1:Ndata
        fx_est_lm(i,1) = (x_lm-data_1(i,1))^2+(y_lm-data_1(i,2))^2+(z_lm-data_1(i,3))^2;
    end
    d_lm=obs_1-fx_est_lm;
    e_lm=dot(d_lm,d_lm);
    % 根据误差，决定如何更新参数和阻尼系数
    if e_lm<e
        lamda=lamda/10;
% % % %         a_est=a_lm;
% % % %         b_est=b_lm;
        x_est=x_lm;
        y_est=y_lm;
        z_est=z_lm;
        e=e_lm;
% % % %         disp(e);
        updateJ=1;
    else
        updateJ=0;
        lamda=lamda*10;
    end
end
% % % %     yuzhi = sqrt(g'*g)
    
   PosiUser(1,1) = x_est;
   PosiUser(2,1) = y_est;
   PosiUser(3,1) = z_est;
  
end


