classdef factor_graph_distributed < handle
%建立分布式因子图
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%factor_type : 0-本机先验因子；1-其他节点先验因子；2-测距因子；
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
        A = [];    %全局雅可比矩阵
        d = [];    %全局残差向量
        delta_X;   %状态更新向量
        x_num      = 1;     %当前时刻状态点总数(即X的长度/16)
        parameters = [];   %记录所有时刻的状态值      注意：这里位置的单位应该是m
        para_list  = [];   %分四列：factor_type；量测数；节点号；残差向量所在第一行
        P_covariance;      %记录当前时刻的协方差
        d_row;
        P_all;
        distance = [];
    end
    
    methods
        function obj = factor_graph_distributed(x,Xerr)     %以初始时刻的先验信息初始化，有个先验因子
            
            obj.para_list = [0,3,1,1];   %先验因子
            Jaco_prior    = eye(3,3);
            Xerr0         = Xerr;
            PK            = diag((Xerr0.^2));
            
            sqrt_info     = chol(inv(PK));
            
            obj.A         = sqrt_info * Jaco_prior;
            
            obj.parameters = [obj.parameters;x];
            obj.d = zeros(3,1);
            obj.d_row = 3;    %表示残差向量已存在3行
        end
        
        function para_add(obj,x,Xerr,k)
            obj.x_num = obj.x_num + 1;
            obj.para_list = [obj.para_list;
                             1,3,k,obj.d_row+1];   %
            obj.parameters = [obj.parameters;x];
            Jaco_prior    = eye(3,3);
            Xerr0         = Xerr;
            PK            = diag((Xerr0.^2));
            
            sqrt_info     = chol(inv(PK));
            [m1,n1] = size(obj.A);
            H = sqrt_info * Jaco_prior;
            obj.A = [obj.A,zeros(m1,3);
                     zeros(3,n1),H];
            obj.d = [obj.d;zeros(3,1)];
            obj.d_row = obj.d_row + 3;
        end
        
        function factor_add(obj,J,b,k1,dis)     %这里需要注意obj算输入个数！！k1代表节点标号,flag表示是锚节点还是移动节点
            obj.d = [obj.d;b];
            [m1,n1] = size(obj.A);       %注意：这里的n1其实是k*16
            m2 = size(b,1);
            J1 = J(1,1:3); J2 = J(1,4:6);
            obj.A = [obj.A;
                     J1,zeros(1,(k1-2)*3),J2,zeros(1,n1-k1*3)];

            obj.para_list = [obj.para_list;
                             2,m2,k1,obj.d_row + 1];
            obj.d_row = obj.d_row + 1;
            obj.distance = [obj.distance dis];
        end
        
        function para_replace(obj,x,Xerr,k)
            for i = 1:size(obj.para_list,1)
                if obj.para_list(i,1) == 1 && obj.para_list(i,3) == k
                    obj.parameters((k-1)*3+1:(k-1)*3+3,1) = x;
                    Jaco_prior    = eye(3,3);
                    Xerr0         = Xerr;
                    PK            = diag((Xerr0.^2));
            
                    sqrt_info     = chol(inv(PK));

                    H = sqrt_info * Jaco_prior;
                    
                    row = obj.para_list(i,4);
                    obj.A(row:row+2,(k-1)*3+1:(k-1)*3+3) = H;
                    obj.d(row:row+2,1) = zeros(3,1);
                    break
                end
            end
        end
        
        function factor_replace(obj,J,b,k1)
            for i = 1:size(obj.para_list,1)
                if obj.para_list(i,1) == 2 && obj.para_list(i,3) == k1
                    row = obj.para_list(i,4);
                    J1 = J(1,1:3); J2 = J(1,4:6);
                    [m1,n1] = size(obj.A);
                    obj.A(row,:) = [J1,zeros(1,(k1-2)*3),J2,zeros(1,n1-k1*3)];
                    
                    obj.d(row,1) = b;
                    
                    break
                end
            end
        end
        
        function Gauss_Newton(obj,posi_1,Xerr)
            n_iters = 50;
            thresh  = 1e-5;
            for i = 1:n_iters
                [Q,R] = qr(obj.A);
                obj.delta_X = - R \ Q' * obj.d;     %注意：这里加负号，那么X = X + delta_X;  delta_X = - (R' * R) \ R' * Q' * obj.d;
                [obj.parameters] = graph_modi(obj.parameters,obj.delta_X);
                k = 1;
                for a = 1:size(obj.para_list,1)                     %
                    switch obj.para_list(a,1)
                       case 0              %代表本机先验因子
                           Xerr0         = Xerr;
                           PK            = diag((Xerr0.^2));
%                            Jaco_prior    = eye(3,3);
            
                           sqrt_info     = chol(inv(PK));
                           obj.d(1:3,1)    = sqrt_info * (obj.parameters(1:3,1) - posi_1);
%                            obj.A(1:3,1:3) = sqrt_info * Jaco_prior;
                           k = k +3;
                       case 1      %代表其他节点先验因子
                           %不用进行更新
                           %%
                           
                           %%                           
                           k = k + 3;
                       case 2
                           k1 = obj.para_list(a,3);
                           
                           Pi = obj.parameters(1:3,1);
                           Pj = obj.parameters((k1-1)*3 + 1 : (k1-1)*3 + 3 , 1);
                           
                           [residual,jaco] = residual_cal(Pi,Pj,obj.distance(1,k1-1));
                           
                           obj.d(obj.para_list(a,4),1) = residual;
                           
                           J1 = jaco(1,1:3); J2 = jaco(1,4:6); n1 = size(obj.A,2);
                           obj.A(obj.para_list(a,4),:) = [J1,zeros(1,(k1-2)*3),J2,zeros(1,n1 - k1*3)];
                           
                           k = k + 1;
                           
                       otherwise
                            disp('there are undefined factors')
                    end
                end
                if max(obj.delta_X(1:3,1)) < thresh
                    break
                end
            end
                            
        end

        function covariance(obj)
%                 [Q_22,R_22] = qr(obj.A); 
%                 P_info  =  R_22' * R_22;
%                 P_covariance_sum = (inv(P_info));
%                 obj.P_covariance = P_covariance_sum(1:3,1:3);
%                 obj.P_gps = P_covariance_sum(4:6,4:6);
                P_info  =  obj.A' * obj.A;
                P_covariance_sum = (inv(P_info));
                obj.P_covariance = P_covariance_sum(1:3,1:3);
                obj.P_all = P_covariance_sum;
        end  
        
    end    
end

