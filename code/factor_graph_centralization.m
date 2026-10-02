classdef factor_graph_centralization  < handle
%建立集中式因子图
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%factor_type : 0-先验因子；1-测距因子；
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    properties
        A = [];   
        d = [];    %全局残差向量
        delta_X;   %状态更新向量
        parameters = [];   %记录所有时刻的状态值      注意：这里位置的单位应该是m
        para_list  = [];   %分四列：factor_type；factor_type=0时，后三列为量测数；节点号；残差向量所在第一行
                                               % factor_type=1时，后三列为节点号；节点号；残差向量所在第一行
        P_covariance;      %记录当前时刻的协方差
        d_row;
        P_all;
        distance = [];
        range_weights = [];
        iteration_count = 0;
        final_step_norm = NaN;
        converged = false;
    end
    
    methods
        function obj = factor_graph_centralization(posiL,Xerr)
            num = size(posiL,2);     %取长机的个数
            
            Jaco_prior = eye(3*num,3*num);
            Xerr0 = [];
            junction_num = 0; row_num = 0;
            for i = 1:num
                obj.para_list = [obj.para_list;
                                 0,3,junction_num + i,(row_num + i-1) *3+1];
                Xerr0 = [Xerr0;Xerr];    %注意：Xerr应该是列向量
                
                obj.parameters = [obj.parameters;
                                  posiL(:,i)];
            end
            
            PK = diag((Xerr0.^2));
            sqrt_info = chol(inv(PK));
            
            obj.A = sqrt_info * Jaco_prior;
            obj.d = zeros(3*num,1);
            obj.d_row = 3*num;    %表示残差向量已经存在3*num行
        end
            
        function para_add(obj,x,Xerr,k)
            obj.para_list = [obj.para_list;
                             0,3,k,obj.d_row + 1];
            obj.parameters = [obj.parameters;x];
            
            Jaco_prior    = eye(3,3);
            Xerr0 = Xerr;
            PK = diag((Xerr0.^2));
            sqrt_info = chol(inv(PK));
            [m1,n1] = size(obj.A);
            H = sqrt_info * Jaco_prior;
            
            obj.A = [obj.A,zeros(m1,3);
                    zeros(3,n1),H];
            obj.d = [obj.d;zeros(3,1)];
            obj.d_row = obj.d_row + 3;
        end
        
        function factor_add(obj,J,b,k1,k2,dis,weight)  %注意：这里的k1/k2在进入函数前要确定好，长机在前
             if nargin < 7
                 weight = 1;
             end
             if ~isscalar(weight) || ~isreal(weight) || ~isfinite(weight) || weight <= 0
                 error('factor_graph_centralization:InvalidRangeWeight', ...
                     'Range weight must be a finite positive real scalar.');
             end
             obj.d = [obj.d;b];   %注意：这里的b已经乘了sqrt_info了
             [m1,n1] = size(obj.A);
             
             J1 = J(1,1:3); J2 = J(1,4:6);
             
             obj.A = [obj.A;zeros(1,n1)];   %首先增加一行
             obj.A(m1+1,(k1-1)*3+1:(k1-1)*3+3) = J1;   %直接对最下面一行进行赋值
             obj.A(m1+1,(k2-1)*3+1:(k2-1)*3+3) = J2;   %这样做的好处是k1,k2之间的顺序不需要考虑
             obj.para_list = [obj.para_list;
                              1,k1,k2,obj.d_row + 1];
             obj.d_row = obj.d_row + 1;
             obj.distance = [obj.distance dis];     %
             obj.range_weights = [obj.range_weights weight];
        end
        
        function Gauss_Newton(obj,posiG_L,posiG_w,Xerr_L,Xerr_w,high_num)   %注意这里的位置信息不能是经纬度，而是以[0;0;0]为原点
            n_iters = 500;
            thresh  = 1e-5;
            obj.iteration_count = 0;
            obj.final_step_norm = NaN;
            obj.converged = false;
            for i = 1:n_iters
                [Q,R] = qr(obj.A);
                obj.delta_X = - R \ Q' * obj.d;
                obj.iteration_count = i;
                obj.final_step_norm = norm(obj.delta_X, inf);
                [obj.parameters] = graph_modi(obj.parameters,obj.delta_X);
                k = 1;
                for a = 1:size(obj.para_list,1)
                    switch obj.para_list(a,1)
                        case 0
                            if obj.para_list(a,3) > high_num
                                Xerr0 = Xerr_w;
                                measure_posi = posiG_w(:,obj.para_list(a,3)-high_num);
                            else
                                Xerr0 = Xerr_L;
                                measure_posi = posiG_L(:,obj.para_list(a,3));
                            end
                            PK = diag((Xerr0.^2));
                            d_row_ini =  obj.para_list(a,4);
                            sqrt_info = chol(inv(PK));
                            
                            obj.d(d_row_ini:d_row_ini+2,1) = sqrt_info * (obj.parameters(d_row_ini:d_row_ini+2,1) - measure_posi);
                            
                        case 1
                            k1 = obj.para_list(a,2);
                            k2 = obj.para_list(a,3);
                            
                            Posi_1 = obj.parameters((k1-1)*3 + 1 : (k1-1)*3 + 3 , 1);
                            Posi_2 = obj.parameters((k2-1)*3 + 1 : (k2-1)*3 + 3 , 1);
                            
                            [residual,jaco] = residual_cal(Posi_1,Posi_2,obj.distance(1,k));
                            range_scale = sqrt(obj.range_weights(1,k));
                            residual = range_scale * residual;
                            jaco = range_scale * jaco;
                            k = k + 1;
                            
                            obj.d(obj.para_list(a,4),1) = residual;
                            
                            J1 = jaco(1,1:3);  J2 = jaco(1,4:6);
                            
                            obj.A(obj.para_list(a,4),(k1-1)*3+1:(k1-1)*3+3) = J1;
                            obj.A(obj.para_list(a,4),(k2-1)*3+1:(k2-1)*3+3) = J2;
                            
                        otherwise
                            disp('there are undefined factors')
                    end
                end
                
                if obj.is_step_converged(obj.delta_X, thresh)
                    obj.converged = true;
                    break
                end
                
            end
        end
        
        function covariance(obj)
                P_info  =  obj.A' * obj.A;
                P_covariance_sum = (inv(P_info));
                obj.P_all = P_covariance_sum;
        end

        function converged = is_step_converged(~, step, threshold)
            converged = norm(step, inf) < threshold;
        end
    end
end
