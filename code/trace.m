%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    航迹发生器
%
%  输入参数:
%  t          仿真时间
%  T          仿真步长
%  atti       横滚、俯仰、航向（单位：度）
%  atti_rate  横滚速率、俯仰速率、航向速率（单位：度/秒）
%  veloB      飞机运动速度——X右翼、Y机头、Z天向（单位：米/秒）
%  acceB      飞机运动加速度——X右翼、Y机头、Z天向（单位：米/秒/秒）
%  posi       航迹发生器初始位置经度、纬度、高度（单位：度、度、米）
%
%        
%                熊智               南京航空航天大学导航研究中心编制 2007年04月
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [t,atti,atti_rate,veloB,acceB]=trace(t,T,atti,atti_rate,veloB,acceB)

%     if( t==0 )
%       acceB(2,1) = 0;               %初始对准时间    
%    elseif(t<=20)                       %滑跑
%       acceB(2,1)=8.0;
%     elseif(t<=24)                   %加速拉起
%       atti_rate(2,1)=3.5;
%       acceB(2,1)=10.0;            
%    elseif (t<=64)                   %爬高
%       atti_rate(2,1)=0.0;
%    elseif (t<=68)                   %改平
%       atti_rate(2,1)=-3.5;
%       acceB(2,1)=0.0;
%    elseif (t<=668)                  %平直飞行
%       atti_rate(2,1)=0.0;
%    elseif (t<=671)                  %倾斜预转弯
%       atti_rate(1,1)=5.0;
%    elseif (t<=731)                  %转弯
%       atti_rate(3,1)=1.5;
%       atti_rate(1,1)=0.0;
%    elseif (t<=734)                  %改平
%       atti_rate(3,1)=0.0;
%       atti_rate(1,1)=-5.0;
%    elseif (t<=1334)                 %平直飞行
%       atti_rate(1,1)=0.0;
%    elseif (t<=1342)                 %加速拉起
%       atti_rate(2,1)=3.5;
%       acceB(2,1)=2.0;
%    elseif (t<=1374)                 %加速爬高
%       atti_rate(2,1)=0.0;
%    elseif (t<=1382)                 %改平
%       atti_rate(2,1)=-3.5;
%       acceB(2,1)=0.0;
%    elseif (t<=1862)                 %平直飞行
%        atti_rate(2,1)=0.0;
%    elseif (t<=1902)                 %减速飞行
%       acceB(2,1)=-2.5;
%    elseif (t<=1905)                 %倾斜预转弯
%       atti_rate(1,1)=5.0;
%       acceB(2,1)=0.0;
%    elseif (t<=1965)                 %转弯
%       atti_rate(3,1)=1.5;
%       atti_rate(1,1)=0.0;
%    elseif (t<=1968)                 %改平
%       atti_rate(3,1)=0.0;
%       atti_rate(1,1)=-5.0;
%    elseif (t<=2568)                 %平直飞行
%       atti_rate(1,1)=0.0;
%    elseif (t<=2574)                 %低头
%       atti_rate(2,1)=-3.5;
%    elseif (t<=2594)                 %俯冲
%       atti_rate(2,1)=0.0;
%    elseif (t<=2600)                 %改平
%       atti_rate(2,1)=3.5;
%    elseif (t<=2603)                 %倾斜预转弯
%       atti_rate(2,1)=0.0;
%       atti_rate(1,1)=5.0;
%    elseif (t<=2663)                 %转弯
%       atti_rate(3,1)=1.5;
%       atti_rate(1,1)=0.0;
%    elseif (t<=2666)                 %改平
%       atti_rate(3,1)=0.0;
%       atti_rate(1,1)=-5.0;
%    elseif (t<=3600)                 %平直飞行
%       atti_rate(1,1)=0.0;            
%    end
   
   if( t==0 )
      acceB(2,1) = 0;               %初始对准时间    
   elseif(t<=300)                       %滑跑
      acceB(2,1) = 0; 
   elseif(t<=315)
      atti_rate(3,1)= 6;
   elseif(t<=320)
      atti_rate(3,1) = 0;
   elseif(t<=330)
      atti_rate(2,1) = 0;
   elseif(t<=340)
      atti_rate(2,1) = 0; 
   elseif(t<=400)
      atti_rate(2,1) = 0;
   elseif(t<=415)
      atti_rate(3,1)= -6.0; 
   elseif(t<=600)   
      atti_rate(3,1)= 0; 
   end

   
%    if( t>0 )   atti_rate(1,1) = 360.0; end 
%    atti_rate(2,1) = 0;
%    atti_rate(3,1) = 0;
   
   veloB(2,1)=veloB(2,1)+acceB(2,1)*T;    

   atti(1,1)=atti(1,1)+atti_rate(1,1)*T;
   atti(2,1)=atti(2,1)+atti_rate(2,1)*T;
   atti(3,1)=atti(3,1)+atti_rate(3,1)*T;
  
%   if(atti(1,1)>360)
%	atti(1,1)=atti(1,1)-fix(atti(1,1)/360)*360;
%   end


