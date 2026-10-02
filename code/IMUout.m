%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    IMU仿真程序
%
%  输入参数:
%  T          仿真步长
%  atti       横滚、俯仰、航向（单位：度）
%  atti_rate  横滚速率、俯仰速率、航向速率（单位：度/秒）
%  veloB      飞机运动速度——X右翼、Y机头、Z天向（单位：米/秒）
%  acceB      飞机运动加速度——X右翼、Y机头、Z天向（单位：米/秒/秒）
%  posi       航迹发生器初始位置经度、纬度、高度（单位：度、度、米）
%
%  输出参数：
%  Wibb       机体系陀螺仪输出   （单位：度/秒）
%  Fb         机体系加速度计输出 （单位：米/秒/秒）
%      
%                   熊智            南京航空航天大学导航研究中心编制 2007年04月
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [Wibb,Fb]=IMUout(T,posi,atti,atti_rate,veloB,acceB,old_veloB,old_atti)

  %%%%%%%%%%%%%%%与地球有关的参数%%%%%%%%%%%%%%%%%%%%%%
  Re=6378137.0;        %地球半径     （单位：米） 
  f=1/298.257;         %地球的椭圆率
  Wie=7.292115147e-5;  %地球自转角速度（单位：弧度/秒）
  g=9.7803698;         %重力加速度    （单位：米/秒/秒）

  long=posi(1,1)*pi/180.0;lati=posi(2,1)*pi/180.0;heig=posi(3,1);
     %飞行器位置

  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
     %地球曲率半径求解

  roll=old_atti(1,1)*pi/180.0;pitch=old_atti(2,1)*pi/180.0;head=old_atti(3,1)*pi/180.0;
  droll=atti_rate(1,1)*pi/180.0;dpitch=atti_rate(2,1)*pi/180.0; dhead=atti_rate(3,1)*pi/180.0;
     %姿态角和姿态角速率

  Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];
    %坐标系N-->B

  Eluer_M=[cos(roll), 0, sin(roll)*cos(pitch);
           0,         1, -sin(pitch);
           sin(roll), 0, -cos(pitch)*cos(roll)];   
    %欧拉角变换矩阵

  %%%%%%%%%%%陀螺仪输出%%%%%%%%%%%%
  Wnbb=Eluer_M*[dpitch;droll;dhead];

  veloN=Cbn'*old_veloB;
  Ve=veloN(1,1);Vn=veloN(2,1);Vu=veloN(3,1);

  Wenn=[-Vn/(Rm+heig); Ve/(Rn+heig);  Ve/(Rn+heig)*tan(lati)];
  Wien=[0;             Wie*cos(lati); Wie*sin(lati)];
  
  Wibb=Cbn*(Wien+Wenn)+Wnbb; %单位：弧度/秒

  Wibb=Wibb*180.0/pi;        %单位：度/秒

  %%%%%%%%%%%%%%加速度计输出%%%%%%%%%%%%%%
    roll=atti(1,1)*pi/180.0;pitch=atti(2,1)*pi/180.0;head=atti(3,1)*pi/180.0;
     %姿态角和姿态角速率

  Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];
    %坐标系N-->B
  
  acceN=Cbn'*(acceB+cross(Wnbb,old_veloB));
  Fn=acceN+cross(2*Wien+Wenn,veloN)-[0.0;0.0;-1.0*g];  %??如何理解
  Fb=Cbn*Fn;                 %单位：米/秒/秒


