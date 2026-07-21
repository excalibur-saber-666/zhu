%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  输入 ：无人机位置坐标 PosiUavA,PosiUavB,PosiUavC
%         测距信息 DistanceSensor
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [PosiUser] = LocationUserTransforCoordinates(PosiUavA,PosiUavB,PosiUavC,DistanceSensor,PosiUser)


% 计算坐标转换矩阵
  [PosiUavAJ,PosiUavBJ,PosiUavCJ,Cej,Crotation,flag_Z]= ECEF2Calculate(PosiUavA,PosiUavB,PosiUavC); %取不在一直线上的三点确立计算系，求取G-j的转换矩阵
  
%   %%%%%%%%%%%%%%%%
%   PosiUserJTemp =Cej * [PosiUser;1];
%   
%   PosiUserJ(1,1) = PosiUserJTemp(1,1)/PosiUserJTemp(4,1);
%   PosiUserJ(2,1) = PosiUserJTemp(2,1)/PosiUserJTemp(4,1);
%   PosiUserJ(3,1) = PosiUserJTemp(3,1)/PosiUserJTemp(4,1);
%   
%   DisJ = zeros(3,1);
%   DisJ(1,1) = DistanceAB(PosiUserJ,PosiUavAJ);
%   DisJ(2,1) = DistanceAB(PosiUserJ,PosiUavBJ);
%   DisJ(3,1) = DistanceAB(PosiUserJ,PosiUavCJ);
%   DisJ
%   DistanceSensor
%   DetaDis = DisJ - DistanceSensor
%   %%%%%%%%%%%%%%%%
  
  
%  距离值
  DisA = DistanceSensor(1,1);
  DisB = DistanceSensor(2,1);
  DisC = DistanceSensor(3,1); 
%  求解用户在计算坐标系下的XYZ值
% X
 X = (DisB^2 - DisA^2 - PosiUavBJ(1,1)^2)/(-2*PosiUavBJ(1,1));
   
% Y
 Y =  -PosiUavCJ(1,1)/PosiUavCJ(2,1)*X + (DisA^2-DisC^2+PosiUavCJ(1,1)^2+PosiUavCJ(2,1)^2)/(2*PosiUavCJ(2,1));
 
% Z
 Z = sqrt(DisA^2 - X^2 - Y^2);
 if(flag_Z>0)  %需要知道目标是否在飞机航迹面上方或下方
   Z = -Z;  
 end
 
  %平移矩阵
 C_m=[eye(3,3),PosiUavA;
      0,0,0,1]; 
 
 PosiUserTemp = C_m*Crotation'*[X;Y;Z;1];
 
  PosiUser(1,1) = PosiUserTemp(1,1)/PosiUserTemp(4,1);
  PosiUser(2,1) = PosiUserTemp(2,1)/PosiUserTemp(4,1);
  PosiUser(3,1) = PosiUserTemp(3,1)/PosiUserTemp(4,1);
  
end


