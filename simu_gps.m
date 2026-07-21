function [posiG] = simu_gps(posi,flag)

  Re=6378137.0;                                      %地球半径（米） 
  f=1/298.257;                                        %地球的椭圆率

  long=posi(1,1)*pi/180.0;lati=posi(2,1)*pi/180.0;heig=posi(3,1);
    %飞行器位置

  %地球曲率半径求解
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));

if flag == 1   %代表是僚机
    Err = [10*randn(1,1)/(Rn+heig)/cos(lati)/pi*180.0; 10*randn(1,1)/(Rm+heig)/pi*180.0; 20*randn(1,1)];
    % 单位需要由弧度统一为度才对
    
    posiG = posi + Err;
else
    Err = [0.2*randn(1,1)/(Rn+heig)/cos(lati) /pi*180.0; 0.2*randn(1,1)/(Rm+heig) /pi*180.0; 0.5*randn(1,1)];
    % 单位需要由弧度统一为度才对
    posiG = posi + Err;
end
end