function posi_xyz = posical_xyz(posiN , posiN_0)
 %%%%%%%%%%%%%%%%%%计算在东北天坐标系下的位置(以m为单位)%%%%%%%%%%%
  Re=6378137.0;  %地球半径（米） 
  f=1/298.257;   %地球的椭圆率

 lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %飞行器位置

  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
  
  posi_xyz(1,1) = (posiN(1,1) - posiN_0(1,1))*pi/180.0*(Rn+heig)*cos(lati);
  posi_xyz(2,1) = (posiN(2,1) - posiN_0(2,1))*pi/180.0*(Rm+heig);
  posi_xyz(3,1) =  posiN(3,1) - posiN_0(3,1);