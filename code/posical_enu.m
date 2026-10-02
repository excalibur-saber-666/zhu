function [posiN] = posical_enu(posi_xyz,posiN_0)

%-----------位置----------------%
Re        =  6378137.0;  %地球半径（米） 
f         =  1/298.257;   %地球的椭圆率

long      =  posiN_0(1,1) * pi / 180.0;
lati      =  posiN_0(2,1) * pi / 180.0;
heig      =  posiN_0(3,1);
    %飞行器位置
Rm        =  Re*(1-2*f+3*f*sin(lati)*sin(lati));
Rn        =  Re*(1+f*sin(lati)*sin(lati));
    %地球曲率半径求解
heig      =  heig +  posi_xyz(3,1);
lati      =  lati + (posi_xyz(2,1)/(Rm+heig));
long      =  long + (posi_xyz(1,1)/((Rn+heig)*cos(lati)));

posiN(1,1) =  long*180.0/pi;       %单位：度
posiN(2,1) =  lati*180.0/pi;       %单位：度
posiN(3,1) =  heig;

end