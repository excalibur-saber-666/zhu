function [posi] = posi_out(T,posi,veloB,atti)  
Re=6378137.0;        %地球半径     （单位：米） 
f=1/298.257;         %地球的椭圆率


long=posi(1,1)*pi/180.0;lati=posi(2,1)*pi/180.0;heig=posi(3,1);
     %飞行器位置

Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
Rn=Re*(1+f*sin(lati)*sin(lati));

roll=atti(1,1)*pi/180.0;pitch=atti(2,1)*pi/180.0;head=atti(3,1)*pi/180.0;
     %姿态角和姿态角速率

Cbn=[cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
     cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
     sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];
    %坐标系N-->B
%%%%%%%%%%%%%%%位置计算%%%%%%%%%%%%%
veloN=Cbn'*veloB;
Ve=veloN(1,1);Vn=veloN(2,1);Vu=veloN(3,1);
  
heig=heig+T*Vu;
lati=lati+T*(Vn/(Rm+heig));
long=long+T*(Ve/((Rn+heig)*cos(lati)));

posi(1,1)=long*180.0/pi;
posi(2,1)=lati*180.0/pi;
posi(3,1)=heig;