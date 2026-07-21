function [Xc,PK,Xerr] = kalm_factor_measure_update(t,posiN,posiG,VG,Xc,PK,Xerr,k_flag)
%进行量测更新
  Re=6378137.0;                                      %地球半径（米） 
  f=1/298.257;                                        %地球的椭圆率
  Wie=7.292115147e-5;                          %地球自转角速度
  g=9.7803698;                                      %重力加速度

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %飞行器位置

  %地球曲率半径求解
  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));

  HG=[zeros(3,6),diag([Rm,Rn*cos(lati),1]),zeros(3,9)];
%   VG=[10;10;20];  % 需要与GPS仿真精度相同
  
  RG=diag((VG.^2)');
  
  I=eye(size(PK));
  
  if (k_flag == 1)
      KK=PK*HG'*inv(HG*PK*HG'+RG);
      PK=(I-KK*HG)*PK*(I-KK*HG)'+KK*RG*KK';
      Yc=[(posiN(2,1)-posiG(2,1))*pi/180.0*(Rm+heig);
        (posiN(1,1)-posiG(1,1))*pi/180.0*(Rn+heig)*cos(lati);
        posiN(3,1)-posiG(3,1)]; %量测次序为纬度、经度、高度
    
    Xc=Xc+KK*(Yc-HG*Xc); 
  end
  
  %%%%%%%%%%%%%%%%%滤波估计精度%%%%%%%%%%%%
  Xerr(1,1)=sqrt(PK(1,1))*180.0*3600.0/pi;    %sec
  Xerr(1,2)=sqrt(PK(2,2))*180.0*3600.0/pi;    %sec
  Xerr(1,3)=sqrt(PK(3,3))*180.0*3600.0/pi;    %sec
  Xerr(1,4)=sqrt(PK(4,4));                    %m/s
  Xerr(1,5)=sqrt(PK(5,5));                    %m/s
  Xerr(1,6)=sqrt(PK(6,6));                    %m/s
  Xerr(1,7)=sqrt(PK(7,7))*(Rm+heig);          %m
  Xerr(1,8)=sqrt(PK(8,8))*(Rn+heig)*cos(lati);%m
  Xerr(1,9)=sqrt(PK(9,9));                    %m
       %INS的9个导航量误差
         
  Xerr(1,10)=sqrt(PK(10,10))*180.0*3600.0/pi;   %deg/h
  Xerr(1,11)=sqrt(PK(11,11))*180.0*3600.0/pi;   %deg/h
  Xerr(1,12)=sqrt(PK(12,12))*180.0*3600.0/pi;   %deg/h
  Xerr(1,13)=sqrt(PK(13,13))*180.0*3600.0/pi;   %deg/h
  Xerr(1,14)=sqrt(PK(14,14))*180.0*3600.0/pi;   %deg/h
  Xerr(1,15)=sqrt(PK(15,15))*180.0*3600.0/pi;   %deg/h
  Xerr(1,16)=sqrt(PK(16,16))/g;                 %g
  Xerr(1,17)=sqrt(PK(17,17))/g;                 %g
  Xerr(1,18)=sqrt(PK(18,18))/g;                 %g
      %IMU的9个误差量
end