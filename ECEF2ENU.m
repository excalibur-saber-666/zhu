
%坐标转换 ECEF - > ENU

function  [posiENU] = ECEF2ENU(posiECEF)

% constant
Re = 6378137.0;                     %地球长半径（单位：米） 
Rp = 6356755.0;                     %地球短半径（单位：米）
f = 1/298.257;
Wie  = 7.292115147e-5;
g = 9.7803698;
% e=sqrt(Re*Re-Rp*Rp)/Re;              %地球的第一偏心率
e=sqrt(2*f-f*f);              %地球的第一偏心率
% deg_rad=0.01745329252e0;% Transfer from angle degree to rad
deg_rad = pi/180;% Transfer from angle degree to rad

 %%%%%%大地直角坐标系G->大地坐标系C%%%%%%
uX = posiECEF(1,1);
uY = posiECEF(2,1);
uZ = posiECEF(3,1);

long = (atan(uY/uX))+pi;                    % 经度信息  单位 弧度
lati = atan(uZ/(1-f)^2/sqrt(uX^2+uY^2));    % 纬度信息  单位 弧度
RandH = uX/cos(lati)/cos(long);
% Rn = Re/sqrt(cos(lati)*cos(lati)+(1-e^2)*sin(lati)*sin(lati));
Rn = Re * (1 + f * sin(lati) * sin(lati));
heig = RandH - Rn;
old_lati = lati+1;
old_heig = heig+1;

k = 0;
% % % % % % N=zeros(100,1);
% % % % % % H=zeros(100,1);
% % % % % % L=zeros(100,1);
% % % % % % N(2,1)=Re;
% % % % % % H(2,1)=sqrt(loca_G(1,1)*loca_G(1,1)+loca_G(2,1)*loca_G(2,1)+loca_G(3,1)*loca_G(3,1))-sqrt(Re*Rp);
% % % % % % L(2,1)=(atan(loca_G(3,1)*(N(2,1)+H(2,1))/(sqrt(loca_G(1,1)*loca_G(1,1)+loca_G(2,1)*loca_G(2,1))*(N(2,1)+H(2,1)-e*e*N(2,1)))))/deg_rad;
% % % % % %    %设置迭代初值
% % % % % % k=1;
 while (abs(heig-old_heig)>1e-8)||(abs(lati-old_lati)/deg_rad>1e-14)
   
     old_lati = lati;
     old_heig = heig;
     lati = atan(RandH*uZ/(RandH-Rn*e*e)/sqrt(uX^2+uY^2));
     RandH = uX/cos(lati)/cos(long);
%      Rn = Re/sqrt(cos(lati)*cos(lati)+(1-e^2)*sin(lati)*sin(lati));
     Rn = Re * (1 + f * sin(lati) * sin(lati));
     heig = RandH - Rn;
     
     k = k+1;
     if k >=100000
         break
     end
% % % % % % %     k=k+1;
% % % % % % %     N(k+1,1)=Re/sqrt((1-e*e)*sin(L(k,1)*deg_rad)*sin(L(k,1)*deg_rad)+cos(L(k,1)*deg_rad)*cos(L(k,1)*deg_rad));
% % % % % % %     H(k+1,1)=sqrt(loca_G(1,1)*loca_G(1,1)+loca_G(2,1)*loca_G(2,1))/cos(L(k,1)*deg_rad)-N(k+1,1);
% % % % % % %     L(k+1,1)=(atan(loca_G(3,1)*(N(k+1,1)+H(k+1,1))/(sqrt(loca_G(1,1)*loca_G(1,1)+loca_G(2,1)*loca_G(2,1))*(N(k+1,1)+H(k+1,1)-e*e*N(k+1,1)))))/deg_rad;
 end

%  k
%  heig-old_heig
%  lati-old_lati
 
posiENU(1,1) = long/deg_rad;                               %目标经度
posiENU(2,1) = lati/deg_rad;                                          %目标纬度
posiENU(3,1) = heig;




