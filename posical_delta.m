function delta_posi = posical_delta(delta_p , posiN)
  Re=6378137.0;  %地球半径（米） 
  f=1/298.257;   %地球的椭圆率

  long=posiN(1,1)*pi/180.0;lati=posiN(2,1)*pi/180.0;heig=posiN(3,1);
    %飞行器位置

  Rm=Re*(1-2*f+3*f*sin(lati)*sin(lati));
  Rn=Re*(1+f*sin(lati)*sin(lati));
  
  heig = delta_p(3,1);
  lati = delta_p(2,1)/(Rm+heig)*180.0/pi;
  long = delta_p(1,1)/((Rn+heig)*cos(lati))*180.0/pi;
  
  delta_posi = [long;lati;heig];
end