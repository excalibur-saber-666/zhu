function posi_ECEF = ENU2ECEF(posiENU)
%GEO2CART Conversion of geographical coordinates (phi, lambda, h) to
%Cartesian coordinates (X, Y, Z). 
%
%[X, Y, Z] = geo2cart(phi, lambda, h, i);
%
%Format for phi and lambda: [degrees minutes seconds].
%h, X, Y, and Z are in meters.
%
%Choices i of Reference Ellipsoid
%   1. International Ellipsoid 1924
%   2. International Ellipsoid 1967
%   3. World Geodetic System 1972
%   4. Geodetic Reference System 1980
%   5. World Geodetic System 1984
%
%   Inputs:
%       phi       - geocentric latitude (format [degrees minutes seconds])
%       lambda    - geocentric longitude (format [degrees minutes seconds]) 
%       h         - height
%       i         - reference ellipsoid type
%
%   Outputs:
%       X, Y, Z   - Cartesian coordinates (meters)

%Kai Borre 10-13-98
%Copyright (c) by Kai Borre
%
% CVS record:
% $Id: geo2cart.m,v 1.1.2.7 2006/08/22 13:45:59 dpl Exp $
%==========================================================================

% global variable
% global sign_settings orbit

% constant
Re = 6378137.0;
f = 1/298.257;
Wie  = 7.292115147e-5;
g = 9.7803698;

% user position (rad) 单位转换成弧度
long = posiENU(1,1) * pi / 180.0;
lati = posiENU(2,1) * pi / 180.0;
heig = posiENU(3,1);

% earth ellipticity
Rn = Re * (1 + f * sin(lati) * sin(lati));

% position ECEF
uX = (Rn + heig) * cos(lati) * cos(long); 
uY = (Rn + heig) * cos(lati) * sin(long) ;
uZ = (Rn * (1 - f)^2 + heig) * sin(lati);
posi_ECEF = [uX uY uZ]';

% l   = posiENU(1);
% l   = l*pi / 180;
% b   = posiENU(2);
% b   = b*pi / 180;
% h   = posiENU(3);
% 
% a   = [6378388 6378160 6378135 6378137 6378137];
% f   = [1/297 1/298.247 1/298.26 1/298.257222101 1/298.257223563];
% 
% ex2 = (2-f(i))*f(i) / ((1-f(i))^2);
% c   = a(i) * sqrt(1+ex2);
% N   = c / sqrt(1 + ex2*cos(b)^2);
% 
% 
% X   = (N+h) * cos(b) * cos(l);
% Y   = (N+h) * cos(b) * sin(l);
% Z   = ((1-f(i))^2*N + h) * sin(b);
% 
% posi_ECEF = [X Y Z]';
%%%%%%%%%%%%%% end geo2cart.m  %%%%%%%%%%%%%%%%%%%%%%%%
