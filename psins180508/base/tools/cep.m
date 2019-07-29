function c = cep(posINS, posGPS, t)
% Calculating CEP.
%
% Prototype: c = cep(posINS, posGPS, t))
% Inputs: posINS - SINS position data [lat,lon,hgt,t]
%         posGPS - GPS refference position data
%         t  - statical time point
% Output: c - CEP value
%
% See also  avpcmp.

% Copyright(c) 2009-2018, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/04/2018
global glv
    posINS = interp1(posINS(:,end), posINS(:,1:3), t);
    posGPS = interp1(posGPS(:,end), posGPS(:,1:3), t);
    err = posINS - posGPS;
    err = [err(:,1)*glv.Re,err(:,2)*glv.Re*cos(posGPS(1,1)),err(:,3)];
    r = rms(err);
    c = 0.5887*(r(1)+r(2));
    myfigure
    plot(t, err, '-*'); xygo('poserr / m'); legend('Latitude err', 'Longitude err', 'Height err');
    title(sprintf('CEP = %.3f / m',c));
