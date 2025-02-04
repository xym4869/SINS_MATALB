function [att0, res] = aligni0fit(imu, pos, ts)
% SINS initial align based on inertial frame method.
%
% Prototype: [att0, res] = aligni0(imu, pos, ts)
% Inputs: imu - IMU data
%         pos - position
%         ts - IMU sampling interval
% Output: att0 - attitude align result
%         res - some other paramters for debug
%
% See also  alignfn, alignvn, aligncmps, alignWahba, alignsb.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/12/2012, 07/03/2014, 28/08/2014
global glv
    if nargin<3,  ts = imu(2,7)-imu(1,7);  end
    nn = 1; nts = nn*ts;  ratio = 1; % 0.995;
    len = fix(length(imu)/nn)*nn;
    eth = earth(pos);  lat = pos(1);  g0 = -eth.gn(3);
    qib0b = [1; 0; 0; 0];
    [vib0, vi0, pib0, pi0, vib0_1, vi0_1] = setvals(zeros(3,1));
    [pib0k, pi0k, vi0k, vib0k, fi0k, fib0k, attk, attkv] = prealloc(len/nn, 3);
    k0 = fix(5/ts); % exculde the first 5s
    ki = timebar(nn, len, 'Initial align based on inertial frame.');
    Ax2 = g0*glv.wie*eth.cl/2; 
    Ay3 = g0*glv.wie^2*eth.sl*eth.cl/6; 
    Az1 = g0; 
    Az3 = -g0*glv.wie^2*eth.cl^2/6;
    kfx.xk = zeros(4,1); kfx.Pxk = eye(4); kfx.Hk = [1, 0, 0, 0];
    kfy = kfx; kfz = kfx;
    c = eye(3);
    for k=1:nn:len-nn+1
        wvm = imu(k:k+nn-1, 1:6);  kts = (k+nn-1)*ts;
        [phim, dvbm] = cnscl(wvm);
        fib0 = qmulv(qib0b, dvbm)/nts;   % f
        vib0 = vib0 + fib0*nts;          % vel
        pib0 = ratio*pib0 + (vib0_1+vib0)*nts/2;  vib0_1 = vib0; % pos
%         fi0 = [eth.cl*cos(kts*glv.wie);eth.cl*sin(kts*glv.wie);eth.sl]*g0;
%         vi0 = vi0 + fi0*nts;
%         pi0 = ratio*pi0 + (vi0_1+vi0)*nts/2;      vi0_1 = vi0;
        [fi0, vi0, pi0] = i0fvp(kts, lat);
        qib0b = qupdt(qib0b, phim);  % qib0b updating
        pib0k(ki,:) = pib0'; vib0k(ki,:) = vib0'; fib0k(ki,:) = fib0'; % recording
        pi0k(ki,:) = pi0';   vi0k(ki,:) = vi0';   fi0k(ki,:) = fi0';
        kfx.Hk = [1, kts, kts^2, kts^3]; kfy.Hk = kfx.Hk; kfz.Hk = kfx.Hk;
        kfx = RLS(kfx, vib0(1));
        kfy = RLS(kfy, vib0(2));
        kfz = RLS(kfz, vib0(3));
        if k>k0
            k1 = fix(ki/2);
            swiet = sin(kts*glv.wie); cwiet = cos(kts*glv.wie);
            Cni0 = [-swiet,cwiet,0; 
                -eth.sl*cwiet,-eth.sl*swiet,eth.cl; 
                eth.cl*cwiet,eth.cl*swiet,eth.sl];
            qni0 = m2qua(Cni0);
            c(1,3) = kfx.xk(2)/Az1; c(2,3) = kfy.xk(2)/Az1; c(3,3) = kfz.xk(2)/Az1; 
            c(1,1) = kfx.xk(3)/Ax2; c(2,1) = kfy.xk(3)/Ax2; c(3,1) = kfz.xk(3)/Ax2; 
            c(1,2) = (kfx.xk(4)-c(1,3)*Az3)/Ay3; c(2,2) = (kfy.xk(4)-c(2,3)*Az3)/Ay3; c(3,2) = (kfz.xk(4)-c(3,3)*Az3)/Ay3; 
            c(abs(c)>1) = 0.1;
            c = mnormlz(c);
            qnib0 = m2qua(c');
            qnb = qmul(qnib0,qib0b);
            attkv(ki,:) = q2att(qnb)';    % using vel
            qi0ib0 = dv2atti(pi0k(k1,:)', pi0, pib0k(k1,:)', pib0);
            qnb = qmul(qmul(qni0,qi0ib0),qib0b);
            attk(ki,:) = q2att(qnb)';     % using pos
       end
       ki = timebar;
    end
    tk = imu(nn:nn:length(attk)*nn,7);
    figure, plot(tk, [vib0k(:,1:3)- [polyval(flipud(kfx.xk),tk), polyval(flipud(kfy.xk),tk), polyval(flipud(kfz.xk),tk)]]); xygo('\deltav fit err / m/s');
    k0 = fix(k0/nn)+1;
%     attk(1:k0,:) = repmat(attk(k0+1,:),k0,1);
    Cni0 = [0,1,0; -eth.sl,0,eth.cl;  eth.cl,0,eth.sl];
    att0 = q2att(qmul(m2qua(Cni0),qi0ib0));
    attk(1:k0,:) = repmat(att0',k0,1);
    attkv(1:k0,:) = repmat(attkv(k0+1,:),k0,1);
    attk(:,4) = tk; attkv(:,4) = tk;
    res = varpack(lat, nts, vib0k, pib0k, fib0k, vi0k, pi0k, fi0k, attk, attkv, att0); 
    att0 = attk(end,1:3)';
    resdisp('Initial align attitudes (arcdeg)', att0/glv.deg);
    ai0plot(nts, attk, attkv);
    
function ai0plot(ts, attk, attkv)
global glv
    t = (1:length(attk))'*ts;
    myfigure;
    subplot(211), plot(t, attk(:,1:2)/glv.deg), xygo('pr');
        hold on,  plot(t, attkv(:,1:2)/glv.deg, 'm:'),
    subplot(212), plot(t, attk(:,3)/glv.deg), xygo('y');
        hold on,  plot(t, attkv(:,3)/glv.deg, 'm:'), legend('i0 pos', 'i0fit vel');
