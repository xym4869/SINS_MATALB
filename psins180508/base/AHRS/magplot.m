function yaw = magplot(mag)
% 3-axis geomagnetic plot.
%
% Prototype: magplot(mag)
% Input: mag - 3-axis geomagnetic in milli-Gauss
% Output: yaw - geomagnetic yaw with pitch=roll=0
%          
% See also  magyaw, imuplot, insplot, inserrplot, kfplot, gpsplot.

% Copyright(c) 2009-2017, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/03/2017
    yaw = atan2(mag(:,1), mag(:,2));
    myfigure;
    subplot(211), plot(mag(:,end), [mag(:,1:3),normv(mag(:,1:3))]), xygo('\itt / \rms', 'Mag / mGauss');
    subplot(212), plot(mag(:,end), yaw/(pi/180)), xygo('\itt / \rms', 'MagYaw / \circ');