% Copyright(c) 2009-2019, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/02/2019
glvs
[nn, ts, nts] = nnts(2, 0.01);
coef = wm2wtcoef(ts, nn);
len = 100;
wm = zeros(len,3);
ki=10; wm(ki,1) = 1*glv.min;  % x陀螺跳变
wm(33,1) = -wm(ki,1);  % x陀螺跳回
len = length(wm); res = zeros(fix(len/nn)-1, 15);
q1 = [1;0;0;0]; q2 = q1; q3 = q1; q4 = q1; q5 = q1; ki = 1;
for k=1:nn:len-nn
    k1 = k+nn-1;
	wmi = wm(k:k1, :);
	phim = cnscl(wmi, 1);  q1 = qmul(q1,rv2q(phim)); % optimal method
    q2 = qrk4bad(q2, wm(k:k1+1, :), nts);  % quaternion Runge-Kutta, 方法1 bad!
    q3 = qrk4(q3, wmi, nts);  % quaternion Runge-Kutta, 方法2 OK!
    q4 = qmul(q4, rv2q(btzrk4(wmi, nts)));  % Bortz Runge-Kutta
    q5 = qmul(q5, qpicard(wmi'*coef, nts));  % quaternion Picard
	res(ki,:) = [q2att(q1); q2att(q2); q2att(q3); q2att(q4); q2att(q5)]';  ki = ki+1;
end
figure
t = (1:size(res,1))*nts;
subplot(131), plot(t, res(:,1:3:end)/glv.sec), xygo('pitch / \prime\prime');
subplot(132), plot(t, res(:,2:3:end)/glv.sec), xygo('roll / \prime\prime');
subplot(133), plot(t, res(:,3:3:end)/glv.sec), xygo('yaw / \prime\prime');