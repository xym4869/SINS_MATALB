% Simulate SIMU sensor outputs form ap=[attitude,position].
% See also  av2avp, avp2imu, sinsp.
glvs
fname = 'leador';
load H:\ygm2015\leador2015\data291.mat; avp610(:,end)=avp610(:,end)-avp610(1,end); % insplot(avp610);
ap0 = avp610(1:4:end,[1:3,7:9,end]);  gts = diff(ap0(1:2,end));
%% transform to browse in Google Earth
pos0 = [dms2r([373038.90; 1045613]); 1427];
ap0 = apmove(ap0, pos0, ap0(1,3)-40*glv.deg);
ap0 = apscale(ap0, fix([400,1600]/gts), 2.3);
kgps = mod(ap0(:,end),1)<gts/2 | mod(ap0(:,end),1)>1-gts/2;
pos2gpx(fname, ap0(kgps,4:6));
%% to IMU
ts = 0.01;
s_avp = ap2avp(ap0, ts);
s_imu = avp2imu(s_avp); % imuplot(s_imu);
imuerr = imuseterr(0.00, 0.00002, 0, 0.1);
s_imu1 = imuadderr(s_imu, imuerr);
imufile(fname, s_imu1(1+95-95:end,:), s_avp(1+95-95,:)', ts, [0.01,1]);
trj.ts = ts; trj.avp0 = s_avp(96-95,1:9)'; trj.imu = s_imu(96-95:end,:); trj.avp = s_avp(97-95:end,:); 
trjfile(fname, trj);
%% GPS simu
kgps = mod(s_avp(:,end),1)<ts/2 | mod(s_avp(:,end),1)>1-ts/2;  % at integer second
gps1 = gpssimu(s_avp(kgps,:), 0.1, 1.0, 10, [0.5;1;2]*0.1, 0, 1);
avpfile(fname, s_avp(kgps,:));
%% pure SINS
nn = 2; nts = nn*ts;
ss = insinit(s_avp(1,:)', ts);
len = length(s_imu1);    s_avp1 = zeros(fix(len/nn), 10);
ki = timebar(nn, len, 'Pure inertial navigation processing.');
for k=1:nn:len-nn+1
	k1 = k+nn-1;
	wvm = s_imu1(k:k1, 1:6);  t = s_imu1(k1,7);
	ss = insupdate(ss, wvm);   ss.vn(3) = s_avp(k1+1,6); %ss.vn = s_avp(k1+1,4:6)';
	s_avp1(ki,:) = [ss.avp; t]';
	ki = timebar;
end
s_avperr = avpcmp(s_avp1, s_avp);
% navplot(s_avp1);
inserrplot(s_avperr);
%% pure SINS verifies after quantization
[imuVer, avpVer0, tsVer] = imufile(fname);
nn = 3; nts = nn*tsVer;
ssVer = insinit(avpVer0, tsVer);
len = length(imuVer);    avpVer = zeros(fix(len/nn), 10);
ki = timebar(nn, len, 'Pure inertial navigation processing.');
for k=1:nn:len-nn+1
	k1 = k+nn-1;
	wvm = imuVer(k:k1, 1:6);  t = imuVer(k1,7);
	ssVer = insupdate(ssVer, wvm);   % ss.vn(3) = s_avp(k1+1,6);
	avpVer(ki,:) = [ssVer.avp; t]';
	ki = timebar;
end
avperr = avpcmp(avpVer, s_avp);  % insplot(avpVer);
inserrplot(avperr);

