% Simulate SIMU sensor outputs form ap=[attitude,position].
% See also  av2avp, avp2imu, sinsp.
glvs
fname = 'leador';
ap0 = ps.iavp(1:3500/0.05,[1:3,7:9,end]);  gts = diff(ap0(1:2,end));
%% transform to browse in Google Earth
% ap0 = apscale(ap0, [100,1155]*round(1/gts), 230/130);
pos0 = [dms2r([345402.90; -1175153.60]); 693.456];
ap0 = apmove(ap0, pos0, 187*glv.deg);
kgps = mod(ap0(:,end),1)<gts/2 | mod(ap0(:,end),1)>1-gts/2;
pos2gpx(fname, ap0(kgps,4:6));
%% to IMU
ts = 0.01;
s_avp = ap2avp(ap0, ts);
s_imu = avp2imu(s_avp); % imuplot(s_imu);
imuerr = imuseterr([0.001; 0.001; 5; 1]*0);
s_imu1 = imuadderr(s_imu, imuerr);
imufile(fname, s_imu1(1+95:end,:), s_avp(1+95,:)');
trj.ts = ts; trj.avp0 = s_avp(96,1:9)'; trj.imu = s_imu(96:end,:); trj.avp = s_avp(97:end,:); 
trjfile(fname, trj);
%% GPS simu
kgps = mod(s_avp(:,end),1)<ts/2 | mod(s_avp(:,end),1)>1-ts/2;  % at integer second
gps1 = gpssimu(s_avp(kgps,:), 1.0, 10, [0.5;1;2]*0.1, 0, 1);
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
nn = 4; nts = nn*tsVer;
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
avperr = avpcmp(avpVer, s_avp);
% inserrplot(avperr);

