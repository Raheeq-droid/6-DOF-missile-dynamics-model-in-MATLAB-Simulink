clc; close all; clear all;
 
A = [-1.064  1.000;
      290.26  0.000];
 
B = [-0.25;
     -331.40];
 
C = [-123.34  0.00;
       0.00   1.00];
 
D = [-13.51; 0];
 
sys = ss(A, B, C, D, ...
    'statename',  {'AoA','q'}, ...
    'inputname',  {'\delta_c'}, ...
    'outputname', {'Az','q'});
 
TF  = tf(sys);
TFq = TF(2,1);
 
% LQR
Q = diag([0.1 0.1]);
R = 0.5;
 
[K, ~, ~] = lqr(A, B, Q, R);
 
Acl   = A - B*K;
syscl = ss(Acl, B, C, D, ...
    'statename',  {'AoA','q'}, ...
    'inputname',  {'\delta_c'}, ...
    'outputname', {'Az','q'});
TFc = tf(syscl); TFc = TFc(2,1);
 
% Kalman / LQG
Qbar = diag(0.00015 * ones(1,2));
Rbar = diag(0.55    * ones(1,2));
 
sys_n        = ss(A, [B eye(2)], C, [D zeros(2)]);
[kest, L, ~] = kalman(sys_n, Qbar, Rbar, 0);
 
dT1 = 0.75;
dT2 = 0.25;
 
% Physical constants
R_earth = 6371e3;
Vel     = 1021.08;
m2f     = 3.2811;
 
% Waypoints (WGS-84)
LAT_TARGET  =  34.6588;    LON_TARGET  = -118.769745;  ELEV_TARGET = 795;
LAT_INIT    =  34.2329;    LON_INIT    = -119.4573;    ELEV_INIT   = 10000;
LAT_OBS     =  34.61916;   LON_OBS     = -118.8429;
 
d2r = pi / 180;
 
l1 = LAT_INIT   * d2r;   u1 = LON_INIT   * d2r;
l2 = LAT_TARGET * d2r;   u2 = LON_TARGET * d2r;
dl = l2 - l1;             du = u2 - u1;
 
% Haversine range
a_hav   = sin(dl/2)^2 + cos(l1)*cos(l2)*sin(du/2)^2;
d_horiz = R_earth * 2 * atan2(sqrt(a_hav), sqrt(1 - a_hav));
r_init  = sqrt(d_horiz^2 + (ELEV_TARGET - ELEV_INIT)^2);
 
% Initial bearing (clockwise from North)
brg          = atan2(sin(du)*cos(l2), cos(l1)*sin(l2) - sin(l1)*cos(l2)*cos(du));
yaw_init_deg = mod(brg / d2r, 360);
yaw_init     = yaw_init_deg * d2r;
 
FPA_INIT = -atan((ELEV_INIT - ELEV_TARGET) / d_horiz);
 
% Obstacle bearing
l_o  = LAT_OBS * d2r;  u_o = LON_OBS * d2r;  du_o = u_o - u1;
obs_az = mod(atan2(sin(du_o)*cos(l_o), cos(l1)*sin(l_o) - sin(l1)*cos(l_o)*cos(du_o)) / d2r, 360);
 
fprintf('Range       : %.0f m\n',   r_init);
fprintf('Heading     : %.2f deg\n', yaw_init_deg);
fprintf('FPA         : %.2f deg\n', FPA_INIT / d2r);
fprintf('Time to tgt : %.1f s\n',   r_init / Vel);
 
figure('Name','Step Response');
subplot(2,1,1); step(TFq, 2);  grid on; title('Open-loop  q/\delta_c');
subplot(2,1,2); step(TFc, 2);  grid on; title('Closed-loop (LQR)  q/\delta_c');
 
figure('Name','Pole-Zero');
pzmap(TFq, TFc); grid on; legend('Open-loop','Closed-loop');