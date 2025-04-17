clear all
%% physical characterisitics
%clc
format long
%=================
mass_a = 1600; mass_b = 1600;  % mass
mmoi_a =  960; mmoi_b =  960;  % mass moment of inertia 
hlen_a =  1.8; hlen_b =  1.8;  % half-length of cars
hwid_a =  1.0; hwid_b =  1.0;  % half-width  of cars 
uamag  =   30; ubmag  =   20;  % speed (magnitude of linear velocities)
wai    =  0.0; wbi    =  0.0;  % angular velocities 
rcoef  = 0.2 ;                 % restitution coefficient "e" 
theta_degree = 20           ;  % collision angle made using initial velocity
%=================
% geometrical setup
tdir = [1,0] ;  % Tangential direction
ndir = [0,1] ;  % Normal direction 
hdia_a = sqrt(hlen_a^2 + hwid_a^2 ) ;  % distance from the CM to contact point of car A
hdia_b = sqrt(hlen_b^2 + hwid_b^2 ) ;  % distance from the CM to contact point of car B

% For car A
theta_a = theta_degree  * pi/180        ;
angle_a = theta_a + atan(hwid_a/hlen_a) ;
uat     = uamag*cos(theta_a)            ;
uan     = -uamag*sin(theta_a)           ;
uai     = [ uat , uan]                  ;
rvec_at =  hdia_a*cos(angle_a)          ;
rvec_an =  hdia_a*sin(angle_a)          ;
rvec_a  = [rvec_at,  rvec_an ]          ;

% For car B
theta_b = -0.0 * pi/180         ;
ubt     = ubmag*cos(theta_b)    ;
ubn     = ubmag*sin(theta_b)    ;
ubi     = [ ubt , ubn]          ;
rvec_bt = 1.8                   ;
rvec_bn = 1                     ;  
rvec_b  = [rvec_bt, rvec_bn]    ;
% setting up matrix and vector for collision
pn      = mass_a * uan + mass_b * ubn   ;    % 
dev     = rcoef * (ubn-uan)             ;
La      = -mass_a * rvec_at * uan       ;
Lb      = -mass_b * rvec_bt * ubn       ;
pvec    = [ pn; dev;  La; Lb]           ;
GM      = [mass_a         , mass_b              , 0             ,0              ;
	  1               ,-1                   , rvec_at       , -rvec_bt	;
	  -mass_a*rvec_at , 0                   , mmoi_a	, 0             ;
	  0               ,-mass_b*rvec_bt      , 0             , mmoi_b ]      ;
% 
sol  = linsolve(GM,pvec) ;
van = sol(1) ;
vbn = sol(2) ;
waf = sol(3) ;
wbf = sol(4) ;
% 
vat = uat ;
vbt = ubt ;
vaf = [vat, van ];
vbf = [vbt, vbn ];

% display
fprintf('\n=== Pair-Collision Simulation === ')
fprintf('\n=== Parameters === \n')
fprintf('Mass and Mass MOI of "A"       : (Mass_a, I_a   ) = (%+.3f, %+.3f)  \n', mass_a, mmoi_a);
fprintf('Mass and Mass MOI of "A"       : (Mass_b, I_b   ) = (%+.3f, %+.3f)  \n', mass_b, mmoi_b);
fprintf('Half Length and Width of "A"   : (hlen_a, hwid_a) = (%+.3f, %+.3f)  \n', hlen_a, hwid_a);
fprintf('Half Length and Width of "B"   : (hlen_b, hwid_b) = (%+.3f, %+.3f)  \n', hlen_b, hwid_b);
fprintf('Incidence angle and epsilon    : (angle , rcoeff) = (%+.2f, %+.3f)  \n', theta_degree, rcoef);
fprintf('\n=== Results: pre- and post-velocities (tangential, normal, angular) === \n')
fprintf('Pre-collision  velocity of "A" : (uat, uan, waz ) = (%+.3f, %+.3f, %+.3f)\n', uat, uan, wai); 
fprintf('Pre-collision  velocity of "B" : (uat, uan, waz ) = (%+.3f, %+.3f, %+.3f)\n', ubt, ubn, wbi);
fprintf('Post-collision velocity of "A" : (vat, van, waf ) = (%+.3f, %+.3f, %+.3f)\n', vat, van, waf); 
fprintf('Post-collision velocity of "B" : (vat, van, waf ) = (%+.3f, %+.3f, %+.3f)\n', vbt, vbn, wbf);
fprintf('\n');
 
