    sys.g = 9.81; %gravity
	sys.t_start = 0.0; % beginning of link motion 
	sys.t_end = 5.0;%total time to excite the two-link system, or time till ball leaves the link end
	sys.L1 = 0.330; % link 1 length
	sys.L2 = 0.433; % link 2 length
	sys.Lc1 = 0.116; %link 1 center mass distance
	sys.Lc2 = 0.222; %link 2 center mass distance
	sys.m1 = 0.674; % mass of link 1
	sys.m2 = 0.307; % mass of link 2
	sys.m3 = 0.074; % mass of ball and magnet
	sys.I1 = 0.00508; %  inertia of link 1 relative to its center of mass
	sys.I2 = 0.00692; %  inertia of link 2 relative to its center of mass
	sys.b1 = 0.010; %  damping coefficient 0.0124
	sys.b2 = 0.006; %  damping coefficient 0.0064
	sys.R = 0.013; %  joint radius
	sys.Rp = 0.026; %  motor pulley radius , original is Rp=0.019
	sys.Lmax = 0.025; % spring displacement range, original is 0.04
	sys.x0 = 0.005; %  initial spring compression
	sys.PI = 3.1415926535897932; %  Pi
	sys.epsilon = 0.5e-2;  %  weight of control signals
	sys.H = 0.810;   % height of (0,0) point from ground level 0.842-0.032
	sys.MAXMOTORTORQUE = 0.9; %  motor stall torque
    sys.Im = 0.00139; % motor rotor inertia, kgm^2 24e-7+47.4e-7
	sys.bm = 0.02; 
    sys.K = 0.9720;
	sys.dumax = 1.8;   %  maximum motor velocity, rad/s
	sys.u10 = 179*sys.PI/180; % position of motor 1 when spring just starts to extend, in rad
	sys.u20 = 185*sys.PI/180; %  position of motor 2 when spring just starts to extend, in rad
	sys.u30 = 191*sys.PI/180; %  position of motor 3 when spring just starts to extend, in rad
	sys.u40 = 177*sys.PI/180; %  position of motor 4 when spring just starts to extend, in rad
	sys.u1i = sys.u10 + sys.x0/sys.Rp; %  initial position of motor 1 before motion starts
	sys.u2i = sys.u20 - sys.x0/sys.Rp;%  initial position of motor 2 before motion starts
	sys.u3i = sys.u30 + sys.x0/sys.Rp; %  initial position of motor 3 before motion starts
	sys.u4i = sys.u40 - sys.x0/sys.Rp; %  initial position of motor 4 before motion starts
	sys.alpha1 = 12400; %  spring coefficient
	sys.beta1 = 1360; %  spring coefficient
	sys.gamma1 = 0; %  spring coefficient
	sys.alpha2 = 13600; %  spring coefficient
	sys.beta2 = 1350; %  spring coefficient
	sys.gamma2 = 0; %  spring coefficient
	sys.alpha3 = 5320;% spring coefficient
	sys.beta3 = 1500; %  spring coefficient
	sys.gamma3 = 0; %  spring coefficient
	sys.alpha4 = 13700; %  spring coefficient
	sys.beta4 = 1410; %  spring coefficient
	sys.gamma4 = 0; %  spring coefficient
    
    udes=zeros(4,1);
    x=zeros(12,1);
    dy = nonlin_eq_DL(x,udes,sys);