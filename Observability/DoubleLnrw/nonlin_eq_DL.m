function dy = nonlin_eq_DL(x,udes,sys)
    q1 =x(1);
    q2 =x(2);
    dq1=x(3);
    dq2=x(4);
    u1 =x(5);
    u2 = x(6);
    u3 =x(7);
    u4 = x(8);
    
    x1 = sys.x0-sys.R*(q1+sys.PI/2)+sys.Rp*(u1-sys.u1i); %  spring displacements
	x2 = sys.x0+sys.R*(q1+sys.PI/2)-sys.Rp*(u2-sys.u2i);
	x3 = sys.x0+sys.R*q2+sys.Rp*(u3-sys.u3i);
	x4 = sys.x0-sys.R*q2-sys.Rp*(u4-sys.u4i);

	T1 = sys.alpha1*x1*x1 + sys.beta1*x1 + sys.gamma1; %  tendon tension
	T2 = sys.alpha2*x2*x2 + sys.beta2*x2 + sys.gamma2;
	T3 = sys.alpha3*x3*x3 + sys.beta3*x3 + sys.gamma3;
	T4 = sys.alpha4*x4*x4 + sys.beta4*x4 + sys.gamma4;

	MT1 = T1*sys.Rp; %  motor torque 1
	MT2 = T2*sys.Rp; %  motor torque 2
	MT3 = T3*sys.Rp; %  motor torque 3
	MT4 = T4*sys.Rp; %  motor torque 4
    
	torque1 = (T1-T2)*sys.R; %  torque by two elastic elements on joint 1
	torque2 = (T4-T3)*sys.R; %  torque by two elastic elements on joint 2

    g=sys.g;
    I1 = sys.I1;
	I2 = sys.I2;
    Lc1= sys.Lc1;
    Lc2= sys.Lc2;
    L1 = sys.L1;
    L2 = sys.L2;
    m1 = sys.m1;
    m2 = sys.m2;
    m3 = sys.m3;
    
    m11 = I1+m1*Lc1*Lc1+I2+m2*(L1*L1+2*L1*Lc2*cos(q2)+Lc2*Lc2)+m3*(L1*L1+2*L1*L2*cos(q2)+L2*L2); 
	m12 = I2+m2*(Lc2*Lc2+L1*Lc2*cos(q2))+m3*(L2*L2+L1*L2*cos(q2)); 
	m21 = I2+m2*(Lc2*Lc2+L1*Lc2*cos(q2))+m3*(L2*L2+L1*L2*cos(q2));
	m22 = I2+m2*Lc2*Lc2+m3*L2*L2;

	c1 = -(m2*Lc2+m3*L2)*L1*sin(q2)*(2*dq1*dq2+dq2*dq2); 
	c2 =  (m2*Lc2+m3*L2)*L1*sin(q2)*dq1*dq1; 

	d1 = sys.b1*dq1;
	d2 = sys.b2*dq2;    
    
    g1 = g*(m1*Lc1+m2*L1+m3*L1)*cos(q1)+g*(m2*Lc2+m3*L2)*cos(q1+q2);
    g2 = g*(m2*Lc2+m3*L2)*cos(q1+q2);
    M(1,1) = m11;
	M(1,2) = m12;
	M(2,1) = m21;
	M(2,2) = m22;
    
    V = [torque1-c1-d1-g1; ...
         torque2-c2-d2-g2];
     
    dd = M\V;
    
    ddq1 = dd(1)*1.0;
    ddq2 = dd(2)*1.0;
    
    dy1 = x(3);
    dy2 = x(4);
    dy3 = ddq1;
    dy4 = ddq2;
    dy5 = x(9);
    dy6 = x(10);
    dy7 = x(11);
    dy8 = x(12);
    dy9 = (sys.K*udes(1) - MT1 - sys.bm*x(9)) /sys.Im;
    dy10= (sys.K*udes(2) - MT2 - sys.bm*x(10))/sys.Im;
    dy11= (sys.K*udes(3) - MT3 - sys.bm*x(11))/sys.Im;
    dy12= (sys.K*udes(4) - MT4 - sys.bm*x(12))/sys.Im;
    dy = [dy1 ;  dy2; dy3 ;  dy4; dy5 ;  dy6; dy7 ;  dy8; dy9 ;  dy10;dy11;  dy12];
end