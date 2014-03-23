%RB	Load kinematic and dynamic data for an eletrohydraulic manipulator at CMTE
%zdl 	11:36AM  5/24/96, updated 6:24PM  5/28/96


rb = [
% alpha	A	theta	D	sigma	m	rx	ry	rz	Ixx	Iyy	Izz	Ixy	Iyz	Ixz	Jm	G	B	Tc+	Tc-
-pi/2	0.12	0	1.725	0	50.0	-0.12	0.2	0	2.0	0.72	2.72	0	0	0	0	0	1.0e-3	0.1	-0.1
pi/2	0	0	0	0	74.0	0	0	0.7396	7.0736	7.0736	0	0	0	0	0	0	1.0e-3	0.1	-0.1
pi/2	0	pi	0	1	105	0	1.2024	0	0.4744	0	0.4744	0	0	0	0	0	1.0e-3	0.1	-0.1
-pi/2	0.20	0	0	0	38	0.06	0	0.125	0.5938	0.7306	0.1368	0	0	0	0	0	1.0e-3	0.1	-0.1
];

qz = [0 0 2.812 0];
qh = [0 pi/2 2.812 pi/2];

