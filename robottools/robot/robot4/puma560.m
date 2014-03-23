%PUMA560	Load kinematic and dynamic data for a Puma 560 manipulator
%
%	Defines the matrix 'p560' which describes the kinematic and dynamic
%	characterstics of a Unimation Puma 560 manipulator.
%	Specifies armature inertia and gear ratios.
%
%	See also DH, DYN, TWOLINK, STANFORD.

%
% Notes:
%    -	the value of m1 is given as 0 here.  Armstrong found no value for it
%	and it does not appear in the equation for tau1 after the substituion
%	is made to inertia about link frame rather than COG frame.
% updated:
%	2/8/95  changed D3 to 150.05mm which is closer to data from Lee, AKB86 and Tarn
%		fixed errors in COG for links 2 and 3
%	29/1/91 to agree with data from Armstrong etal.  Due to their use
%		of modified D&H params, some of the offsets Ai, Di are
%		offset, and for links 3-5 swap Y and Z axes.
%	14/2/91 to use Paul's value of link twist (alpha) to be consistant
%		with ARCL.  This is the -ve of Lee's values, which means the
%		zero angle position is a righty for Paul, and lefty for Lee.
%		Note that gravity load torque is the motor torque necessary
%		to keep the joint static, and is thus -ve of the gravity
%		caused torque.
%
%	8/95	fix bugs in COG data for Puma 560. This led to signficant errors in
%		inertia of joint 1. 
%

%	Copright (C) Peter Corke 1990

% alpha	A	theta	D	sigma	m	rx	ry	rz	Ixx	Iyy	Izz	Ixy	Iyz	Ixz	Jm	G	B	Tc+	Tc-
p560 = [
pi/2	0	0	0	0	0	0	0	0	0	0.35	0	0	0	0	200e-6	-62.6111	1.48e-3	.395	-.435
0	.4318	0	0	0	17.4	-.3638	.006	.2275	.13	.524	.539	0	0	0	200e-6	107.815	.817e-3	.126	-.071
-pi/2	.0203	0	.15005	0	4.8	-.0203	-.0141	.070	.066	.086	.0125	0	0	0	200e-6	-53.7063	1.38e-3	.132	-.105
pi/2	0	0	.4318	0	0.82	0	.019	0	1.8e-3	1.3e-3	1.8e-3	0	0	0	33e-6	76.0364	71.2e-6	11.2e-3	-16.9e-3
-pi/2	0	0	0	0	0.34	0	0	0	.3e-3	.4e-3	.3e-3	0	0	0	33e-6	71.923	82.6e-6	9.26e-3	-14.5e-3
0	0	0	0	0	.09	0	0	.032	.15e-3	.15e-3	.04e-3	0	0	0	33e-6	76.686	36.7e-6	3.96e-3	-10.5e-3
];

%
% some useful poses
%
qz = [0 0 0 0 0 0];	% zero angles, L shaped pose
qr = [0 pi/2 -pi/2 0 0 0];	% ready pose, arm up
qstretch = [0 0 -pi/2 0 0 0];
