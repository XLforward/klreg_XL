%MDYN	Matrix representation of manipulator kinematics and dynamics
%
%	Some robot toolbox functions take a MDYN matrix which describes the
%	kinematics and dynamics of a manipulator in a general way.
%
%	For an n-axis manipulator, MDYN is an nx20 matrix, whose rows comprise
%	
%	1	alpha(i-1)	link twist angle
%	2	A(i-1)		link length
%	3	theta(i)	link rotation angle
%	4	D(i)		link offset distance
%	5	sigma(i)	joint type, 0 for revolute, non-zero for 
%	6	mass	mass of the link
%	7	rx	link COG with respect to the link coordinate frame
%	8	ry
%	9	rz
%	10	Ixx	elements of link inertia tensor about the link COG
%	11	Iyy
%	12	Izz
%	13	Ixy
%	14	Iyz
%	15	Ixz
%	16	Jm	armature inertia
%	17	G	reduction gear ratio. joint speed/link speed
%	18	B	viscous friction, motor refered
%	19	Tc+	coulomb friction (positive rotation), motor refered
%	20	Tc-	coulomb friction (negative rotation), motor refered
%
%	The first 5 columns of a MDYN matrix contain the kinematic parameters
%	and maybe used anywhere that a MDH kinematic matrix is required -- the
%	dynamic data is ignored.
%
%	See also MDH.

%	Copright (C) Peter Corke 1993

% MOD.HISTORY
%	1/95	reverse labels on A & D
