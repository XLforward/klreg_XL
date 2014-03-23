%MDH	Matrix representation of modified manipulator kinematics
%
%	Many robot toolbox functions take a MDH matrix which describes the
%	kinematics of a manipulator in a general way using the conventions
%	of Craig.
%
%	For an n-axis manipulator, MDH is an nx4 or nx5 matrix, whose rows 
%	comprise
%	
%	1	alpha(i-1)	link twist angle
%	2	A(i-1)		link length
%	3	theta(i)	link rotation angle
%	4	D(i)		link offset distance
%	5	sigma(i)	joint type, 0 for revolute, non-zero for 
%				  prismatic
%
%	If the last column is not given, most toolbox functions assume that
%	the manipulator is all-revolute.
%
%	The first 5 columns of a MDYN matrix contain the kinematic parameters
%	and maybe used anywhere that a MDH kinematic matrix is required -- the
%	dynamic data is ignored.
%
%	See also MDYN, DH, DYN.

% MOD.HISTORY
%	1/95	reverse labels on A & D%	Copright (C) Peter Corke 1993
