%CORIOLIS	Compute the manipulator coriolis matrix
%
%	CORIOLIS(DYN, Q, QD) for an n-axis manipulator returns the n element
%	coriolis torque vector at the specified pose and velocity.
%	DYN describes the manipulator dynamics and kinematics.
%
%	If Q and QD are row vectors, CORIOLIS(DYN,Q,QD) is a row vector 
%	of joint torques.
%	If Q and QD are matrices, each row is interpretted as a joint state 
%	vector, and CORIOLIS(DYN,Q,QD) is a matrix each row being the 
%	corresponding joint %	torques.
%
%	See also DYN, RNE, ITORQUE, GRAVLOAD.


%	Copright (C) Peter Corke 1993
function c = coriolis(dyn, q, qd)
	c = rne(dyn, q, qd, zeros(size(q)), [0;0;0]);
