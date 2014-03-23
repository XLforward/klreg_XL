%INERTIA	Compute the manipulator inertia matrix
%
%	INERTIA(DYN, Q) for an n-axis manipulator returns the nxn symmetric 
%	inertia matrix which relates joint torque to joint acceleration.
%	DYN describes the manipulator dynamics and kinematics, and Q is
%	an n element vector of joint state.
%
%	See also DYN, RNE, ITORQUE, CORIOLIS, GRAVLOAD.


%	Copright (C) Peter Corke 1993
function M = inertia(dyn, q)
	n = numrows(dyn);
	M = rne(dyn, ones(n,1)*q, zeros(n,n), eye(n), [0;0;0]);
	
