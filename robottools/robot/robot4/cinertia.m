%CINERTIA	Compute the Cartesian manipulator inertia matrix
%
%	CINERTIA(DYN, Q) for an n-axis manipulator returns the nxn symmetric 
%	inertia matrix which relates Cartesian force/torque to Cartesian
%	acceleration.
%	DYN describes the manipulator dynamics and kinematics, and Q is
%	an n element vector of joint state.
%
%	See also INERTIA, DYN, RNE.

%	Copyright (C) 1993 Peter Corke

function Mx = cinertia(dyn, q)
	J = jacob0(dyn, q);
	Ji = inv(J);
	M = inertia(dyn, q);
	Mx = Ji' * M * Ji;
