%ACCEL	Compute manipulator forward dynamics
%
%	QDD = ACCEL(DYN, Q, QD, TORQUE) returns a vector of joint accelerations
%	that result from applying the actuator TORQUE to the manipulator in state
%	Q and QD.
%
%	Uses the method 1 of Walker and Orin to compute the forward dynamics.  This
%	form is useful for simulation of manipulator dynamics, in conjunction with
%	a numerical integration function.
%
%	See also RNE, DYN, ODE45.

%	Copyright (C) 1993 Peter Corke

function qdd = accel(dyn, q, qd, torque)
	q = q(:)';
	qd = qd(:)';
	M = inertia(dyn, q);	% compute current manipulator inertia
	tau = rne(dyn, q, qd, zeros(size(q)));	% compute gravity and coriolis torque

	qdd = inv(M) * (torque(:) - tau');