%FDYN	Integrate forward dynamics
%
%	[T Q QD] = FDYN(DYN, T0, T1)
%	[T Q QD] = FDYN(DYN, T0, T1, TORQFUN)
%	[T Q QD] = FDYN(DYN, T0, T1, TORQFUN, Q0, QD0)
%
%	Integrates the manipulator dynamics over the time interval and returns 
%	vectors of joint position and velocity.  A control torque may be specified
%	by a user specified function
%
%		TAU = TORQFUN(T, X)
%
%	where X is a 2n-element column vector [Q; QD], and T is the current time.
%	If TORQFUN is not specified then zero torque is applied to the manipulator.
%
%	See also ACCEL, RNE, DYN, ODE45.

%	Copyright (C) 1993 Peter Corke

function [t, q, qd] = fdyn(dyn, t0, t1, torqfun, q0, qd0)
	global	FDYN_DYN FDYN_TORQFUN;

	n = numrows(dyn);
	if nargin == 3,
		FDYN_TORQFUN = 0;
		x0 = zeros(2*n,1);
	elseif nargin == 4,
		FDYN_TORQFUN = torqfun;
		x0 = zeros(2*n, 1);
	elseif nargin == 6,
		FDYN_TORQFUN = torqfun;
		x0 = [q0(:); qd0(:)];
	end
	FDYN_DYN = dyn;
		
	[t,y] = ode45('fdyn2', t0, t1, x0);
	q = y(:,1:n);
	qd = y(:,n+1:2*n);