%JTRAJ	Compute a joint space trajectory between two points
%
%	[Q QD QDD] = JTRAJ(Q0, Q1, N)
%	[Q QD QDD] = JTRAJ(Q0, Q1, N, QD0, QD1)
%	[Q QD QDD] = JTRAJ(Q0, Q1, T)
%	[Q QD QDD] = JTRAJ(Q0, Q1, T, QD0, QD1)
%
%	Returns a joint space trajectory Q from state Q0 to Q1.  The number
%	of points is N or the length of the given time vector T.  A 7th
%	order polynomial is used with default zero boundary conditions for
%	velocity and acceleration.  Non-zero boundary velocities can be
%	optionally specified as QD0 and QD1.
%
%	The function can optionally return a velocity and acceleration
%	trajectories as QD and QDD.
%
%	Each trajectory is an mxn matrix, with one row per time step, and
%	one column per joint parameter.
%

%	Copright (C) Peter Corke 1993

%  MOD.HISTORY
%	1/95 add support for initial velocities as advertised
%	8/95 fix bug with initial conditions

function [qt,qdt,qddt] = jtraj(q0, q1, tv, qd0, qd1)
	if length(tv) > 1,
		tscal = max(tv);
		t = tv(:)/tscal;
	else
		tscal = 1;
		t = [0:(tv-1)]'/(tv-1);	% normalized time from 0 -> 1
	end

	q0 = q0(:);
	q1 = q1(:);

	if nargin == 3,
		qd0 = zeros(size(q0));
		qd1 = qd0;
	end

	% compute the polynomial coefficients
	A = 6*(q1 - q0) - 3*(qd1+qd0);
	B = -15*(q1 - q0) + 8*qd0 + 7*qd1;
	C = 10*(q1 - q0) - 6*qd0 - 4*qd1;
	E = qd0;
	F = q0;


	tt = [t.^5 t.^4 t.^3 t.^2 t ones(size(t))];
	c = [A B C zeros(size(A)) E F]';
	
	qt = tt*c;

	% compute optional velocity
	if nargout >= 2,
		c = [ zeros(size(A)) 5*A 4*B 3*C  zeros(size(A)) E ]';
		qdt = tt*c/tscal;
	end

	% compute optional acceleration
	if nargout == 3,
		c = [ zeros(size(A))  zeros(size(A)) 20*A 12*B 6*C  zeros(size(A))]';
		qddt = tt*c/tscal*2;
	end
