%CTRAJ	Compute a Cartesian trajectory between two points
%
%	TC = TRAJ(T0, T1, N)
%	TC = TRAJ(T0, T1, T)
%
%	Returns a Cartesian trajectory TC from point T0 to T1.  The number
%	of points is N or the length of the given time vector T.
%
%	Each trajectory is a 16xn matrix, with one row per time step, each being
%	a flattened homogeneous transform (use reshape(row',4,4) to restore).
%

%	Copyright (C) Peter Corke 1993

% MOD. HISTORY
%	12/94	track changes to trinterp()

function tt = ctraj(t0, t1, n)
	if length(n) > 1,
		n = length(n);
	end

	tt = [];
	dp = drivepar(t0, t1);

	for i=1:n,
		r = (i-1)/(n-1);
		t = trinterp(t0, dp, r);
		tt = [tt; t(:)'];
	end

	
