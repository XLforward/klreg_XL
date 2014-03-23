%TRINTERP	Interpolate homogeneous transformations
%
%	T = TRINTERP(T0, T1, R)
%	T = TRINTERP(T0, DP, R)
%
%	Returns a homogeneous transform interpolation between T0 and T1 as
%	R varies from 0 to 1.
%
%	The second form uses a drive parameter matrix computed by DRIVEPAR
%	which represents the `difference' between T0 and T1 and may
%	be more efficient for computing many points between the same 
%	two endpoints.
%
%	Robot manipulators: mathematics, programming and control
%	R.P. Paul, MIT Press, 1981.
%
%	See also CTRAJ, DRIVEPAR.

%	Copyright (C) 1993 Peter Corke

% MOD.HISTORY
%	12/94	must have T0 as well as DP
%	3/96	fixed bug: sin/cos(dp(4)) should be of dp(6)

function t = trinterp(T0, a2, r)
	D = zeros(4,4);

	% eqn (5.69)

	% Using the notation of Paul, dp is
	%	dp(1)	x
	%	dp(2)	y
	%	dp(3)	z
	%	dp(4)	phi
	%	dp(5)	theta
	%	dp(6)	psi

	if ishomog(a2),
		dp = drivepar(T0, a2);
	else
		dp = a2;
	end
	srp = sin(dp(4)*r);	% r*phi
	crp = cos(dp(4)*r);
	srt = sin(dp(5)*r);	% r*theta
	crt = cos(dp(5)*r);
	vrt = 1 - crt;
	sp = sin(dp(6));	% psi
	cp = cos(dp(6));

	D(1,2) = -srp*(sp^2*vrt+crt)+crp*(-sp*cp*vrt);
	D(2,2) = -srp*(-sp*cp*vrt)+crp*(cp^2*vrt+crt);
	D(3,2) = -srp*(-cp*srt)+crp*(-sp*srt);

	D(1,3) = cp*srt;
	D(2,3) = sp*srt;
	D(3,3) = crt;

	D(1:3,1) = cross(D(1:3,2), D(1:3,3));

	D(:,4) = [dp(1:3)*r 1]';

	t = T0*D;
