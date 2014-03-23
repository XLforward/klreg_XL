%QINTERP	Interpolate rotations expressed by quaternion objects
%
%	QI = qinterp(Q1, Q2, R)
%
%	Return a unit-quaternion that interpolates between Q1 and Q2 as R moves
%	from 0 to 1.  This is a spherical linear interpolation (slerp) that can
%	be interpretted as interpolation along a great circle arc on a sphere.
%
%	See also TR2Q

% MOD HISTORY
% 2/99	convert to use of objects

%	Copright (C) Peter Corke 1999
function q = qi(Q1, Q2, r)


	q1 = double(Q1);
	q2 = double(Q2);

	if (r<0) | (r>1),
		error('R out of range');
	end

	theta = acos(q1*q2');

	q = quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) );
