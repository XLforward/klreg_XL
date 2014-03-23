%QNORM	Normalize a quaternion
%
%	QN = qnorm(Q)
%
%	Return a unit-quaternion corresponding to the quaternion Q.
%
%	See also TR2Q

%	Copyright (C) 1993 Peter Corke
function qn = qnorm(q)

	qn = q / norm(q);

