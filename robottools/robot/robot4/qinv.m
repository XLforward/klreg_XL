%QINV	Invert a unit-quaternion
%
%	QI = qinv(Q)
%
%	Return the inverse of the unit-quaternion Q.
%
%	See also QQMUL

%	Copyright (C) 1993 Peter Corke
function qi = qinv(q)

	qi = [q(1) -q(2:4)] / sum(q.^2);
