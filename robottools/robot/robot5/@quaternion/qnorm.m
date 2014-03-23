%QUNIT	Return an equivalent unit quaternion
%
%	QU = qunit(Q)
%
%	Return a unit-quaternion corresponding to the quaternion Q.
%
%	See also TR2Q

%	Copyright (C) 1993 Peter Corke
function qu = qunit(q)

	qu = q / norm(double(q));

