%QVMUL	Multiply vector by unit-quaternion
%
%	VT = qvmul(Q, V)
%
%	Rotate the vector V by the unit-quaternion Q.
%
%	See also QQMUL, QINV

%	Copyright (C) 1993 Peter Corke
function vt = qvmul(q, v)
	v = v(:)';
	if numcols(v) ~= 3,
		error('Must be 3-vector');
	end

	v = [0 v];		% turn it into a quaternion
	p = qqmul(qinv(q), qqmul(v,q))
	vt = p(2:4)';

