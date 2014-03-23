%MRDIVIDE	compute quotient of two quaternion objects

%	Copright (C) Peter Corke 1999
function qq = mrdivide(Q1, Q2)

	if isa(Q2, 'quaternion'),
		% qq = q1 / q2
		%    = q1 * qinv(q2)

		qq = q1 * qinv(q2);

	elseif isa(Q2, 'double') & all(size(Q2) == [1 1])
		qq = quaternion( Q1.s/Q2, Q1.v/Q2);
	else
		error('cant divide quaternion by this');
	end

	
