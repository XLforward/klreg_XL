%SUBSREF	subscript reference methods on a QUATERNION object
%
%	QUATERNION.d		return a 4-vector of quaternion elements
%	QUATERNION.s		return the scalar component
%	QUATERNION.v		return the vector component
%	QUATERNION.t		return a 4x4 homogeneous transform
%	QUATERNION.r		return a 3x3 orthonormal rotation matrix

%	Copyright (C) 2001 Peter. I. Corke
function v = subsref(q, s)
	if s(1).type  == '.'

		% NOTE WELL:  the following code can't use getfield() since
		% getfield()  uses this, and Matlab will crash!!

		el = char(s(1).subs);
		switch el,
		case 'd',
			v = double(q);
		case 's',
			v = q.s;
		case 'v',
			v = q.v;
		case 't',
			v = q2tr(q);
		case 'r',
			v = q2tr(q);
			v = v(1:3,1:3);
		end
	else
		error('only .field supported')
	end
