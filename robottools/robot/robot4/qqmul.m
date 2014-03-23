%QQMUL	Multiply unit-quaternion by unit-quaternion
%
%	QQ = qqmul(Q1, Q2)
%
%	Return a product of unit-quaternions.
%
%	See also TR2Q

%	Copyright (C) 1993 Peter Corke
function qq = qqmul(q1, q2)

	% decompose into scalar and vector components
	s1 = q1(1);	v1 = q1(2:4);
	s2 = q2(1);	v2 = q2(2:4);

	% form the product
	qq = [s1*s2-v1*v2' s1*v2+s2*v1+cross(v1,v2)];


