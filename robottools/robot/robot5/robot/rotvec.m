%ROTVEC	Rotation about arbitrary axis
%
%	ROTVEC(V, THETA) returns a homogeneous transformation representing a 
%	rotation of THETA about the vector V.
%
%	See also ROTX, ROTY, ROTZ.

% 	Copyright (C) Peter Corke 1990
function r = rotvec(v, t)
	v = unit(v);
	ct = cos(t);
	st = sin(t);
	vt = 1-ct;
	v = v(:);
	r =    [ct		-v(3)*st	v(2)*st
		v(3)*st		ct		-v(1)*st
		-v(2)*st	v(1)*st		ct	];
		r
		v*v'*vt
	r = [v*v'*vt+r zeros(3,1); 0 0 0 1];
