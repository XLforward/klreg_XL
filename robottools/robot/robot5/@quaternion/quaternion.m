%QUATERNION	constructor for quaternion objects
%	
% 	QUATERNION([s v1 v2 v3])	from 4 elements
% 	QUATERNION(v, theta)		from vector plus angle
% 	QUATERNION(R)			from a 3x3 or 4x4 matrix
% 	QUATERNION(q)			from another quaternion

%	Copright (C) Peter Corke 1999
function q = quaternion(a1, a2)


	if nargin == 0,
		q.v = [];
		q.s = [];
		q = class(q, 'quaternion');
	elseif isa(a1, 'quaternion')
		q = a1;
	elseif nargin == 1
		% input is a rotation matrix
		if all(size(a1) == [3 3])
			q = quaternion( tr2q(a1) );
		elseif all(size(a1) == [4 4])
			q = quaternion( tr2q(a1(1:3,1:3)) );
		elseif all(size(a1) == [1 4])
			q = quaternion(a1(1), a1(2:4));
		else
			error('unknown dimension of input');
		end
	elseif nargin == 2 & all(size(a1) == [1 1]) & (length(a2) == 3),
		% <s vec> form
		q.s = sin(a1/2);
		q.v = a2(:).';
		q = class(q, 'quaternion');
	else
		error('bad arguments');
	end

%TR2Q	Convert homogeneous transform to a unit-quaternion
%
%	Q = tr2q(T)
%
%	Return a unit quaternion corresponding to the rotational part of the
%	homogeneous transform T.
%
%	See also Q2TR

%	Copyright (C) 1993 Peter Corke
function q = tr2q(t)
	kx = t(3,2) - t(2,3);	% Oz - Ay
	ky = t(1,3) - t(3,1);	% Ax - Nz
	kz = t(2,1) - t(1,2);	% Ny - Ox

	if (t(1,1) >= t(2,2)) & (t(1,1) >= t(3,3)) 
		kx1 = t(1,1) - t(2,2) - t(3,3) + 1;	% Nx - Oy - Az + 1
		ky1 = t(2,1) + t(1,2);			% Ny + Ox
		kz1 = t(3,1) + t(1,3);			% Nz + Ax
		add = (kx >= 0);
	elseif (t(2,2) >= t(3,3))
		kx1 = t(2,1) + t(1,2);			% Ny + Ox
		ky1 = t(2,2) - t(1,1) - t(3,3) + 1;	% Oy - Nx - Az + 1
		kz1 = t(3,2) + t(2,3);			% Oz + Ay
		add = (ky >= 0);
	else
		kx1 = t(3,1) + t(1,3);			% Nz + Ax
		ky1 = t(3,2) + t(2,3);			% Oz + Ay
		kz1 = t(3,3) - t(1,1) - t(2,2) + 1;	% Az - Nx - Oy + 1
		add = (kz >= 0);
	end

	if add
		kx = kx + kx1;
		ky = ky + ky1;
		kz = kz + kz1;
	else
		kx = kx - kx1;
		ky = ky - ky1;
		kz = kz - kz1;
	end
	nm = norm([kx ky kz]);
	if nm == 0,
		q = quaternion(1, [0 0 0]);
	else
		s = sqrt(trace(t))/2;
		l = sqrt(1 - q(1)^2) / nm;
		v = l*[kx ky kz];
		q = quaternion(s, v);
	end
