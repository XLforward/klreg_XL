%SUBSREF	subscript reference methods on a LINK object
%
%	LINK(q)	return link xform matrix
%
%	LINK.alpha		return DH parameters
%	LINK.A
%	LINK.theta
%	LINK.D
%	LINK.sigma		return prismatic flag
%	LINK.RP		return 'R' or 'P'
%	LINK.mdh		0 if standard D&H, else 1
%
%	LINK.I		return 3x3 symmetric inertia matrix
%	LINK.m		return link mass
%	LINK.r		return 3x1 link COG vector
%
%	LINK.G		return gear ratio
%	LINK.Jm		return motor inertia
%	LINK.B		return viscous friction
%	LINK.Tc		return viscous friction
%
%	LINK.dh	return legacy DH row
%	LINK.dyn	return legacy DYN row


%	Copyright (C) 1999 Peter. I. Corke
function v = subsref(l, s)
	if s(1).type  == '()'
		if l.mdh == 0,
			v = linktran([l.alpha l.A l.theta l.D l.sigma], s(1).subs{1});
		else
			v = mlinktran([l.alpha l.A l.theta l.D l.sigma], s(1).subs{1});
		end
	elseif s(1).type  ~= '.'
		error('only .field supported')
	else

		% NOTE WELL:  the following code can't use getfield() since
		% getfield()  uses this, and Matlab will crash!!

		el = char(s(1).subs);
		switch el,
		case 'alpha',
			v = l.alpha;
		case 'A',
			v = l.A;
		case 'theta',
			v = l.theta;
		case 'D',
			v = l.D;
		case 'sigma',
			v = l.sigma;
		case 'RP',
			if l.sigma == 0,
				v = 'R';
			else
				v = 'P';
			end
		case 'mdh',
			v = l.mdh;
		case 'G',
			v = l.G;
		case 'I',
			v = l.I;
		case 'r',
			v = l.r;
		case 'Jm',
			v = l.Jm;
		case 'B',
			v = l.B;
		case 'Tc',
			v = l.Tc;
		case 'm',
			v = l.m;
		case 'dh',
			v = [l.alpha l.A l.theta l.D l.sigma];
		case 'dyn',
			v = [l.alpha l.A l.theta l.D l.sigma ...
			l.m l.r diag(l.I)' l.I(2,1) l.I(2,3) l.I(1,3) l.Jm l.G l.B l.Tc];
		otherwise, disp('Unknown method')
		end
	end
