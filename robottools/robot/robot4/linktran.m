%LINKTRAN	Compute the link transform from kinematic parameters
%
%	LINKTRAN(alpha, an, theta, dn)
%	LINKTRAN(DH, q) is a homogeneous 
%	transformation between link coordinate frames.
%
%	alpha is the link twist angle
%	an is the link length
%	theta is the link rotation angle
%	dn is the link offset
%	sigma is 0 for a revolute joint, non-zero for prismatic
%
%	In the second case, q is substitued for theta or dn according to sigma.
%
%	Based on the standard Denavit and Hartenberg notation.

%	Copright (C) Peter Corke 1993
function t = linktran(a, b, c, d)

	if nargin == 4,
		alpha = a;
		an = b;
		theta = c;
		dn = d;
	else
		if numcols(a) < 4,
			error('too few columns in DH matrix');
		end
		alpha = a(1);
		an = a(2);
		if numcols(a) > 4,
			if a(5) == 0,	% revolute
				theta = b;
				dn = a(4);
			else		% prismatic
				theta = a(3);
				dn = b;
			end
		else
			theta = b;	% assume revolute if sigma not given
			dn = a(4);
		end
	end
	sa = sin(alpha); ca = cos(alpha);
	st = sin(theta); ct = cos(theta);

	t =    [	ct	-st*ca	st*sa	an*ct
			st	ct*ca	-ct*sa	an*st
			0	sa	ca	dn
			0	0	0	1];
