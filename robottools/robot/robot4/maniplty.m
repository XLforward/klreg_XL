%MANIPLTY	Manipulability measure
%
%	M = MANIPLTY(DH, Q)
%	M = MANIPLTY(DYN, Q)
%
%	Computes the manipulability index for the manipulator
%	at the given pose.
%
%	For an n-axis manipulator Q may be an n-element vector, or an m x n
%	joint space trajectory.
%
%	If Q is a vector MANIPLTY returns a scalar manipulability index.
%	If Q is a matrix MANIPLTY returns a column vector of 
%	manipulability indices for each pose specified by Q.
%
%	The first form computes Yoshikawa's manipulability measure which
%	gives an indication of how far the manipulator is from singularities 
%	and thus able to move and exert forces uniformly in all directions.
%
%	The second form computes Asada's manipulability measure based on
%	the Cartesian manipulator inertia matrix.  An n-dimensional 
%	inertia ellipsoid
%		X' M(q) X = 1
%	gives an indication of how well the manipulator can accelerate
%	in each of the Cartesian directions.  The scalar measure computed
%	here is the ratio of the smallest/largest ellipsoid axis.  Ideally
%	the ellipsoid would be spherical, giving a ratio of 1, but in
%	practice will be less than 1.
%
%	See also INERTIA, JACOB0.

%	Copyright (C) 1993 Peter Corke
function w = maniplty(dyn, q)
	n = numrows(dyn);

	if numcols(dyn) > 5,
		% use Asada's measure


		if length(q) == n,
			J = jacob0(dyn, q);
			Ji = inv(J);
			M = inertia(dyn, q);
			Mx = Ji' * M * Ji;
			e = eig(Mx);
			w = min(e) / max(e);
		else
			w = [];
			for Q = q',
				J = jacob0(dyn, Q);
				Ji = inv(J);
				M = inertia(dyn, Q);
				Mx = Ji' * M * Ji;
				e = sqrt(eig(Mx));
				w = [w; min(e)/max(e)];
			end
		end
		

	else
		% use Yoshikawa's measure
		if length(q) == n,
			J = jacob0(dyn, q);
			w = sqrt(det(J * J'));
		else
			w = [];
			for Q = q',
				J = jacob0(dyn, Q);
				w = [w; sqrt(det(J * J'))];
			end
		end
	end
