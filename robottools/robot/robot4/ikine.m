%IKINE	Inverse manipulator kinematics
%
%	Q = IKINE(DH, T)
%	Q = IKINE(DH, T, Q)
%	Q = IKINE(DH, T, Q, M)
%
%	Returns the joint coordinates corresponding to the
%	end-effector transform T.  Note that the inverse kinematic solution is
%	generally not unique, and depends on the initial guess Q (which 
%	defaults to 0).
%
%	Q = IKINE(DH, TG)
%	Q = IKINE(DH, TG, Q)
%	Q = IKINE(DH, TG, Q, M)
%
%	Returns the joint coordinates corresponding to
%	each of the transforms in flattened form which are the rows of TG.
%	Returns one row of Q for each input transform.  The initial estimate 
%	of Q for each time step is taken as the solution from the previous 
%	time step.
%
%	If the manipulator has fewer than 6 DOF then this method of solution
%	will fail, since the solution space has more dimensions than can
%	be spanned by the manipulator joint coordinates.  In such a case
%	it is necessary to provide a mask matrix, M, which specifies the 
%	Cartesian DOF (in the wrist coordinate frame) that will be ignored
%	in reaching a solution.  The mask matrix has six elements that
%	correspond to translation in X, Y and Z, and rotation about X, Y and
%	Z respectively.  The value should be 0 (for ignore) or 1.  The number
%	of non-zero elements should equal the number of manipulator DOF.
%
%	Solution is computed iteratively using the pseudo-inverse of the
%	manipulator Jacobian.
%
%	Such a solution is completely general, though much less efficient 
%	than specific inverse kinematic solutions derived symbolically.
%	
%	This approach allows a solution to obtained at a singularity, but 
%	the joint angles within the null space are arbitrarily assigned.
%
%	For instance with a typical 5 DOF manipulator one would ignore
%	rotation about the wrist axis, that is, M = [1 1 1 1 1 0].
%
%
%	See also FKINE, TR2DIFF, JACOB0.
	
%	Copyright (C) 1993 Peter Corke

% MOD.HISTORY
%	2/95	use new 2-argument version of tr2diff(), cleanup

function qt = ikine(dh, tr, q, m)
	%
	%  solution control parameters
	%
	ilimit = 1000;
	stol = 1e-12;

	n = numrows(dh);

	if nargin == 2,
		q = zeros(n, 1);
	else
		q = q(:);
	end
	if nargin == 4,
		m = m(:);
		if numrows(m) ~= 6 
			error('Mask matrix must have 6 elements')
		end
	else
		if numrows(dh) < 6,
			disp('For a manipulator with fewer than 6DOF a mask matrix argument should be specified');
		end
		m = ones(6, 1);
	end
		

	tcount = 0;
	if ishomog(tr),		% single xform case
		nm = 1;
		count = 0;
		while nm > stol,
			e = tr2diff(fkine(dh, q'), tr) .* m;
			dq = pinv( jacob0(dh, q) ) * e;
			q = q + dq;
			nm = norm(dq);
			count = count+1;
			if count > ilimit,
				error('Solution wouldn''t converge')
			end
		end
		qt = q';
	else			% trajectory case
		for i=1:numrows(tr),
			nm = 1;
			T = reshape(tr(i,:), 4,4);
			count = 0;
			while nm > stol,
				e = tr2diff(fkine(dh, q'), T) .* m;
				dq = pinv( jacob0(dh, q) ) * e;
				q = q + dq;
				nm = norm(dq);
				count = count+1;
				if count > ilimit,
					error('Solution wouldn''t converge')
				end
			end
			qt = [qt; q'];
			tcount = tcount + count;
		end
	end
