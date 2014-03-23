%MFKINE  Forward robot kinematics for serial link manipulator
%
%	MFKINE(DH, Q)  computes the forward kinematics for each joint space
%	point defined by Q.  DH describes the manipulator kinematics in modified
%	Denavit Hartenberg notation.
%
% DH has one row of kinematic parameters for each axis.  Each row is of
% the form
%		[alpha A theta D]	for an all revolute manipulator
%	or	[alpha A theta D sigma]	for a mixed revolute/hybrid manipulator
%
% For an n-axis manipulator Q is an n element vector or an m x n matrix. The
% elements are interpretted as joint angle or link length according to
% the form of DH or the j'th sigma value (0 for revolute, other for prismatic).
% 
% If Q is a vector it is interpretted as the generalized joint coordinates, and
% MFKINE(DH, Q) returns a 4x4 homogeneous transformation for the final link of
% the manipulator.
%
% If Q is a matrix, the rows are interpretted as the generalized 
% joint coordinates for a sequence of points along a trajectory.  Q(i,j) is
% the j'th joint parameter for the i'th trajectory point.  In this case
% MFKINE(DH, Q) returns an m x 16 matrix with each row containing a 'flattened' 
% homogeneous transform corresponding to the input joint state.  A row can 
% be unflattened using reshape(v, 4, 4).
%
%	See also MLINKTRANS, FKINE.

%	Copright (C) Peter Corke 1993

function t = mfkine(dh, q)
	%
	% evaluate fkine for each point on a trajectory of 
	% theta_i or q_i data
	%

	n = numrows(dh);
	if numcols(q) ~= n,
		error('bad data')
	end
	
	if length(q) == n,
		t = eye(4,4);
		for i=1:n,
			t = t * mlinktra(dh(i,:), q(i));
		end
	else
		t = [];
		for qv=q',		% for each trajectory point
			tt = eye(4,4);
			for i=1:n,
				tt = tt * mlinktra(dh(i,:), qv(i));
			end
		t = [t; tt(:)'];
		end
	end
