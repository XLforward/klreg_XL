%FKINE  Forward robot kinematics for serial link manipulator
%
%	FKINE(ROBOT, Q)  computes the forward kinematics for each joint space
%	point defined by Q.  ROBOT is a robot object.
%
% For an n-axis manipulator Q is an n element vector or an m x n matrix. The
% elements are interpretted as joint angle or link length according to
% the form of DH or the j'th sigma value (0 for revolute, other for prismatic).
% 
% If Q is a vector it is interpretted as the generalized joint coordinates, and
% FKINE(ROBOT, Q) returns a 4x4 homogeneous transformation for the final link of
% the manipulator.
%
% If Q is a matrix, the rows are interpretted as the generalized 
% joint coordinates for a sequence of points along a trajectory.  Q(i,j) is
% the j'th joint parameter for the i'th trajectory point.  In this case
% FKINE(ROBOT, Q) returns 3D matrix where the last subscript is the index
% along thepath.
%
%	See also LINKTRAN, MFKINE.

%	Copright (C) Peter Corke 1999

%	6/99	init tt to zeros rather than [], problem with cat() v 5.3

function t = fkine(robot, q)
	%
	% evaluate fkine for each point on a trajectory of 
	% theta_i or q_i data
	%

	n = robot.n;

	L = robot.link;
	if length(q) == n,
		t = robot.base;
		for i=1:n,
			t = t * L{i}(q(i));
		end
		t = t * robot.tool;
	else
		if numcols(q) ~= n,
			error('bad data')
		end
		t = [];
		for qv=q',		% for each trajectory point
			tt = robot.base;
			for i=1:n,
				tt = tt * L{i}(qv(i));
			end
			tt = tt * robot.tool;
			t = cat(3, t, tt);
		end
	end
