%JACOBN	Compute manipulator Jacobian in end-effector frame
%
%	JACOBN(ROBOT, Q) returns a Jacobian matrix for the current pose Q.
%
% 	The manipulator Jacobian matrix maps differential changes in joint space
%	to differential Cartesian motion of the end-effector.
%			dX = J dQ
%
%	This function uses the technique of
%		Paul, Shimano, Mayer
%		Differential Kinematic Control Equations for Simple Manipulators
%		IEEE SMC 11(6) 1981
%		pp. 456-460
%
%	For an n-axis manipulator the Jacobian is a 6 x n matrix.
%
%	See also DIFF2TR, TR2DIFF, DIFF

% MOD.HISTORY
%	3/99	uses objects
%	10/01	handle Craig's conventions
%
%	Copyright (C) Peter Corke 1999
function J = jacobn(robot, q)

	n = robot.n;
	L = robot.link;		% get the links

	J = [];
	U = robot.tool;
	for j=n:-1:1,
		if robot.mdh == 0,
			% standard DH convention
			U = L{j}( q(j) ) * U;
		end
		if L{j}.RP == 'R',
			% revolute axis
			d = [	-U(1,1)*U(2,4)+U(2,1)*U(1,4)
				-U(1,2)*U(2,4)+U(2,2)*U(1,4)
				-U(1,3)*U(2,4)+U(2,3)*U(1,4)];
			delta = U(3,1:3)';	% nz oz az
		else
			% prismatic axis
			d = U(3,1:3)';		% nz oz az
			delta = zeros(3,1);	%  0  0  0
		end
		J = [[d; delta] J];

		if robot.mdh ~= 0,
			% modified DH convention
			U = L{j}( q(j) ) * U;
		end
	end
