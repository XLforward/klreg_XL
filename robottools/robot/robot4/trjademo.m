echo on
%
% Jacobian and differential motion demonstration
%
%  A differential motion can be represented by a 6-element vector with elements
%	[dx dy dz drx dry drz]
%
% where the first 3 elements are a differential translation, and the last 3 are a differential
% rotation.  When dealing with infinitisimal rotations, the order becomes unimportant.  The
% differential motion could be written in terms of compounded transforms
%
%	transl(dx,dy,dz) * rotx(drx) * roty(dry) * rotz(drz)
%
% but a more direct approach is to use the function diff2tr()
%
	D = [.1 .2 0 -.2 .1 .1]';
	diff2tr(D)
pause	% any key to continue
%
% More commonly it is useful to know how a differential motion in one coordinate
% frame appears in another frame.  If the second frame is represented by the transform
	T = transl(100, 200, 300) * roty(pi/8) * rotz(-pi/4);
%
% then the differential motion in the second frame would be given by

	DT = tr2jac(T) * D;
	DT'
%
% tr2jac() has computed a 6x6 Jacobian matrix which transforms the differential 
% changes from the first frame to the next.
%
pause	% any key to continue

%
% Consider a Puma 560 robot in the pose
	q = [0 pi/4 -pi 0 -pi/8 0];

%
% The corresponding Cartesian end-point location can be determined by forward kinematics
	T = fkine(p560, q);
%
% Now if the joint coordinate is perturbed slightly, we can compute a perturbed end-point
% location
	delta = 1e-6;
	Td = fkine(p560, q+[1 0 0 0 0 0]*delta);
%
% and the difference between these transforms is
	dT = Td - T
%
% which may be converted to a differential motion
	D = tr2diff(dT)/delta;
	D'
%
% which indicates that joint 1 velocity contributes to velocity in the 
% X and Y directions and also some rotational velocity about the Z axis
%
pause	% any key to continue

%  Extending this procedure we may build up the manipulator Jacobian matrix
% which relates differential joint coordinate motion to differential Cartesian
% motion;
%		dX = J(q) dQ
%
%  For an n-joint manipulator the manipulator Jacobian is a 6 x n matrix and
% is used is many manipulator control schemes.  For a 6-axis manipulator like
% the Puma 560 the Jacobian is square
%
	J = jacobian(p560, q)
%
%  Note the top right 3x3 block is all zero.  This indicates, correctly, that
% motion of joints 4-6 does not cause any translational motion of the robot's
% end-effector.
pause	% any key to continue

%
%  Many control schemes require the inverse of the Jacobian.  The Jacobian
% in this example is not singular
	det(J)
%
% and may be inverted
	Ji = inv(J)
pause	% any key to continue
%
% A classic control technique is Whitney's resolved rate motion control
%
%	dQ/dt = J(q)^-1 dX/dt
%
% where dX/dt is the desired Cartesian velocity, and dQ/dt is the required
% joint velocity to achieve this.
	vel = [1 0 0 0 0 0]';	% translational motion in the X direction
	qvel = Ji * vel;
	qvel'
%
%  This is an alternative strategy to computing a Cartesian trajectory 
% and solving the inverse kinematics.  However like that other scheme, this
% strategy also runs into difficulty at a manipulator singularity where
% the Jacobian is singular.

pause	% any key to continue
%
% As already stated this Jacobian relates joint velocity to end-effector velocity
% expressed in the end-effector reference frame.  We may wish instead to specify
% the velocity in base or world coordinates.
%
% We have already seen how differential motions in one frame can be translated 
% to another.  Consider the velocity as a differential in the world frame, that
% is, d0X.  We can write
%		d6X = Jac(T6) d0X
%
	T6 = fkine(p560, q);	% compute the end-effector transform
	d6X = tr2jac(T6) * vel;	% translate world frame velocity to T6 frame
	qvel = Ji * d6X;	% compute required joint velocity as before
	qvel'
%
%  Note that this value of joint velocity is quite different to that calculated
% above, which was for motion in the T6 X-axis direction.
pause	% any key to continue
%
%  The Jacobian can provide information about how `well-conditioned' the
% manipulator is for making certain motions.  In some sense this `condition'
% is related to proximity to a singularity.
%
%  A number of scalar manipulability measures have been proposed.  One by
% Yoshikawa
	yoshi(p560, q)
%
% is based purely on kinematic parameters of the manipulator.  Another by
% Asada takes into account the inertia of the manipulator which affects the
% acceleration achievable in different directions.  This measure varies from
% 0 to 1, where 1 indicates uniformity of acceleration in all directions
	asada(p560, q)

%
% Both of these measures indicate that this particular pose is not well
% conditioned.
pause	% any key to continue

%  An interesting class of manipulators are those that are redundant, that is,
% they have more than 6 degrees of freedom.  Computing the joint motion for
% such a manipulator is not straightforward.  Approaches have been suggested
% based on the pseudo-inverse of the Jacobian (which will not be square) or
% singular value decomposition of the Jacobian.
%
echo off