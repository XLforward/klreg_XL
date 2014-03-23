%MRNE	Compute inverse dynamics via recursive Newton-Euler formulation
%
%	TAU = MRNE(DYN, Q, QD, QDD)
%	TAU = MRNE(DYN, [Q QD])
%
%	Returns the joint torque required to achieve the specified joint position,
%	velocity and acceleration state (specified in modified D-H parameters).
%
%	Gravity is assumed to be acting in the -Z direction with acceleration of
%	9.81m/s/s, but this may be overriden by providing a gravity acceleration
%	vector [gx gy gz].
%
%	TAU = MRNE(DYN, Q, QD, QDD, GRAV)
%	TAU = MRNE(DYN, [Q QD], GRAV)
%
%	An external force/moment acting on the end of the manipulator may also be
%	specified by a 6-element vector [Fx Fy Fz Mx My Mz].
%
%	TAU = MRNE(DYN, Q, QD, QDD, GRAV, FEXT)
%	TAU = MRNE(DYN, [Q QD], GRAV, FEXT)
%
%	where	Q, QD and QDD are row vectors of the manipulator state; pos, vel, and accel.
%
%	The torque computed also contains a contribution due to armature
%	inertia.
%
%	See also DYN, MFDYN, MACCEL, MGRAVLOAD, MINERTIA.
%
%	Should be a MEX file.

%
% verified against MAPLE code, which is verified by examples
%
%  Uses notation similar to "Introduction to Robotics", J.J. Craig.
%

%	Copyright (C) 1995 Peter Corke

function tau = mrne(dh_dyn, a1, a2, a3, a4, a5)
	z0 = [0;0;1];
	grav = 9.81*z0;
	fext = zeros(6, 1);

	n = numrows(dh_dyn);
	if numcols(a1) == 3*n,
		Q = a1(:,1:n);
		Qd = a1(:,n+1:2*n);
		Qdd = a1(:,2*n+1:3*n);
		np = numrows(Q);
		if nargin >= 3,	
			grav = a2;
		end
		if nargin == 4,
			fext = a3;
		end
	else
		np = numrows(a1);
		Q = a1;
		Qd = a2;
		Qdd = a3;
		if numcols(a1) ~= n | numcols(Qd) ~= n | numcols(Qdd) ~= n | ...
			numrows(Qd) ~= np | numrows(Qdd) ~= np,
			error('bad data');
		end
		if nargin >= 5,	
			grav = a4;
		end
		if nargin == 6,
			fext = a5;
		end
	end
	
	%
	% create local vars for mass, COG and inertia matrices
	%
	m = dh_dyn(:,6);	% column vector of link mass
	Pc = dh_dyn(:,7:9);	% matrix of COM data; row per link
	Jm = [];
	for j=1:n,
		% create the symmetric link inertia matrix from DYN data
		J = [	dh_dyn(j,10) dh_dyn(j,13) dh_dyn(j,15); ...
			dh_dyn(j,13) dh_dyn(j,11) dh_dyn(j,14); ...
			dh_dyn(j,15) dh_dyn(j,14) dh_dyn(j,12)	];

		% create  a pseudo 3D matrix 
		Jm = [Jm J];
	end

	tau = zeros(np,n);

	for p=1:np,
		q = Q(p,:)';
		qd = Qd(p,:)';
		qdd = Qdd(p,:)';
	
		Fm = [];
		Nm = [];
		Pm = [];
		Rm = [];
		w = zeros(3,1);
		wd = zeros(3,1);
		v = zeros(3,1);
		vd = grav;

	%
	% init some variables, compute the link rotation matrices
	%
		for j=1:n,
			alpha = dh_dyn(j,1);
			A = dh_dyn(j,2);
			if dh_dyn(j,5) == 0,
				theta = q(j);
				D = dh_dyn(j,4);
			else
				theta = dh_dyn(j,3);
				D = q(j);
			end
		        sa = sin(alpha); ca = cos(alpha);
			st = sin(theta); ct = cos(theta);
 
			%
			% link rotation matrix for mod D-H params: (i-1)/i R
			%    
			R = [   ct      -st     0
				st*ca   ct*ca   -sa
				st*sa   ct*sa   ca];

			%  another pseudo 3D matrix.
			% the i'th element is (i-1)/i R
			Rm = [Rm R];
			P = [A; -D*sa; D*ca];	% (i-1) P i
			Pm = [Pm P];
		end

	%
	%  the forward recursion
	%
		for i=0:(n-1),
			R = Rm(:,3*i+1:3*i+3)';	% (i+1)/i R
			P = Pm(:,i+1);		% i/P/(i+1)

			%
			% trailing underscore means new value
			%
			if dh_dyn(i+1,5) == 0,
				% revolute axis
				w_ = R*w + z0*qd(i+1);
				wd_ = R*wd + cross(R*w,z0*qd(i+1)) + z0*qdd(i+1);
				%v = cross(w,P) + R*v;
				vd_ = R * (cross(wd,P) + ...
					cross(w, cross(w,P)) + vd);

			else
				% prismatic axis
				w_ = R*w;
				wd_ = R*wd;
				%v = R*(z0*qd(i) + v) + cross(w,P);
				vd_ = R*(z0*qdd(i)+vd) + ...
					cross(wd,P) + ...
					cross(w,R*z0*qd(i)) +...
					cross(w, cross(w,P)) + R*vd;
			end
			% update variables
			w = w_;
			wd = wd_;
			vd = vd_;

			J = Jm(:,3*i+1:3*i+3);
			vdhat = cross(wd,Pc(i+1,:)') + ...
				cross(w,cross(w,Pc(i+1,:)')) + vd;
			F = m(i+1)*vdhat;
			N = J*wd + cross(w,J*w);
			Fm = [Fm F];
			Nm = [Nm N];
		end

	%
	%  the backward recursion
	%

		f = fext(1:3);		% force/moments on end of arm
		nn = fext(4:6);

		for i=n:-1:1,
			
			%
			% order of these statements is important, since both
			% nn and f are functions of previous f.
			%
			
			if i == n,
				R = eye(3);
				P = [0;0;0];
			else
				R = Rm(:,3*i+1:3*i+3);	% i/(i+1) R
				P = Pm(:,i+1);		% i/P/(i+1)
			end
			f_ = R*f + Fm(:,i);
			nn_ = Nm(:,i) + R*nn + cross(Pc(i,:)',Fm(:,i)) + ...
				cross(P,R*f);
			
			f = f_;
			nn = nn_;
			if dh_dyn(i,5) == 0,
				% revolute
				tau(p,i) = nn'*z0 + dh_dyn(i,16)*qdd(i)*dh_dyn(i,17)^2;
			else
				% prismatic
				tau(p,i) = f'*(R'*z0) + dh_dyn(i,16)*qdd(i)*dh_dyn(i,17)^2;
			end
		end
	end
