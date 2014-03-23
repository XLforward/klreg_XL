%RNE	Compute inverse dynamics via recursive Newton-Euler formulation
%
%	TAU = RNE(DYN, Q, QD, QDD)
%	TAU = RNE(DYN, [Q QD QDD])
%
%	Returns the joint torque required to achieve the specified joint position,
%	velocity and acceleration state.
%
%	Gravity is assumed to be acting in the -Z direction with acceleration of
%	9.81m/s/s, but this may be overriden by providing a gravity acceleration
%	vector [gx gy gz].
%
%	TAU = RNE(DYN, Q, QD, QDD, GRAV)
%	TAU = RNE(DYN, [Q QD QDD], GRAV)
%
%	An external force/moment acting on the end of the manipulator may also be
%	specified by a 6-element vector [Fx Fy Fz Mx My Mz].
%
%	TAU = RNE(DYN, Q, QD, QDD, GRAV, FEXT)
%	TAU = RNE(DYN, [Q QD QDD], GRAV, FEXT)
%
%	where	Q, QD and QDD are row vectors of the manipulator state; pos, vel, and accel.
%
%	The torque computed also contains a contribution due to armature
%	inertia.
%
%	See also DYN, FDYN, ACCEL, GRAVLOAD, INERTIA.
%
%	Should be a MEX file.

%
% verified against MAPLE code, which is verified by examples
%

%	Copyright (C) 1992 Peter Corke

% MOD.HISTORY
%       6/95    make use of passed in FEXT 
%       4/95    fix bug in use of passed FEXT 
 

function tau = rne(dh_dyn, a1, a2, a3, a4, a5)
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
	r = dh_dyn(:,7:9);	% matrix of COM data; row per link
	Jm = [];
	for j=1:n,
		J = [	dh_dyn(j,10) dh_dyn(j,13) dh_dyn(j,15); ...
			dh_dyn(j,13) dh_dyn(j,11) dh_dyn(j,14); ...
			dh_dyn(j,15) dh_dyn(j,14) dh_dyn(j,12)	];
		Jm = [Jm J];
	end

	tau = zeros(np,n);

	for p=1:np,
		q = Q(p,:)';
		qd = Qd(p,:)';
		qdd = Qdd(p,:)';
	
		Fm = [];
		Nm = [];
		pstarm = [];
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
 
			R = [   ct      -st*ca  st*sa
				st      ct*ca   -ct*sa
				0       sa      ca];
			Rm = [Rm R];
			pstar = [A; D*sa; D*ca];
			pstarm = [pstarm pstar];
		end

	%
	%  the forward recursion
	%
		for j=1:n,
			R = Rm(:,3*j-2:3*j)';
			pstar = pstarm(:,j);

			%
			% statement order is important here
			%
			if dh_dyn(j,5) == 0,
				% revolute axis
				wd = R*(wd + z0*qdd(j) + ...
					cross(w,z0*qd(j)));
				w = R*(w + z0*qd(j));
				%v = cross(w,pstar) + R*v;
				vd = cross(wd,pstar) + ...
					cross(w, cross(w,pstar)) +R*vd;

			else
				% prismatic axis
				w = R*w;
				wd = R*wd;
				%v = R*(z0*qd(j) + v) + cross(w,pstar);
				vd = R*(z0*qdd(j)+vd) + ...
					cross(wd,pstar) + ...
					2*cross(w,R*z0*qd(j)) +...
					cross(w, cross(w,pstar)) +R*vd;
			end

			J = Jm(:,3*j-2:3*j);
			vhat = cross(wd,r(j,:)') + ...
				cross(w,cross(w,r(j,:)')) + vd;
			F = m(j)*vhat;
			N = J*wd + cross(w,J*w);
			Fm = [Fm F];
			Nm = [Nm N];
		end

	%
	%  the backward recursion
	%

		f = fext(1:3);		% force/moments on end of arm
		nn = fext(4:6);

		for j=n:-1:1,
			pstar = pstarm(:,j);
			
			%
			% order of these statements is important, since both
			% nn and f are functions of previous f.
			%
			if j == n,
				R = eye(3,3);
			else
				R = Rm(:,3*j+1:3*j+3);
			end
			nn = R*(nn + cross(R'*pstar,f)) + ...
				cross(pstar+r(j,:)',Fm(:,j)) + ...
				Nm(:,j);
			f = R*f + Fm(:,j);
			R = Rm(:,3*j-2:3*j);
			if dh_dyn(j,5) == 0,
				% revolute
				tau(p,j) = nn'*(R'*z0) + dh_dyn(j,16)*qdd(j)*dh_dyn(j,17)^2;
			else
				% prismatic
				tau(p,j) = f'*(R'*z0) + dh_dyn(j,16)*qdd(j)*dh_dyn(j,17)^2;
			end
		end
	end
