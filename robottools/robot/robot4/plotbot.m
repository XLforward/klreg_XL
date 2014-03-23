%PLOTBOT	Graphical robot animation
%
%	PLOTBOT(DH, Q)
%	PLOTBOT(DH, Q, OPT)
%
%	produces a graphical animation of a robot from a
%	description of the kinematics, DH, and a joint trajectory Q.
%	For an n-axis manipulator Q is mxn for an m-point trajectory.
%
%	OPT is a character string which may contain the following letters:
%		l	leave trail, that is, dont erase the previous pose
%		w	don't draw the wrist axes
%		r	repeat, play the animation 50 times
%		b<value>	set the height of the robot's base in Z.  Must
%			be the last argument.
%
%	See also FKINE, DH.

%	Copright (C) Peter Corke 1993

% MOD.HISTORY
%	12/94	make axis scaling adjust to robot kinematic params
function plotbot(dh, q, opt)
	np = numrows(q);
	n = numrows(dh);


	if numcols(q) ~= n,
		error('Insufficient columns in q')
	end

	erasemode = 'xor';
	wrist = 1;
	repeat = 1;
	base = 0.0;

	%
	% options
	%
	if nargin == 3,
		mopt = size(opt,2);
		for i=1:mopt,
			if (opt(i) == 'l'), erasemode = 'none'; end;
			if (opt(i) == 'w'), wrist = 0; end;
			if (opt(i) == 'r'), repeat = 50; end;
			if (opt(i) == 'b'),
					base = str2num(opt(i+1:mopt));
					break;
				end;
		end;
	end;
	%
	% simple heuristic to figure the maximum reach of the robot
	%
	reach = sum(abs(dh(:,2))) + sum(abs(dh(:,4)));

	%
 	% setup an axis in which to animate the robot
	%
	clg
	axis([-reach reach -reach reach -reach reach]);
	figure(gcf);		% bring to the top
	xlabel('X')
	ylabel('Y')
	zlabel('Z')
	set(gca, 'drawmode', 'fast');
	grid
	line('xdata', [0;0], 'ydata', [0;0], 'zdata', [-reach;base], 'color', 'magenta');
	
	% create a line which we will
	% subsequently modify.  Set erase mode to xor for fast
	% update
	%
	hr = line('color', 'yellow', 'erasemode', erasemode);
	if wrist,	
		hx = line('xdata', [0;0], 'ydata', [0;0], 'zdata', [0;base], ...
		 'color', 'red', 'erasemode', 'xor');
		hy = line('xdata', [0;0], 'ydata', [0;0], 'zdata', [0;base], ...
			 'color', 'green', 'erasemode', 'xor');
		hz = line('xdata', [0;0], 'ydata', [0;0], 'zdata', [0;base], ...
			 'color', 'blue', 'erasemode', 'xor');

		mag = reach/10;
	end

	for r=1:repeat,
	    for p=1:np,
		% for every trajectory point

		x = 0;
		y = 0;
		z = base;
		% compute the link transforms, and record the origin of each frame
		% for the animation.
		t = [eye(3,3) [0;0;base];0 0 0 1];
		for j=1:n,
			t = t * linktran(dh(j,:), q(p,j));
			x = [x; t(1,4)];
			y = [y; t(2,4)];
			z = [z; t(3,4)];
		end
 
		if wrist,
			%
			% compute the wrist axes
			%
			xv = t*[mag;0;0;1];
			yv = t*[0;mag;0;1];
			zv = t*[0;0;mag;1];

			%
			% update the line segments, wrist axis and links
			%
			set(hx,'xdata',[t(1,4) xv(1)], 'ydata', [t(2,4) xv(2)], ...
				'zdata', [t(3,4) xv(3)]);
			set(hy,'xdata',[t(1,4) yv(1)], 'ydata', [t(2,4) yv(2)], ...
				 'zdata', [t(3,4) yv(3)]);
			set(hz,'xdata',[t(1,4) zv(1)], 'ydata', [t(2,4) zv(2)], ...
				 'zdata', [t(3,4) zv(3)]);
		end
		
		set(hr,'xdata', x, 'ydata', y, 'zdata', z);
		
		drawnow
	    end
	end
