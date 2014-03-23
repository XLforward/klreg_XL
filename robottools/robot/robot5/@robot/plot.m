%PLOT	Graphical robot animation
%
%	PLOT(ROBOT, Q)
%	PLOTBOT(ROBOT, Q, OPT)
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
%	See also FKINE, ROBOT.

%	Copright (C) Peter Corke 1993

% MOD.HISTORY
%	12/94	make axis scaling adjust to robot kinematic params
%	4/99	use objects

%	Copyright (C) Peter Corke 1999
function plot(robot, tg, opt)

	np = numrows(tg);
	n = robot.n;

	if numcols(tg) ~= n,
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
	L = robot.link;
	reach = 0;
	for i=1:n,
		reach = reach + abs(L{i}.A) + abs(L{i}.D);
	end

	%
 	% setup an axis in which to animate the robot
	%
	if ~ishold,
		clf
		axis([-reach reach -reach reach -reach reach]);
	end
	figure(gcf);		% bring to the top
	title(robot.name)
	xlabel('X')
	ylabel('Y')
	zlabel('Z')
	set(gca, 'drawmode', 'fast');
	grid on
	line('xdata', [0;0], 'ydata', [0;0], 'zdata', [-reach;base], 'color', 'magenta');
	
	% create a line which we will
	% subsequently modify.  Set erase mode to xor for fast
	% update
	%
	hr = line('color', 'yellow', 'erasemode', erasemode);

	hx = [];
	hy = [];
	hz = [];

	if wrist,	
		hx = line('xdata', [0;0], 'ydata', [0;0], 'zdata', [0;base], ...
		 'color', 'red', 'erasemode', 'xor');
		hy = line('xdata', [0;0], 'ydata', [0;0], 'zdata', [0;base], ...
			 'color', 'green', 'erasemode', 'xor');
		hz = line('xdata', [0;0], 'ydata', [0;0], 'zdata', [0;base], ...
			 'color', 'blue', 'erasemode', 'xor');

		mag = reach/10;
	end

	% save the handles in the passed robot object
	robot.handle = [hr hx hy hz mag];
	assignin('base', inputname(1), robot);

	for r=1:repeat,
	    for p=1:np,
		animate(robot, tg(p,:));
	    end
	end
