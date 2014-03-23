function animate(robot, q)

	n = robot.n;
	h = robot.handle;
	L = robot.link;

	hr = h(1);
	hx = h(2);
	hy = h(3);
	hz = h(4);
	mag = h(5);

	x = 0;
	y = 0;
	z = 0;
	% compute the link transforms, and record the origin of each frame
	% for the animation.
	t = robot.base;
	for j=1:n,
		t = t * L{j}(q(j));

		x = [x; t(1,4)];
		y = [y; t(2,4)];
		z = [z; t(3,4)];
	end

	if ~isempty(hx),
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
