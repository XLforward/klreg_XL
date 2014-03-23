%IKINE2


function e = ikine2(q, robot, t)
	tq = fkine(robot, q');
	e = norm(t - tq);
