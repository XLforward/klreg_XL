%FRICTION	compute friction torque
%
%	TAU = FRICTION(QD, B, TC)
%	TAU = FRICTION(QD, B, TC+, TC-)
%	TAU = FRICTION(QD, [B TC+ TC-])
%

%	Copyright (C) 1994 Peter. I. Corke

function  tau = friction(dh_dyn, qd)
	[n,nc] = size(dh_dyn);
	if nc < 18
		error('no friction model present');
	end
	if numcols(qd) ~= n
		error('bad data');
	end
	
	b = dh_dyn(:,18);
	if nc >= 19,
		tcp = dh_dyn(:,19);
		if nc >= 20,
			tcm = dh_dyn(:,20);
		else
			tcm = -tcp;
		end
	else
		tcp = zeros(size(b));
		tcm = tcp;
	end
	% refer friction valuesto link
	b = b.*(dh_dyn(:,17).^2);
	tcp = tcp.*dh_dyn(:,17);
	tcm = tcm.*dh_dyn(:,17);
	
	tau = qd*diag(b) + (qd>0)*diag(tcp) + (qd<0)*diag(tcm);