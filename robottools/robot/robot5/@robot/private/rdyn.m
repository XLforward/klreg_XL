function d = rdyn(r)
	d = [];
	for i = 1:r.n
		d = [d; r.link{i}.dyn];
	end
