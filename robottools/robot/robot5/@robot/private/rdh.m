function d = rdh(r)
	d = [];
	for i = 1:r.n
		d = [d; r.link{i}.dh];
	end
