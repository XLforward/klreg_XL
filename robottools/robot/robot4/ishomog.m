%ISHOMOG	test if argument is a homogeneous transformation


function h = ishomog(tr)
	h = all(size(tr) == [4 4]);
