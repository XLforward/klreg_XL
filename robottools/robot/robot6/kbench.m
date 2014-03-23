q = [.1 .2 .3 .4 .5 .6]
puma560
t6=fkine(p560,t6)
tic
for i=1:100,
	qq= ikine(p560,t6);
end
t=toc;
t/100
tic
for i=1:100,
	qq= ikine560(p560,t6);
end
t=toc;
t/100
