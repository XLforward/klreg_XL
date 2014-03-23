clear;
load 2Dfishmissing25;
[N,D]=size(X);
[M,D]=size(Y);
 figure,plot_iter(X,Y); title('Before registering');
 opt.maxiter=50;
 opt.dplot=0;
tic;
[A,B,T,d]=icp(X', Y',opt);
toc;
figure,plot_iter(A' ,B',opt); title('after registering');
% fprintf('dis=%.6f\n',d);

X=A';
Y=B';

X=X(1:(N-k+1),:);
Y=Y(k:M,:);
dif=X-Y;
[N,D]=size(X);
dnorm=0;
for i=1:1:N
    dnorm=dnorm+norm(dif(i,:),2);
end
dnorm=dnorm/N

