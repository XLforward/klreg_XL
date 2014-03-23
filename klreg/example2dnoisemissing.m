clear;
clc;
load 2Dfishmissing25;
[N,D]=size(X);
[M,D]=size(Y);
OX=X;
OY=Y;
figure1 = figure('Color',[1 1 1])
plot_iter(X,Y); title('Before registering');
 axis off;
 tic;
[T,R,t,X,Y]=kl_register(X,Y,30,0.2);
toc;

OY=OY*(T.R)+repmat(T.t,[N,1]);

figure1 = figure('Color',[1 1 1])
plot_iter(OX,OY); title('Our Method');
axis off;

X=X(1:(N-k+1),:);
Y=Y(k:M,:);
dif=X-Y;
dnorm=0;
[N,D]=size(X);
for i=1:1:N
    dnorm=dnorm+norm(dif(i,:),2);
end
dnorm=dnorm/N