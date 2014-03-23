function [ R,t,s,sigma2] = computetransform( X,Y,W )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
[N,D]=size(X);
[M,D]=size(Y);
sumW=sum(W(:));
sumWY=sum(W,2);
sumWX=sum((W'),2);

mux=((X')*sumWX)/sumW;
muy=((Y')*sumWY)/sumW;

NX=X-repmat(mux',[N,1]);
NY=Y-repmat(muy',[M,1]);

A=(NX')*(W')*NY;
[U,S,V]=svd(A);
C=eye(D,D);
C(end,end)=det(U*V');
R=U*C*V';
t=mux-R*muy;
dW1=zeros(M,M);
dW_1=zeros(N,N);
for m=1:1:M
    dW1(m,m)=sumWY(m);
end
for n=1:1:N
    dW_1(n,n)=sumWX(n);
end
s=trace((A')*R)/trace((NY')*dW1*NY);
sigma2=(trace((NX')*dW_1*NX)-s*trace((A')*R))/(sumW*D);
% sigma2=(trace((NX')*dsumcol*NX)-trace((A')*R))/(sumW*D);
end

