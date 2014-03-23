function [ R,t] = klrigid( X,Y,W )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
[N,D]=size(X);
[M,D]=size(Y);
sumW=sum(W(:));
sumWcol=sum(W,2);
mux=((X')*sumWcol)/sumW;
muy=((Y')*sumWcol)/sumW;
NX=X-repmat(mux',[N,1]);
NY=Y-repmat(muy',[M,1]);

A=(NX')*(W')*NY;
[U,S,V]=svd(A);
C=eye(D,D);
C(end,end)=det(U*V');
R=U*C*V';
t=mux-R*muy;
dsumcol=zeros(N,N);
for n=1:1:N
    dsumcol(n,n)=sumWcol(n);
end
s=trace((A')*R)/trace();
% sigma2=(trace((NX')*dsumcol*NX)-trace((A')*R))/(sumW*D);
end

