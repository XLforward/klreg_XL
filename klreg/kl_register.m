
%
function [T,R,t,X,Y ] = kl_register(X,Y,MaxIter,outliers)

[N,D1] = size(X);
[M,D2] = size(Y);
if D1~=D2, error('The dimension of point-sets is not the same.'); end;

sumdis=0;
 for i=1:1:N
    for j=1:1:M
        sumdis=sumdis+(X(i,:)-Y(j,:))*(X(i,:)-Y(j,:))';
    end
 end
 sigma2=sumdis/(N*M*D1);

X = double (X);
Y = double (Y);
X_=X;
R = eye(3);
t = [0;0;0];
sigma2_con = 1e-8;
iter = 0;
%MaxIter = 40;
%outliers=0.2;
T.R=eye(D1);
T.t=zeros(1,D1);
%W = computeW(X,Y,sigma2);
 R_=eye(D1);
R_(2,2)=cos(-90);
R_(1,2)=-sin(-90);
R_(2,1)=sin(-90);
R_(1,1)=cos(-90);
flag=1;
space=1;
figure,plot_iter(X,Y); title('Before registering');

    dW1=zeros(M,M);
    dW_1=zeros(N,N);
while(sigma2_con<sigma2)&&(iter<MaxIter)
    %update correspondence weight W[i,j]
    if space>20
        flag=1;
    end
    [W] = computeW(X,Y,sigma2,outliers);
    %[W] = computeIW(X_,Y,sigma2);
    %miniz
    %[R,t]=computeTrans(X_,Y,W);m
%     [R,t,s,sigma2]=computetransform(X,Y,W);
    %-----------------------------------------
    sumW=sum(W(:));
    sumWY=sum(W,2);
    sumWX=sum((W'),2);
    D=D1;
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

    for m=1:1:M
        dW1(m,m)=sumWY(m);
    end
    for n=1:1:N
        dW_1(n,n)=sumWX(n);
    end
    s=trace((A')*R)/trace((NY')*dW1*NY);
    sigma2=(trace((NX')*dW_1*NX)-s*trace((A')*R))/(sumW*D);
    
    %-----------------------------------------
    if (iter>30)&&(sigma2>0.01)&&(flag)
         R=R*R_;
         flag=0;
         space=1;
    end
    %transform points X
    T.R=T.R*R';
    T.t=(T.t)*R'+t';
    Y=Y*R'+repmat(t',[M,1]);
   %  Y=s*Y*R'+repmat(t',[M,1]);
    %update sigma2
   % sigma2=updatesigma2(X,Y,W);
%     fprintf('iter = %d,sigma2 = %.9f \n',iter,sigma2);
    iter = iter + 1;
    space=space+1;
    %close;
%     figure,plot_iter(X,Y);
end
    figure,plot_iter(X, Y);title('After registering');
end

