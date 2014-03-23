clear;
load face;
[N,D]=size(X);
t=[1,1,1];
X=X+repmat(t,[N,1]);
% add a random rigid transformation
Y=X;
OX=X;
OY=Y;
%R_=cpd_R(rand(1),rand(1),rand(1));
%X=X*(R_)'+1;
 figure,plot_iter(Y, Y); title('init');
 sumdis=0;
 [N,D]=size(X);
 [M,D]=size(Y);
 t=[10.0,0.0,10.0];
 R_=eye(D);
R_(2,2)=0.7071;
R_(2,3)=-0.7071;
R_(3,2)=0.7071;
R_(3,3)=0.7071;
OX=OX*(R_)';
X=OX+repmat(t,[N,1]);
figure,plot_iter(X,Y); title('Before registering');
 for i=1:1:N
    for j=1:1:M
        sumdis=sumdis+(X(i,:)-Y(j,:))*(X(i,:)-Y(j,:))';
    end
 end
 sigma2=sumdis/(N*M*D);
 fprintf('before iter sigma2 = %.5f \n',sigma2);
 disp(t);
[T,R,t,X_,Y]=kl_register(X,Y,sigma2);
 disp(R);
 disp(t);
