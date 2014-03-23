%add random gaussain noise

load 2Dfishnoise4;
[M,D]=size(X);

%  R_=eye(D);
% R_(2,2)=cos(-175);
% R_(1,2)=-sin(-175);
% R_(2,1)=sin(-175);
% R_(1,1)=cos(-75);
% t=[1,2];
% Y=X*(R_)'+repmat(t,[M,1]);

K=0.3*M;
noise1=zeros(K,D);
noise2=zeros(K,D);
mean1=sum(X)/M;
mean2=sum(Y)/M;
for i=1:1:K
    noise1(i,:)=normrnd(mean1,0.7);
    noise2(i,:)=normrnd(mean2,0.7);
end

X=[X;noise1];
Y=[Y;noise2];
save('noisedata\2Dfishnoise7','X','Y');
% save('noisedata\2Dfish','X','Y');
figure1 = figure('Color',[1 1 1]);
plot_iter(X,Y);title('Before registering');
axis off;
clear;