%   CPD_PLOT(X, Y, C); plots 2 data sets. Works only for 2D and 3D data sets.
%
%   Input
%   ------------------ 
%   X           Reference point set matrix NxD;
%   Y           Current postions of GMM centroids;
%   C           (optional) The correspondence vector, such that Y corresponds to X(C,:) 
%
%   See also CPD_REGISTER.


function plot_iter(X, Y, C)

[m, d]=size(Y);

if d>3, error('cpd_plot.m error! Supported dimension for visualizations are only 2D and 3D.'); end;
if d<2, error('cpd_plot.m error! Supported dimension for visualizations are only 2D and 3D.'); end;

% for 2D case
if d==2,
   plot(X(:,1), X(:,2),'r*', Y(:,1), Y(:,2),'bo'); %axis off; axis([-1.5 2 -1.5 2]);
else
% for 3D case
   plot3(X(:,1),X(:,2),X(:,3),'r.',Y(:,1),Y(:,2),Y(:,3),'b*'); % title('X data (red). Y GMM centroids (blue)');set(gca,'CameraPosition',[15 -50 8]);
end
% 
% % plot correspondences
%     hold on;
%     if d==2,
%         for i=1:m,
%             plot([X(C(i),1) Y(i,1)],[X(C(i),2) Y(i,2)]);
%         end
%     else
%         for i=1:m,
%             plot3([X(C(i),1) Y(i,1)],[X(C(i),2) Y(i,2)],[X(C(i),3) Y(i,3)]);
%         end
%     end
%     hold off;
disp('display over');
drawnow;