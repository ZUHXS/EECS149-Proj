load census;
% x = [-2:0.2:-1];
plusfun = @(x) max(x,0);
% left wall: y = x+2
x = (-0.5+1.5).*rand(40, 1) -1.5;
y = x + 2 + rand(40, 1) - 0.5;


% right wall: y = -x + 2
rx = (4-3).*rand(60, 1) +3;
ry = -rx + 4 + rand(60, 1) - 0.5;
x = cat(1, x, rx);
y = cat(1, y, ry);

rx = (2.5-1.5).*rand(15, 1) +1.5;
ry = -rx + 4 + rand(15, 1) - 0.5;
x = cat(1, x, rx);
y = cat(1, y, ry);

% noise
noise_x_lower = 1.90;
noise_x_higher = 2.20;
noise_y_lower = 0.10;
noise_y_higher = 0.35;
noise_x = (noise_x_higher-noise_x_lower).*rand(20,1) + noise_x_lower;
noise_y = (noise_y_higher-noise_y_lower).*rand(20,1) + noise_y_lower;
x = cat(1, x, noise_x)
y = cat(1, y, noise_y);

printx = cat(1, x, [-2]);
printy = cat(1, y, [-2]);
printx = cat(1, x, [5]);
printy = cat(1, y, [5]);
disp(x) % column vector
disp(y)
plot(printx, printy, 'o');


f=fit(x,y,'poly2')
hold on
plot(f)
hold off

% step 1 detect cluster of points
data = [x y]
% [idx,C] = kmeans(data,5);
% figure
% plot(data(idx==1,1),data(idx==1,2),'r.','MarkerSize',12)
% hold on
% plot(data(idx==2,1),data(idx==2,2),'b.','MarkerSize',12)
% plot(data(idx==3,1),data(idx==3,2),'g.','MarkerSize',12)
% plot(data(idx==4,1),data(idx==4,2),'g.','MarkerSize',12)
% plot(data(idx==5,1),data(idx==5,2),'g.','MarkerSize',12)
% plot(C(:,1),C(:,2),'kx',...
%      'MarkerSize',15,'LineWidth',3) 

figure
[idx,C,SUMD,K] = kmeans_opt(data)
disp(K)
plot(data(idx==1,1),data(idx==1,2),'r.','MarkerSize',12)
hold on
plot(data(idx==2,1),data(idx==2,2),'b.','MarkerSize',12)
plot(data(idx==3,1),data(idx==3,2),'g.','MarkerSize',12)
plot(data(idx==4,1),data(idx==4,2),'g.','MarkerSize',12)
plot(data(idx==5,1),data(idx==5,2),'g.','MarkerSize',12)
plot(C(:,1),C(:,2),'kx',...
     'MarkerSize',15,'LineWidth',3) 




function [IDX,C,SUMD,K]=kmeans_opt(X,varargin)


[m,~]=size(X); %getting the number of samples

if nargin>1, ToTest=cell2mat(varargin(1)); else, ToTest=ceil(sqrt(m)); end
if nargin>2, Cutoff=cell2mat(varargin(2)); else, Cutoff=0.95; end
if nargin>3, Repeats=cell2mat(varargin(3)); else, Repeats=3; end

D=zeros(ToTest,1); %initialize the results matrix
for c=1:ToTest %for each sample
    [~,~,dist]=kmeans(X,c,'emptyaction','drop'); %compute the sum of intra-cluster distances
    tmp=sum(dist); %best so far
    
    for cc=2:Repeats %repeat the algo
        [~,~,dist]=kmeans(X,c,'emptyaction','drop');
        tmp=min(sum(dist),tmp);
    end
    D(c,1)=tmp; %collect the best so far in the results vecor
end

Var=D(1:end-1)-D(2:end); %calculate %variance explained
PC=cumsum(Var)/(D(1)-D(end));

[r,~]=find(PC>Cutoff); %find the best index
K=1+r(1,1); %get the optimal number of clusters
[IDX,C,SUMD]=kmeans(X,K); %now rerun one last time with the optimal number of clusters

end