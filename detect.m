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
% [idx,C,SUMD,K] = kmeans_opt(data)
[idx,C,SUMD,K]=best_kmeans(data)
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
%%% [IDX,C,SUMD,K]=kmeans_opt(X,varargin) returns the output of the k-means
%%% algorithm with the optimal number of clusters, as determined by the ELBOW
%%% method. this function treats NaNs as missing data, and ignores any rows of X that
%%% contain NaNs.
%%%
%%% [IDX]=kmeans_opt(X) returns the cluster membership for each datapoint in
%%% vector X.
%%%
%%% [IDX]=kmeans_opt(X,MAX) returns the cluster membership for each datapoint in
%%% vector X. The Elbow method will be tried from 1 to MAX number of
%%% clusters (default: square root of the number of samples)
%%% [IDX]=kmeans_opt(X,MAX,CUTOFF) returns the cluster membership for each datapoint in
%%% vector X. The Elbow method will be tried from 1 to MAX number of
%%% clusters and will choose the number which explains a fraction CUTOFF of
%%% the variance (default: 0.95)
%%% [IDX]=kmeans_opt(X,MAX,CUTOFF,REPEATS) returns the cluster membership for each datapoint in
%%% vector X. The Elbow method will be tried from 1 to MAX number of
%%% clusters and will choose the number which explains a fraction CUTOFF of
%%% the variance, taking the best of REPEATS runs of k-means (default: 3).
%%% [IDX,C]=kmeans_opt(X,varargin) returns in addition, the location of the
%%% centroids of each cluster.
%%% [IDX,C,SUMD]=kmeans_opt(X,varargin) returns in addition, the sum of
%%% point-to-cluster-centroid distances.
%%% [IDX,C,SUMD,K]=kmeans_opt(X,varargin) returns in addition, the number of
%%% clusters.

%%% sebastien.delandtsheer@uni.lu
%%% sebdelandtsheer@gmail.com
%%% Thomas.sauter@uni.lu


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


function [IDX,C,SUMD,K]=best_kmeans(X)
% [IDX,C,SUMD,K] = best_kmeans(X) partitions the points in the N-by-P data matrix X
% into K clusters. Rows of X correspond to points, columns correspond to variables. 
% IDX containing the cluster indices of each point.
% C is the K cluster centroids locations in the K-by-P matrix C.
% SUMD are sums of point-to-centroid distances in the 1-by-K vector.
% K is the number of cluster centriods determined using ELBOW method.
% ELBOW method: computing the destortions under different cluster number counting from
% 1 to n, and K is the cluster number corresponding 90% percentage of
% variance expained, which is the ratio of the between-group variance to
% the total variance. see <http://en.wikipedia.org/wiki/Determining_the_number_of_clusters_in_a_data_set>
% After find the best K clusters, IDX,C,SUMD are determined using kmeans
% function in matlab.
dim=size(X);
% default number of test to get minimun under differnent random centriods
test_num=10;
distortion=zeros(dim(1),1);
for k_temp=1:dim(1)
    [~,~,sumd]=kmeans(X,k_temp,'emptyaction','drop');
    destortion_temp=sum(sumd);
    % try differnet tests to find minimun disortion under k_temp clusters
    for test_count=2:test_num
        [~,~,sumd]=kmeans(X,k_temp,'emptyaction','drop');
        destortion_temp=min(destortion_temp,sum(sumd));
    end
    distortion(k_temp,1)=destortion_temp;
end
variance=distortion(1:end-1)-distortion(2:end);
distortion_percent=cumsum(variance)/(distortion(1)-distortion(end));
plot(distortion_percent,'b*--');
[r,~]=find(distortion_percent>0.9);
K=r(1,1)+1;
[IDX,C,SUMD]=kmeans(X,K);
end