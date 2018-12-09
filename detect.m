realx1 = x;%x(1:1,:);
realy1 = y;%x(2,:);

data = [transpose(realx1) transpose(realy1)];
figure
plot(realx1, realy1, 'o');
epsilon = 0.5;
MinPts = 20;
[idx, isnoise]=DBSCAN(data,epsilon,MinPts);
disp(idx)
figure
PlotClusterinResult(data, idx)
title(['DBSCAN Clustering (\epsilon = ' num2str(epsilon) ', MinPts = ' num2str(MinPts) ')']);
hold on
% Step 3 extract individual cluster

leftwallx = [100];
leftwally = [100];
rightwallx = [100];
rightwally = [100];
leftwallb = -1;
rightwallb = -1;

maxidx = max(idx);
for i=1:maxidx
    fprintf('what');
    disp(i);
    datai = data(idx==i,:);
    dataix = datai(:,1,:); % both are column vector
    dataiy = datai(:,2,:);
    coefficients = polyfit(dataix, dataiy, 1);
    if coefficients(1) > 0.5
%         [n1,Center,n2,alldistance] = kmeans(datai, 1);
        Center = [mean(dataix), mean(dataiy)]
        s = size(dataiy);
        s = s(1);
        s = int32(s/200);
%         s2 = mean(alldistance);
        dataixafter = dataix;
        dataiyafter = dataiy;
        for i=0:s
            r = rand()/50;%*2*s2 - s2;
            dataixafter = [dataixafter;Center(1)+r];
            dataiyafter = [dataiyafter;Center(2)+r];
        end
        coefficients = polyfit(dataixafter, dataiyafter, 1);
        if abs(1- coefficients(1)) < 0.3
            % add to left wall
            leftwallx = [leftwallx;dataix]
            leftwally = [leftwally;dataiy]
            % draw the line to verify correctness
            bFit = mean(dataiy-dataix);
            plotx = [min(dataix):0.1:max(dataix)];
            ploty = bFit + plotx;
            plot(plotx, ploty, 'o');
            if bFit > leftwallb
                leftwallb = bFit
            end
%         else
%             PlotObject(datai);
        end
        
    elseif coefficients(1) < -0.5
%         [n1,Center,n2,alldistance] = kmeans(datai, 1);
        Center = [mean(dataix), mean(dataiy)]
        s = size(dataiy);
        s = s(1);
        s = int32(s/200);
%         s2 = mean(alldistance);
        dataixafter = dataix;
        dataiyafter = dataiy;
        for i=0:s
            r = rand()/50;%*2*s2 - s2;
            dataixafter = [dataixafter;Center(1)+r];
            dataiyafter = [dataiyafter;Center(2)-r];
        end
        coefficients = polyfit(dataixafter, dataiyafter, 1);
        if abs(-1- coefficients(1)) < 0.3
            rightwallx = [rightwallx;dataix]
            rightwally = [rightwally;dataiy]
            bFit = mean(dataiy+dataix);
            plotx = [min(dataix):0.1:max(dataix)];
            ploty = bFit - plotx;
            plot(plotx, ploty, 'o');
            if bFit > rightwallb
                rightwallb = bFit
            end
%             xFit = linspace(min(dataix), max(dataix), 1000);
%             yFit = polyval(coefficients , xFit);
%             disp(coefficients)
%             plot(xFit, yFit, 'b', 'LineWidth', 2);
%         else
%             PlotObject(datai);
        end
    else
        fprintf("skip single horizontal wall for now.");
%         PlotObject(datai);
    end
end

% figure
% hold on
if leftwallb > 0 && rightwallb > 0
    interceptx = (rightwallb - leftwallb) / 2;
    xplot = [min(realx1):0.01:interceptx];
    yplot = xplot+leftwallb;
    plot(xplot, yplot, '-');
    xplot = [interceptx:0.01:max(realx1)];
    yplot = -xplot+rightwallb;
    plot(xplot, yplot, '-');
elseif leftwallb > 0
    xplot = [min(realx1):0.01:max(realx1)];
    yplot = xplot+leftwallb
    plot(xplot, yplot, '-')
elseif rightwallb > 0
    xplot = [min(realx1):0.01:max(realx1)];
    yplot = -xplot+rightwallb
    plot(xplot, yplot, '-')
end

% figure
% hold on
% datai = data(idx==i,:);
%     dataix = datai(:,1,:); % both are column vector
%     dataiy = datai(:,2,:);
noise = data(isnoise==1,:);
[noiseidx, useless]=DBSCAN(noise,epsilon,8);
PlotClusterinResult(noise, noiseidx);
title(['DBSCAN Clustering (\epsilon = ' num2str(epsilon) ', MinPts = ' num2str(MinPts) ')']);
maxnoiseidx = max(noiseidx);
for i=1:maxnoiseidx
    noisei = noise(noiseidx==i,:);
    PlotObject(noisei);
end

function PlotObject(data)
    [n1,Center,n2,alldistance] = kmeans(data, 1);
    ptcount = size(alldistance, 1);
    radius = 4*n2 / ptcount;
    circlex = Center(1) - radius;
    circley = Center(2) - radius;
    circlewh = radius * 2;
    pos = [circlex circley circlewh circlewh]; 
    rectangle('Position',pos,'Curvature',[1 1], 'FaceColor', 'black', 'Edgecolor','none')
end


% draw the new fit line of all leftwall and rightwall points
% leftwallx=leftwallx(2:end,:);
% leftwally=leftwally(2:end,:);
% rightwallx=rightwallx(2:end,:);
% rightwally=rightwally(2:end,:);
% 
% if ~isempty(leftwallx)
%     coefficients = polyfit(leftwallx, leftwally, 1);
%     xFit = linspace(min(leftwallx), max(leftwallx), 1000);
%     yFit = polyval(coefficients , xFit);
%     plot(xFit, yFit, 'm', 'LineWidth', 2);
% end
% 
% if ~isempty(rightwallx)
%     coefficients = polyfit(rightwallx, rightwally, 1);
%     xFit = linspace(min(rightwallx), max(rightwallx), 1000);
%     yFit = polyval(coefficients , xFit);
%     plot(xFit, yFit, 'c', 'LineWidth', 2);
% end



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


% function [IDX,C,SUMD,K]=best_kmeans(X)
% % [IDX,C,SUMD,K] = best_kmeans(X) partitions the points in the N-by-P data matrix X
% % into K clusters. Rows of X correspond to points, columns correspond to variables. 
% % IDX containing the cluster indices of each point.
% % C is the K cluster centroids locations in the K-by-P matrix C.
% % SUMD are sums of point-to-centroid distances in the 1-by-K vector.
% % K is the number of cluster centriods determined using ELBOW method.
% % ELBOW method: computing the destortions under different cluster number counting from
% % 1 to n, and K is the cluster number corresponding 90% percentage of
% % variance expained, which is the ratio of the between-group variance to
% % the total variance. see <http://en.wikipedia.org/wiki/Determining_the_number_of_clusters_in_a_data_set>
% % After find the best K clusters, IDX,C,SUMD are determined using kmeans
% % function in matlab.
% dim=size(X);
% % default number of test to get minimun under differnent random centriods
% test_num=10;
% distortion=zeros(dim(1),1);
% for k_temp=1:dim(1)
%     [~,~,sumd]=kmeans(X,k_temp,'emptyaction','drop');
%     destortion_temp=sum(sumd);
%     % try differnet tests to find minimun disortion under k_temp clusters
%     for test_count=2:test_num
%         [~,~,sumd]=kmeans(X,k_temp,'emptyaction','drop');
%         destortion_temp=min(destortion_temp,sum(sumd));
%     end
%     distortion(k_temp,1)=destortion_temp;
% end
% variance=distortion(1:end-1)-distortion(2:end);
% distortion_percent=cumsum(variance)/(distortion(1)-distortion(end));
% plot(distortion_percent,'b*--');
% [r,~]=find(distortion_percent>0.9);
% K=r(1,1)+1;
% [IDX,C,SUMD]=kmeans(X,K);
% end


function [IDX, isnoise]=DBSCAN(X,epsilon,MinPts)
    C=0;
    
    n=size(X,1);
    IDX=zeros(n,1);
    
    D=pdist2(X,X);
    
    visited=false(n,1);
    isnoise=false(n,1);
    
    for i=1:n
        if ~visited(i)
            visited(i)=true;
            
            Neighbors=RegionQuery(i);
            if numel(Neighbors)<MinPts
                % X(i,:) is NOISE
                isnoise(i)=true;
            else
                C=C+1;
                ExpandCluster(i,Neighbors,C);
            end
            
        end
    
    end
    
    function ExpandCluster(i,Neighbors,C)
        IDX(i)=C;
        
        k = 1;
        while true
            j = Neighbors(k);
            
            if ~visited(j)
                visited(j)=true;
                Neighbors2=RegionQuery(j);
                if numel(Neighbors2)>=MinPts
                    Neighbors=[Neighbors Neighbors2];   %#ok
                end
            end
            if IDX(j)==0
                IDX(j)=C;
            end
            
            k = k + 1;
            if k > numel(Neighbors)
                break;
            end
        end
    end
    
    function Neighbors=RegionQuery(i)
        Neighbors=find(D(i,:)<=epsilon);
    end
end

function PlotClusterinResult(X, IDX)
    k=max(IDX);
    Colors=hsv(k);
    Legends = {};
    for i=0:k
        Xi=X(IDX==i,:);
        if i~=0
            Style = 'x';
            MarkerSize = 8;
            Color = Colors(i,:);
            Legends{end+1} = ['Cluster #' num2str(i)];
        else
            Style = 'o';
            MarkerSize = 6;
            Color = [0 0 0];
            if ~isempty(Xi)
                Legends{end+1} = 'Noise';
            end
        end
        if ~isempty(Xi)
            plot(Xi(:,1),Xi(:,2),Style,'MarkerSize',MarkerSize,'Color',Color);
        end
        hold on;
    end
    hold off;
    axis equal;
    grid on;
    legend(Legends);
    legend('Location', 'NorthEastOutside');
end
