clc;
clear all;

num_cluster = 5;
num_elements = 500;

[data cp idx]=generateData(1, 0.5, num_cluster, 12, 12, 5, 1, 2, num_elements);

minx = -min(data(:,1)) + 10;
miny = -min(data(:,2)) + 10;

data(:,1) = data(:,1) + minx;
data(:,2) = data(:,2) + miny;

scatter(data(:,1), data(:,2), 8, idx, 'LineWidth',3);
%set(gca,'fontsize',16)
%set(gca,'LineWidth',4)
set(gca,'xtick',[],'ytick',[]);
%axis off

random = randperm(num_elements, num_elements);

data2save = [idx-1 data];

datarand = [];

for i = 1 : num_elements
    datarand = [datarand; data2save(random(i), :)];
end

labels = datarand(:,1);
Data = datarand(:,2:end);

save '2Dclusters.dat' 'Data' '-ascii';
save '2Dclusterslabels.dat' 'labels' '-ascii';