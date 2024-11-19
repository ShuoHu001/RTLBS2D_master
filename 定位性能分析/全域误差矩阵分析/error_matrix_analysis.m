
data = load("errormatrix.txt");
data(isnan(data))=0;
xmin = 15;
xmax = max(data(:,1));
ymin = 15;
ymax = max(data(:,2));
gap = data(2,1) - data(1,1);
x=xmin:gap:xmax;
y=ymin:gap:ymax;
[x,y]=meshgrid(x,y);
mean_dis=data(:,3);
x=x';
y=y';
mean_dis_matrix=reshape(mean_dis,size(x));
figure();
surf(x,y,mean_dis_matrix,'EdgeColor','none');
title("mean");
view(2)
