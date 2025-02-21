% open("环境底图.fig");
figure;
fig=gcf;
fig.Position=[100,100,1100,800];
hold on;
txposition=[45 21 2];
scatter3(45,21,2,800,'red','pentagram','filled','MarkerEdgeColor','k');
text(47,23,2,'R','FontSize',32,'FontWeight','bold','FontName','times',Color='Red');

method='TOA';
timeErrors=[1,2,5,10,15,20];
id=6;

if strcmp(method,'TOA')
    filename=[method,'_',num2str(timeErrors(id)),'.000000_errormatrix.txt'];
    titlename=['MSD Distribution ',' ','@',num2str(timeErrors(id)),'ns TOA Range'];
    figname=strcat('MSD_Distribution_',method,'_',num2str(timeErrors(id)),".fig");
else
    filename=[method,'_2.000000_',num2str(timeErrors(id)),'.000000_errormatrix.txt'];
    titlename=['MSD Distribution ','@2° AOA and','@',num2str(timeErrors(id)),'ns TOA Range'];
    figname=strcat('MSD_Distribution_',method,'_2_',num2str(timeErrors(id)),".fig");
end

data = load(filename);
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
e=data(:,3);
e=reshape(e,size(x));
e=e';
c=zeros([size(e),3]);
%% 建立颜色与定位精度值的映射关系
eMin=0;
eMax=50;
cdata=jet(20);                         %分为100个色阶
[xi,yi]=size(e);
eGap=(eMax-eMin)/20;                   %每个色阶间隔
for i=1:xi
    for j=1:yi
        c_id=floor((e(i,j)-eMin)/eGap)+1;
            if(c_id>length(cdata))
                c_id=length(cdata);
            end
            c(i,j,:)=cdata(c_id,:);
    end
end
z=2*ones(size(x));
surf(x',y',z',c,'EdgeColor','none');
colormap("jet");
h=colorbar();
clim([eMin,eMax]);
set(get(h,'Title'),'string','MSD(m)','FontSize',20,'FontName','Times');%设置colorbar的上方文字说明
set(gca,"FontSize",28,"FontWeight","bold");
title(titlename,"FontSize",26,"FontWeight","bold");

view(2)
% saveas(gcf,figname);
% close;
