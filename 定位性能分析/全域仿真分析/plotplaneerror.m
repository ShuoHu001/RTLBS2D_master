open("building.fig");
hold on;
data=load("phiError_8.000000_powerError_0.000000.txt");
gap = data(2,2)-data(1,2);
xmin= min(data(:,2))-0.23*gap;
xmax = max(data(:,2))-0.23*gap;
ymin = min(data(:,3))-0.35*gap;
ymax = max(data(:,3))-0.35*gap;
x=xmin:gap:xmax;
y=ymin:gap:ymax;
[x,y]=meshgrid(x,y);
x=x';
y=y';
e=data(:,6);
e=reshape(e,size(x));
e=e';
validMatrix=data(:,1);
validMatrix=reshape(validMatrix,size(x));
validMatrix=validMatrix';
c=zeros([size(e),3]);

%% 建立颜色与定位精度值的映射关系
eMin=0;
eMax=10;
cdata=jet(20);                         %分为100个色阶
[xi,yi]=size(e);
eGap=(eMax-eMin)/20;                   %每个色阶间隔
for i=1:xi
    for j=1:yi
        if validMatrix(i,j)==1
            c_id=floor((e(i,j)-eMin)/eGap)+1;
            if(c_id>length(cdata))
                c_id=length(cdata);
            end
            c(i,j,:)=cdata(c_id,:);
        else
            c(i,j,:)=[1,1,1];
        end
    end
end
z=2*ones(size(x));
surf(x',y',z',c);
colormap("jet");
h=colorbar();
clim([eMin,eMax]);
set(get(h,'Title'),'string','Localization Error(m)','FontSize',20,'FontName','Times');%设置colorbar的上方文字说明
title("8.0° AoA Error & 0 dB Power Error");
set(gca,"FontSize",28,"FontWeight","bold");


