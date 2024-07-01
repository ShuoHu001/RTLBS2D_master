function DrawBuildings(filename1)
height = 10;
%key=0;不加载绿化，key=1;加载绿化
figure("WindowState","maximized", "Color","white");
if (exist(filename1,'file')==2)
    fp=fopen(filename1,'r');
    j=1;
    f=1;
    while(~feof(fp))
        %获取数据
        a=fgets(fp);
        a=char(a);
        a=strsplit(a);
        a=str2double(char(a(3)));
        for i=1:a
           b(i,:)=fscanf(fp,'%f %f\n',2); 
        end
        %获取最小值
        fmax(f,:)=max(b);
        fmin(f,:)=min(b);
        f=f+1;
        %绘制图像
        %水平面图像
        patch(b(1:a,1),b(1:a,2),zeros(a,1),[0.7 0.7 0.7]);
        patch(b(1:a,1),b(1:a,2),height*ones(a,1),[0.7 0.7 0.7]);
        hold on;
        %绘制垂直面劈图像
        for i=1:a-1
            temp(1:2,1:2)=b(i:i+1,:);
            temp(3:4,1:2)=b(i+1:-1:i,:);
            temp(1:2,3)=zeros(2,1);
            temp(3:4,3)=height*ones(2,1);
            patch(temp(1:4,1),temp(1:4,2),temp(1:4,3),[0.7 0.7 0.7]);
        end
        j=j+1;
    end
    fclose(fp);
else
    opts = struct('WindowStyle','modal','Interpreter','tex');
    msgbox('\color{red}\fontname{楷体}\fontsize{16}建筑物文件不存在！'...
        ,'文件错误提示','error',opts);
end
axis equal;
grid on;
xlabel("x/m");
ylabel("y/m");
zlabel("z/m");
set(gca,"fontname","times","fontsize",26);
