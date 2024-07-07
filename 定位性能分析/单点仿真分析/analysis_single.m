phiErrors=[0.1,0.2,0.5,1.0,2.0,3.0,4.0,5.0,6.0];
powerErrors=[0,1,2,3,4,5,6,7,8,9,10];
station_name="B";
phiNum=length(phiErrors);
powerNum=length(powerErrors);
error_matrix = zeros(phiNum,powerNum);
mark={"o","+","*",".","x","square","diamond","^", "v","<",">"};
color=turbo(9);
figure("WindowState","maximized", "Color","white");
h=zeros(phiNum,1);
for i=1:length(phiErrors)
    for j=1:length(powerErrors)
        filename=strcat(station_name,"_phiError_",num2str(phiErrors(i)),"_powerError_",num2str(powerErrors(j)),".txt");
        data=load(filename);
        error_matrix(i,j)=mean(data(:,6));
    end
    p=plot(0:10,error_matrix(i,:));
    % p.Color=color(i,:);
    p.LineStyle="--";
    p.Marker=mark(i);
    p.LineWidth=3.0;
    p.MarkerSize=20.0;
    h(i)=p;
    hold on;
end
grid on;
xlabel("Power Error(PE)/dB");
ylabel("RMSE/m");
legend(h,{'0.1° AOA Error','0.2°AOA Error','0.5°AOA Error','1.0°AOA Error','2.0°AOA Error','3.0°AOA Error','4.0°AOA Error','5.0°AOA Error','6.0°AOA Error'});
set(gca,"fontname","times","fontsize",28,"fontweight","bold");