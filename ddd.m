%close all;
figure();
data=load('SDF.txt');
surf(data,'EdgeColor','none')
shading interp;