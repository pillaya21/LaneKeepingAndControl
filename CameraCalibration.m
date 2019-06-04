clear
clc
trial = 3;
folder = strcat('test',num2str(trial));
mkdir(folder);
Cam=webcam(1);
Cam.Resolution='1920x1080'
preview(Cam);
template='image_%d.jpeg';
for i=1:20
    name=sprintf(template,i)
    path = strcat(folder,'/',name)
%     name = path;
    fprintf('  press enter when next image ready, ctrl-c to exit');
    A=snapshot(Cam);
    input('');
    imwrite(A,path);
    end
    
