%% LISSAJOUS

%function [x_3turns, y_3turns, z_3turns]=gen_traiettoria(x_0,y_0,r)

%Traiettoria
samplerate=80;
x = [];
y = [];

%time=linspace(1,10,length(x_3turns))';       %tempo per la traiettoria [s]

for t=0:1/samplerate:10
    x = [x; 0.2*sin(t)+x_0];
    y = [y; 0.3*sin(2*t)+y_0];
end
z1 = linspace(d1,d1+d2,length(x)/2)';
z2 = linspace(d1+d2, d1, length(z1))';
z_2turns=[z1; z2; z2(end)];
x_2turns=x;     
y_2turns=y;  

% h=animatedline;
% 
% for k=1:length(x_2turns)
%     addpoints(h,x_2turns(k),y_2turns(k));
%     drawnow
% end