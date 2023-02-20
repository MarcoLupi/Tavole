%Test traiettoria

run serial_struttura2.m

x_0=d2+d4;       %centro circonferenza ellisse asse x
y_0=0;           %centro circonferenza ellisse asse y

%waypoints

%[x_3turns, y_3turns, z_3turns]=gen_traiettoria(x_0, y_0, r);
run lissajous_new.m

samplerate=100;
dt = 1/samplerate;
time=(0:dt:10)';
numSamples=length(time);

time_w = [0; linspace(1,10,length(x_2turns))'];

%waypoint iniziale

T_init=getTransform(robot,robot.homeConfiguration,'L6');

pos_init=T_init(1:3,4);

R_0=tform2rotm(T_init);

x_w_0=pos_init(1);
y_w_0=pos_init(2);
z_w_0=pos_init(3);

x_w= [x_w_0 x_2turns'];
y_w= [y_w_0 y_2turns'];
z_w= [z_w_0 z_2turns'];
waypoints=[x_w; y_w; z_w];

%%
%generazione traiettoria

R_f = R_0;

%[pos,vel, acc, ~] = bsplinepolytraj([x_w; y_w; z_w], [0 10], time);
[pos,vel, acc, ~] = quinticpolytraj([x_w; y_w; z_w], time_w, time);
[R, omega, ~] = rottraj(R_0, R_f, [0 10], time);
EUL= rotm2eul(R);                       %matrice di angoli di eulero in configurazione XYZ
x_ref = [pos;EUL'];
x_ref_ts = timeseries(x_ref, time);
x_ref_tt = timeseries2timetable(x_ref_ts);

x_dot_ref = [vel; omega];
x_dot_ref_ts = timeseries(x_dot_ref, time);
x_dot_ref_tt = timeseries2timetable(x_dot_ref_ts);

%%
%inversione cinematica
aik = analyticalInverseKinematics(robot);
generateIKFunction(aik,'robotIK');

q_ref = zeros(6,numSamples);
q_dot_ref= zeros(6,numSamples);
q_dot_dot_ref = zeros(6,numSamples);
angle= zeros(3, numSamples);
weights = [0.2 0.2 0.2 1 1 1];
no_config = [];

q_ref(:,1)=robot.homeConfiguration;
roll = zeros(1,numSamples);
yaw = zeros(1,numSamples);
pitch = zeros(1,numSamples);
e = zeros(6,numSamples);

for i=2:numSamples
    norm_old = 100;
    eePose=[R(:,:,i) pos(:,i); 0 0 0 1];
    ikConfig = robotIK(eePose);          % Uses the generated file

    for k = 1:size(ikConfig,1)
        diff = ikConfig(k,:)'-q_ref(:,i-1);
        norm_diff = norm(diff);
        if norm_diff < norm_old
            q_ref(:,i)=ikConfig(k,:)';
            norm_old = norm_diff;
        end
    end
%         poseNow = getTransform(robot,q_ref(:,i),'L6');
%         q1 = q_ref(1,i);
%         q2 = q_ref(2,i);
%         q3 = q_ref(3,i);
%         q4 = q_ref(4,i);
%         q5 = q_ref(5,i);
%         q6 = q_ref(6,i);    
%         J_a_now = eval(J_a);
%         angle(:,i) = tform2eul(poseNow)'; 
%     
%         e(:,i) = [pos(:,i); EUL(i,:)'] - [poseNow(1:3,4); angle(:,i)];
%         
%         q_dot_ref(:,i) = J_a_now'*([vel(:,i); omega(:,i)] + 5*e(:,i));

%         q_dot_dot_ref(:,i) = q_dot_ref(:,i)-q_dot_ref(:,i-1)/dt;
end

% Time Series
q_ref_ts = timeseries(q_ref, time);
q_dot_ref_ts = timeseries(q_dot_ref, time);
q_dot_dot_ref_ts = timeseries(q_dot_dot_ref, time);

q_ref_tt = timeseries2timetable(q_ref_ts);
q_dot_ref_tt = timeseries2timetable(q_dot_ref_ts);
q_dot_dot_ref_tt = timeseries2timetable(q_dot_dot_ref_ts);

%%
%figure
plot (time, e)
figure
set(gcf,'Visible','on')
show(robot);

%%
rc = rateControl(20);
figure
for i = 1:numSamples
    show(robot, q_ref(:,i),FastUpdate=true,PreservePlot=false);
    hold on
    poseNow = getTransform(robot,q_ref(:,i),'L6');
    taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',10);
    drawnow;

    waitfor(rc);
end
%
% plot(t_vet,pos)
% hold on
% plot(time,waypoints,'x')
% xlabel('t')
% ylabel('Positions')
% legend('X-positions','Y-positions','Z-positions')
% hold off

%% GRAFICO CLICK

q_real = out.q_clik;
%q_real = q_reali';
rc = rateControl(20);
figure
for i = 1:size(q_real,3)
    show(robot, q_real(:,1,i),FastUpdate=true,PreservePlot=false);
    hold on
    poseNow = getTransform(robot,q_real(:,1,i),'L6');
    taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',10);
    drawnow;

    waitfor(rc);
end

%% GRAFICO COMPUTED TORQUE

q_real = out.q_CT.Data;
%q_real = q_reali';
rc = rateControl(20);
figure
for i = 1:size(q_real,3)
    show(robot, q_real(:,1,i),FastUpdate=true,PreservePlot=false);
    hold on
    poseNow = getTransform(robot,q_real(:,1,i),'L6');
    taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',10);
    drawnow;

    waitfor(rc);
end

%% GRAFICO BACK STAPPING

q_real = out.q_BS.Data;
%q_real = q_reali';
rc = rateControl(20);
figure
for i = 1:size(q_real,3)
    show(robot, q_real(:,1,i),FastUpdate=true,PreservePlot=false);
    hold on
    poseNow = getTransform(robot,q_real(:,1,i),'L6');
    taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',10);
    drawnow;

    waitfor(rc);
end