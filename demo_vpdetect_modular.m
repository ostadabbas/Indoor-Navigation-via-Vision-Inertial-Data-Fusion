clc
clear all
close all

%% Read Entire Video %%%%
mov = VideoReader('sample_video\aclab_video_TF.mp4');  %WIN_20170508_13_45_27_Pro
i=0;
while hasFrame(mov)
i=i+1;
vid = rgb2gray(readFrame(mov));
video(:,:,i)=vid;
end

%% Synchonize IMU and Video
video=video(:,:,1:size(video,3)/339:end); %obtain this number from csv file

%% Read IMU data %%%
showfig = 0;
l_start = 1;
l_end = 339; %size(video,3); obtain this number from csv file
frq=30;
filename='sample_video\aclab_data_TF.csv';
[data_IMU,data_lables,t] = iPhone_IMU_reading(filename,frq,l_start,l_end,showfig);
%%%Acceleration & Gravity data
x_g = data_IMU(:,5)*9.81; % X gravity * g
y_g = data_IMU(:,6)*9.81; % Y gravity * g
z_g = data_IMU(:,7)*9.81; % Z gravity * g
acc_x=data_IMU(:,2)*9.81; % X acceleration * g
acc_y=data_IMU(:,3)*9.81; % Y acceleration * g
acc_z=data_IMU(:,4)*9.81; % Z acceleration * g

%%%The 3 attitudes reported by iPhone
AttPitch = data_IMU(:,12);   %rotation over x axis
AttYaw = data_IMU(:,13);     %rotation over z axis
AttYaw = ThetaCorrect(AttYaw);
AttRoll = data_IMU(:,14);    %rotation over y axis

%%%Angle computed by integration of gyro measurement
x_gyro = data_IMU(:,15);
y_gyro = data_IMU(:,16);
z_gyro = data_IMU(:,17);
delta_t = [t(1) ; t(2:end) - t(1:end-1)];
pitch_gyro = cumsum(x_gyro.*delta_t,1); 
roll_gyro = cumsum(y_gyro.*delta_t,1); 
yaw_gyro = cumsum(z_gyro.*delta_t,1); 

%%%Angle computed by geometry of gravity
pitch_g = atan2(y_g , z_g) + pi;
pitch_g = ThetaCorrect(pitch_g); 
yaw_g = atan2((- x_g) , sqrt(y_g .* y_g + z_g .* z_g));
yaw_g = ThetaCorrect(yaw_g);

%% 2D-map initialization
poss_x=0;
poss_y=0;
str_len=0.5;
cntx=0;cnty=0;
wall2_ly=[];wall2_ry=[];wall2_leftx=[];wall2_rightx=[];
w_hall2=[];w_hall3=[];
cnn=1;
showfig_path=1;

%% Kalman filter initialization
flag_fusion=0;
P_Pitch = [1 0; 0 1];
P_Roll = [1 0; 0 1];
P_Yaw = [1 0; 0 1];
q_bias_Pitch=0;
q_bias_Roll=0;
q_bias_Yaw=0;

%% Apply GMM Method On Each Frame

g_filt=4;
fig_flag=0;
figure();
nVp=3;
map=jet(nVp);

for j=1:size(video,3)
img=video(:,:,j);
if j==1
    Vp0=eye(3);alpha0=0;beta0=0;gamma0=0;
else
    Vp0=Vp{1,j-1};
end
[X1,W1,I1,nVp,Vp1,PN1,alpha0,beta0,gamma0]=func_vpdetect4(img,g_filt,alpha0,beta0,gamma0);
X{j,:}=X1;W{1,j}=W1;I{1,j}=I1;Vp{1,j}=Vp1;PN{1,j}=PN1;
j
%% Line Grouping %%
if fig_flag==1
imshow(I{1,j},[]);
hold on
for i=1:numel(X{j,1})
    [p_max,idx]=max(W{1,j}(:,i));
    if p_max>0.33
    plot(X{j,1}{1,i}(2,:),X{j,1}{1,i}(1,:),'Linewidth',2,'color',map(idx,:));
    text(mean(X{j,1}{1,i}(2,:)),mean(X{j,1}{1,i}(1,:)),num2str(i),'color','r')
    end
end
end
[~,idx]=sort(W{1,j}','descend');

%% Find ALpha, Beta and Gamma for each frame from Vanishing Directions 
for k=1:nVp
    
    idx=find(W{1,j}(k,:)>0.33); %probabilities greater than 0.95
    XX=[X{j,1}{1,idx}];
    XX=[XX;ones(1,2*length(idx))];

    web_cam=[848,0,0;0,848,0;0,0,1];
    temp=web_cam*Vp{1,j}(:,k)/Vp{1,j}(3,k);
    Vp_img{1,j}(:,k)=temp(1:2,1)+size(vid)'/2;
    
    if fig_flag==1
    plot(Vp_img{1,j}(2,k),Vp_img{1,j}(1,k),'*','color',map(k,:))
    
    %plot([XX(2,1),Vp_img{1,j}(2,k)],[XX(1,1),Vp_img{1,j}(1,k)],'--','color',map(k,:))
    %plot([XX(2,3),Vp_img{1,j}(2,k)],[XX(1,3),Vp_img{1,j}(1,k)],'--','color',map(k,:))
    end
end

web_cam=[848,0,0;0,848,0;0,0,1];
Vp_norm=web_cam^-1*[Vp_img{1,j}(:,3)-size(vid)'/2;1];
Vp_norm=Vp_norm/norm(Vp_norm);

Vp_norm=web_cam^-1*[Vp_img{1,j}(:,2)-size(vid)'/2;1];
Vp_norm=Vp_norm/norm(Vp_norm);

beta1{1,j}=acos(Vp_norm(2))*180/pi;

if j>1
    if abs(beta1{1,j}-beta1{1,j-1})>80 & abs(beta1{1,j}-beta1{1,j-1})<260 %%%check here
        beta1{1,j}=beta1{1,j}-180*sign(beta1{1,j}-beta1{1,j-1});
    end
end

Vp_norm=web_cam^-1*[Vp_img{1,j}(:,1)-size(vid)'/2;1];
Vp_norm=Vp_norm/norm(Vp_norm);

alpha2{1,j}=atan(Vp_norm(2)/Vp_norm(1))*180/pi;
beta2{1,j}=asin(-Vp_norm(3))*180/pi;

Vp_norm=web_cam^-1*[Vp_img{1,j}(:,2)-size(vid)'/2;1];
Vp_norm=Vp_norm/norm(Vp_norm);

gamma2{1,j}=real(asin(Vp_norm(3)/cos(beta2{1,j}*pi/180))*180/pi);

if fig_flag==1
j
%pause(0.000005)
%xlim([min(0,min(Vp_img{1,j}(2,:))),max(size(vid,2),max(Vp_img{1,j}(2,:)))]);
%ylim([min(0,min(Vp_img{1,j}(1,:))),max(size(vid,1),max(Vp_img{1,j}(1,:)))]);
pause
hold off
end
%%%%
%%%smooth beta by 10 and plot yaw angle%%%
if j>=10
bet(j)=median([beta1{1,j-9:j}]);
else
bet(j)=median([beta1{1,1:j}]);
end
if fig_flag==1
    U=cos(bet(j)*pi/180);
    V=sin(bet(j)*pi/180);
    plt=compass(U,V,'-r');
    set(plt,'LineWidth',3);
    pause(0.000005)
end
%%%%%%%%%%%


%% Horizon line Detection
if j>=10
alp(j)=median([alpha2{1,j-9:j}])*pi/180;
bett(j)=median([beta2{1,j-9:j}])*pi/180;
gam(j)=median([gamma2{1,j-9:j}])*pi/180;
else
alp(j)=median([alpha2{1,1:j}])*pi/180;
bett(j)=median([beta2{1,1:j}])*pi/180;
gam(j)=median([gamma2{1,1:j}])*pi/180;
end

fVp1=@(alpha,beta) [cos(alpha)*cos(beta);...
        sin(alpha)*cos(beta);...
        -sin(beta)];
fVp2=@(alpha,beta,gamma) [cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma);...
        sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma);...
        cos(beta)*sin(gamma)];
fVp3=@(alpha,beta,gamma) [cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);...
        sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);...
        cos(beta)*cos(gamma)];

%%%%%%%


%% Kalman filter fusion

%%%Angle computed from video or acc
roll_video=bet;
AttRoll_video=-roll_video'*pi/180;

AttPitch_measured = pitch_g;
Attpitch_video=pi/2--alp';
Attyaw_video=-gam';
AttYaw_measured = yaw_g;

if j>1 & flag_fusion==1
for id = 1 : 3 %three id for fusion of pitch, roll and yaw
    if id == 1 
        %fusion of pitch from gyro and acc
        gyro = x_gyro; %model parameter
        angle_measurement = AttPitch_measured; %measurement
        angle_vid=Attpitch_video;
        P=P_Pitch;angle=alp(j-1);q_bias=q_bias_Pitch;
        title_ang = 'Pitch';
    end
    if id == 2
        %fusion of roll from gyro and video 
        gyro = y_gyro; %model parameter
        angle_measurement = AttRoll_video; %measurement
        angle_vid=AttRoll_video;
        P=P_Roll;angle=bet(j-1);q_bias=q_bias_Roll;
        title_ang = 'Roll';
    end
    if id == 3
        %fusion of yaw from gyro and acc
        gyro = z_gyro; %model parameter
        angle_measurement = AttYaw_measured; %measurement
        angle_vid=Attyaw_video;
        P=P_Yaw;angle=gam(j-1);q_bias=q_bias_Yaw;
        title_ang = 'Yaw';
    end
    %%%
    [angle, q_bias, P] = Kalman_Angle(delta_t(j),gyro(j),angle_measurement(j),P,angle,q_bias);
    %%%
    if id==1
        P_Pitch=P;alp(j)=angle;pitch_g(j)=angle;q_bias_Pitch=q_bias;
    end
    if id==2
        P_Roll=P;bet(j)=angle;bett(j)=angle;q_bias_Roll=q_bias;
    end
    if id==3
        P_Yaw=P;gam(j)=angle;yaw_g(j)=angle;q_bias_Yaw=q_bias;
    end
    
end
end

%% step counting & finding step locations
if j==1
mag = sqrt(sum((x_g+acc_x).^2 + (y_g+acc_y).^2 + (z_g+acc_z).^2, 2));
magNoG = mag - mean(mag);
minPeakHeight = std(magNoG)*0.1;
[pks, locs] = findpeaks(magNoG, 'MINPEAKHEIGHT', minPeakHeight);
numSteps = numel(pks);
end

%% averaging attitude between steps
AttYaw1=AttRoll_video;
if j==locs(cnn)
    if cnn==1
        orien(cnn)=mean(AttYaw1(1:locs(cnn)));
    else
        orien(cnn)=mean(AttYaw1(locs(cnn-1):locs(cnn))); 
    end
end

%% plane detection
flag=1;  
%figure()
map=jet(nVp);
%fig=figure('Visible','off');
%set(fig,'WindowStyle','docked');
subplot(1,2,1) 
imshow(I{1,j},[]);
hold on
sVp=[fVp1(alp(j),bett(j)),...
fVp2(alp(j),bett(j),gam(j)),...
fVp3(alp(j),bett(j),gam(j))];

%%fixing color coding for depth estimation
cmpr=eye(nVp);idx=[];
for k=1:nVp
    [~,idx(k)]=max(abs(sVp'*cmpr(:,k)));
end
sVp=sVp(:,idx);
W_temp{1,j}=W{1,j}(idx,:);
xy=idx(2);
clear idx
%%%%%
for k=1:nVp
    web_cam=[848,0,0;0,848,0;0,0,1];
    temp=web_cam*sVp(:,k)/sVp(3,k);
    Vp_img{1,j}(:,k)=temp(1:2,1)+size(vid)'/2;
end

[~,idx]=max(W_temp{1,j});
dist_left=[];
idx_left=[];
dist_right=[];
idx_right=[];
for i=1:numel(X{j,1})
    
    if idx(i)==3 & min(X{j,1}{1,i}(1,:))>Vp_img{1,j}(1,3) & max(X{j,1}{1,i}(2,:))<Vp_img{1,j}(2,3)%depth lines below & left of horizon line
        slope=atan((X{j,1}{1,i}(1,1)-X{j,1}{1,i}(1,2))/(X{j,1}{1,i}(2,1)-X{j,1}{1,i}(2,2)))*180/pi;
        if abs(slope)<70 & abs(slope)>20 & sign(slope)==-1
            [~,L_idx]=min(X{j,1}{1,i}(1,:));
            idx_left=[idx_left,[i;L_idx]];
            dist_left=[dist_left,norm(X{j,1}{1,i}(:,L_idx)-Vp_img{1,j}(:,3))*cos(slope*pi/180)];
        end
    end
        
    if idx(i)==3 & min(X{j,1}{1,i}(1,:))>Vp_img{1,j}(1,3) & min(X{j,1}{1,i}(2,:))>Vp_img{1,j}(2,3)%depth lines below & left of horizon line
        slope=atan((X{j,1}{1,i}(1,1)-X{j,1}{1,i}(1,2))/(X{j,1}{1,i}(2,1)-X{j,1}{1,i}(2,2)))*180/pi;
        if abs(slope)<70 & abs(slope)>20 & sign(slope)==1
            [~,L_idx]=min(X{j,1}{1,i}(1,:));
            idx_right=[idx_right,[i;L_idx]];
            dist_right=[dist_right,norm(X{j,1}{1,i}(:,L_idx)-Vp_img{1,j}(:,3))*cos(slope*pi/180)];
        end
    end
end

if numel(idx_left)~=0
gl_idx=idx_left(:,dist_left==min(dist_left));
gl=X{j,1}{1,gl_idx(1)}(:,gl_idx(2));
else
gl=[size(vid,1);0];
end
if numel(idx_right)~=0
gr_idx=idx_right(:,dist_right==min(dist_right));
gr=X{j,1}{1,gr_idx(1)}(:,gr_idx(2));
else
gr=[size(vid,1);size(vid,2)];
end
if gl(1)<=gr(1)
    gp=gl;
else
    gp=gr;
end

idx_gnd=[];
dist_gnd=[];
for i=1:numel(X{j,1}) 
    if idx(i)==2 & min(X{j,1}{1,i}(1,:))>Vp_img{1,j}(1,3) & min(X{j,1}{1,i}(1,:))<=gp(1)+5 ...
            & max(X{j,1}{1,i}(2,:))>gl(2) & min(X{j,1}{1,i}(2,:))<gr(2) %ground lines below horizon line
        idx_gnd=[idx_gnd,i];
        dist_gnd=[dist_gnd,abs(mean(X{j,1}{1,i}(1,:))-gp(1))];
    end
end
if numel(idx_gnd)~=0
g_idx=idx_gnd(dist_gnd==min(dist_gnd));
gnd=X{j,1}{1,g_idx};
else
    gnd=gp;
end
if numel(idx_left)~=0 & numel(idx_right)~=0
dl=X{j,1}{1,gl_idx(1)};
dr=X{j,1}{1,gr_idx(1)};
elseif numel(idx_left)~=0
dl=X{j,1}{1,gl_idx(1)};
[~,v_idx]=max(gnd(2,:));
dr=[Vp_img{1,j}(:,3),gnd(:,v_idx)];
elseif numel(idx_right)~=0
dr=X{j,1}{1,gr_idx(1)};
[~,v_idx]=min(gnd(2,:));
dl=[Vp_img{1,j}(:,3),gnd(:,v_idx)];
else
[~,v_idx]=min(gnd(2,:));
dl=[Vp_img{1,j}(:,3),gnd(:,v_idx)];
[~,v_idx]=max(gnd(2,:));
dr=[Vp_img{1,j}(:,3),gnd(:,v_idx)];
end
   
%plot

slope_l=(dl(2)-dl(4))/(dl(1)-dl(3));
slope_r=(dr(2)-dr(4))/(dr(1)-dr(3));

point_l=slope_l*mean(gnd(1,:))+(dl(1)*dl(4)-dl(2)*dl(3))/(dl(1)-dl(3));
point_r=slope_r*mean(gnd(1,:))+(dr(1)*dr(4)-dr(2)*dr(3))/(dr(1)-dr(3));
point_ld=slope_l*size(vid,1)+(dl(1)*dl(4)-dl(2)*dl(3))/(dl(1)-dl(3));
point_rd=slope_r*size(vid,1)+(dr(1)*dr(4)-dr(2)*dr(3))/(dr(1)-dr(3));

if flag==1
plot([point_l,point_r],[mean(gnd(1,:)),mean(gnd(1,:))]...
    ,'-','color',map(2,:),'LineWidth',3);

plot([point_l,point_ld],[mean(gnd(1,:)),size(vid,1)]...
    ,'-','color',map(3,:),'LineWidth',3);
plot([point_r,point_rd],[mean(gnd(1,:)),size(vid,1)]...
    ,'-','color',map(3,:),'LineWidth',3);

plot([point_l,point_l],[mean(gnd(1,:)),0]...
    ,'-','color',map(1,:),'LineWidth',3);
plot([point_r,point_r],[mean(gnd(1,:)),0]...
    ,'-','color',map(1,:),'LineWidth',3);
end
%

%%%
%% update horizon line if possible
if numel(idx_left)~=0 & numel(idx_right)~=0
     hz_x=mean(gnd(1,:))-abs((point_l-point_r)/(slope_l-slope_r));
     hz_y=point_l-abs((point_l-point_r)/(slope_l-slope_r))*slope_l;
 else
    hz_x=Vp_img{1,j}(1,3);
    hz_y=Vp_img{1,j}(2,3);
end
if flag==1
plot(1:size(vid,2),repmat(hz_x,1,size(vid,2))...
    ,'--','color',map(k,:),'LineWidth',3)
end
%%%
%% depth estimation
h=1.4; %camera height
f=size(vid,1); %camera focal point
b=bett(j); %camera pitch angle
if numel(idx_left)~=0 & numel(idx_right)~=0
dy=abs((point_l-point_r)/(slope_l-slope_r));
else
dy=mean(gnd(1,:))-Vp_img{1,j}(1,3); %ground to horizon distance in pixel
end
%depth(j)=(tan(b)+f/(dy*cos(b)^2))*h; %or simply f*h/dy
depth(j)=f*h/dy;
%%%
%% width & height estimation
pl=find_xy([mean(gnd(1,:));point_l],depth(j),sVp);
pr=find_xy([mean(gnd(1,:));point_r],depth(j),sVp);
width(j)=norm(pl-pr);

point_u=[0,point_l];
pu=find_xy(point_u,depth(j),sVp);
height(j)=norm(pl-pu);
%%%
%j
if flag==1 %& numel(idx_left)~=0 & numel(idx_right)~=0
title(['depth=' num2str(sprintf('%.1f',depth(j))) ', width=' num2str(sprintf('%.1f',width(j))) ', h=' num2str(sprintf('%.1f',height(j)))]);
end
%pause(0.000005)
%xlim([min(0,min(Vp_img{1,j}(2,:))),max(size(vid,2),max(Vp_img{1,j}(2,:)))]);
%ylim([min(0,min(Vp_img{1,j}(1,:))),max(size(vid,1),max(Vp_img{1,j}(1,:)))]);
%pause
hold off
     
%% 2D-map generation
if j>=100 %smooth hallway width by 100
width_hallway(j)=median(width(j-99:j));
else
width_hallway(j)=median(width(1:j));
end
%%%%

if j==locs(cnn)
    poss_y=[poss_y,poss_y(end)+str_len*cos(orien(cnn))]; %roll_video
    poss_x=[poss_x,poss_x(end)+str_len*sin(orien(cnn))];
    
    if xy==2
        cnty=cnty+1;
        cntx=0;w_hall3=[];
        w_hall2=[w_hall2,width_hallway(j)];
        w_hall=median(w_hall2); 
        wall2_ly=[wall2_ly,poss_y(end)];
        wall2_ry=[wall2_ry,poss_y(end)];
        wall2_leftx=[wall2_leftx,poss_x(end-cnty)-width_hallway(j)/2];
        wall2_rightx=[wall2_rightx,poss_x(end-cnty)+width_hallway(j)/2];
        wall2_leftx(end-cnty+1:end)=poss_x(end-cnty)-w_hall/2;
        wall2_rightx(end-cnty+1:end)=poss_x(end-cnty)+w_hall/2;
        if poss_x(end)<wall2_leftx(end)
            poss_x(end)=wall2_leftx(end)+w_hall/2;%+width_hallway(j)/2;
        end
        if poss_x(end)>wall2_rightx(end)
            poss_x(end)=wall2_rightx(end)-w_hall/2;%-width_hallway(j)/2;
        end
    elseif xy==3
        cntx=cntx+1;
        cnty=0;w_hall2=[];
        w_hall3=[w_hall3,width_hallway(j)];
        w_hall=median(w_hall3); 
        wall2_leftx=[wall2_leftx,poss_x(end)];
        wall2_rightx=[wall2_rightx,poss_x(end)];
        wall2_ly=[wall2_ly,poss_y(end-cntx)-width_hallway(j)/2];
        wall2_ry=[wall2_ry,poss_y(end-cntx)+width_hallway(j)/2];
        wall2_ly(end-cntx+1:end)=poss_y(end-cntx)-w_hall/2;
        wall2_ry(end-cntx+1:end)=poss_y(end-cntx)+w_hall/2;
        if poss_y(end)<wall2_ly(end)
            poss_y(end)=wall2_ly(end)+w_hall/2;%+width_hallway(j)/2;
        end
        if poss_y(end)>wall2_ry(end)
            poss_y(end)=wall2_ry(end)-w_hall/2;%-width_hallway(j)/2;
        end
    end
    cnn=cnn+1;
end
if showfig_path==1
    subplot(1,2,2)
plot(poss_x,poss_y,'LineWidth',1,'color','black')
%xlim([-20,10]);
%ylim([-15,15]);
hold on
text(poss_x(1),poss_y(1),'*start','FontSize',14);
text(poss_x(end),poss_y(end),'*You','color','red','FontSize',10);
if xy==2 | xy==3
    plot(wall2_leftx,wall2_ly,'LineWidth',1,'color','blue');
    plot(wall2_rightx,wall2_ry,'LineWidth',1,'color','blue');
end
hold off
%saveas(gcf,['C:\Users\Amir\Documents\MATLAB\pic\VP_Path\img' num2str(j) '.jpg']);
end
%%%%%
if showfig_path==0
     subplot(1,2,2)
     U=cos(bet(j)*pi/180);
     V=sin(bet(j)*pi/180);
     plt=compass(U,V,'-r');
     set(plt,'LineWidth',3);
     %saveas(gcf,['C:\Users\Amir\Documents\MATLAB\pic\VP_final\img' num2str(j) '.jpg']);
     %%M(i)=getframe;
end
%j
%pause
pause(0.00000005)

end


%% Kalman filter function
function [angle, q_bias, P] = Kalman_Angle(delta_t,gyro,angle_measurement,P,angle,q_bias)

R_angle = 0.3; % R represents the measurement covariance noise. 
%In this case, it is a 1x1 matrix that says that we expect 0.3rad = 17.2 degree jitter from the measurement

Q_angle = 0.1;%0.001;
Q_bias = 0.01;%0.003;
Q = [Q_angle 0; 0 Q_bias]; % Q is a 2x2 matrix that represents the process covariance noise.
%In this case, it indicates how much we trust the measurement relative to the gyros.

X = [angle; q_bias];

%%%Update equations at each time point
A = [1 -delta_t; 0 1]; % x = Ax + Bu; Our state vector is: x= [angle gyro_bias]
angle = angle + (gyro - q_bias)*delta_t; % update x1 = x1 + (gyro - q_bias)*dt and x2 = x2;
C = [1 0]; % Output matrix angle = C.x
angle_err = angle_measurement - angle; %This is called the innovation 
% Calculate the difference between the second value and the value predicted by our model. 

E = C*P*C' + R_angle; % Calculate covariance 
K = A*P*C'*inv(E); % Calculate Kalman gain
X = X + K * angle_err; % Correct the prediction of state
P = A*P*A' - K*C*P*A' + Q; %Calculate the covariance of the prediction error
angle = X(1);
q_bias = X(2);
end

%% Theta correct function
function thetaC = ThetaCorrect(theta)
    theta = [0; theta(:)];
    tDiff = theta(2:end) - theta(1:end-1);
    tDiff(tDiff > pi) = tDiff(tDiff > pi) - 2*pi;
    tDiff(tDiff < -pi) = tDiff(tDiff < -pi) + 2*pi;
    thetaC = cumsum(tDiff);
end

%% iPhone IMU reading function
function [data_IMU,data_lables,t] = iPhone_IMU_reading(filename,frq,l_start,l_end,showfig)
R = 1; %start from second column to avoid the test
C = 0; %srat from first row
data = csvread(filename,R,C);
data_IMU = data(l_start:l_end,:);
data_lables = {'TimeStamp'	'AccX'	'AccY'	'AccZ'	'GravX'	'GravY'	'GravZ'	'MagAccuracy'	'MagX'	'MagY'	'MagZ'	'AttPitch'	'AttYaw' 'AttRoll'	'rotX'	'rotY'	'rotZ'};
t = data_IMU(:,1) - data_IMU(1,1);

%check for time jump
if showfig
    figure()
    plot(t(2:end) - t(1:end-1)),title('Time Jump'),xlabel('time(s)'),ylabel('time sample difference(s)'), grid on
end

%ploting accelometer and attitude data
if showfig
    figure()
    No = 6; %number of signals to be plotted
    for id = 1:3
        subplot(No,1,id),plot(t,data_IMU(:,id+1)),xlabel('time(s)'),ylabel('m/s2'),title(data_lables{id+1}), grid on
    end
    for idx = 15:17
        subplot(No,1,idx-11),plot(t,data_IMU(:,idx)*180/pi),xlabel('time(s)'),ylabel('degree/s'),title(data_lables{idx}), grid on
    end
end
end