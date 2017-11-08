% clc
% clear all
% close all
function [X,W,I,nVp,Vp,PN,alpha,beta,gamma]=func_vpdetect4(img,g_filt,alpha,beta,gamma)

%img=imread('C:\Users\Amir\Downloads\RGB-d\vpdetection-master\vpdetection-master\data\indoor_10.jpg');
%load('C:\Users\Amir\Downloads\RGB-d\vpdetection-master\vpdetection-master\data\d1');
%img=img(10:480-9,10:640-9,:);
%d=d(10:480-9,10:640-9);
%I=rgb2gray(img);
I=img;
I_smth=imgaussfilt(I,g_filt);
%I_smth=histeq(I_smth,5);
%B=edge(I_smth,'Canny');

% I_smth=imgaussfilt(I,8);
% [Gmag, Gdir] = imgradient(I_smth,'prewitt'); %[Gmag, Gdir] = imgradient(B,'prewitt');
% 
% Gdir(Gdir<0)=180+Gdir(Gdir<0);
% 
% Gdir_B=-ones(size(I));
% Gdir_B(B==1)=Gdir(B==1);
% 
% nbin=8;
% edges=0:180/nbin:180;
% [~,~,bin] = histcounts(Gdir_B,edges);
% 
% CnCm={};
% for i=1:nbin
%     temp=zeros(size(I));
%     temp(bin==i)=1;
%     %temp=imfill(temp); %comment
%     CC=bwconncomp(temp);
%     idx=cellfun('length',CC.PixelIdxList);
%     CnCm=[CnCm,{CC.PixelIdxList{1,idx>=round(0.05*min(size(I)))}}];
% end


[m,n]=size(I);
A=[1/m,0,-1/2;0,1/m,-n/(2*m);0 0 1];
%A=eye(3);
X={};

% [H,T,R] = hough(B);
% P = houghpeaks(H,30,'threshold',ceil(0.1*max(H(:))));
% 
% lines = houghlines(B,T,R,P);%,'FillGap',5,'MinLength',7);

%lines = line_detector(I);
lines = get_straight_line_segments(double(I_smth),round(0.05*sqrt(m^2+n^2))); %0.025
%points3d = rgb_plane2rgb_world(d);

% for k = 1:size(lines,2)
%     x1=flip(lines(k).point1');
%     x2=flip(lines(k).point2');
% %   x1=lines(1:2,k);
%  %  x2=lines(3:4,k);
%    X=[X,[x1,x2]];
%    pn=cross(A*[x1;1],A*[x2;1]);
%    PN(:,k)=pn/norm(pn);
% end

%d=(d-min(min(d)))/max(max(d));
for k = 1:size(lines,1)
    x1=round(flip(lines(k,[1,3])'));
    x2=round(flip(lines(k,[2,4])'));
%    x1=round(lines(1:2,k));
%    x2=round(lines(3:4,k));
   X=[X,[x1,x2]];
   pn=cross(A*[x1;1],A*[x2;1]);
   %pn=points3d(sub2ind([m,n],x1(1),x1(2)),:)'-points3d(sub2ind([m,n],x2(1),x2(2)),:)';
   PN(:,k)=pn/norm(pn);
end

% for i=1:numel(CnCm)
%     
%     temp=CnCm{1,i};
%     [x,y]=ind2sub(size(I),temp);
%     x_mean=mean(x);
%     y_mean=mean(y);
%     L=sqrt((max(x)-min(x))^2+(max(y)-min(y))^2);
%     D=[sum((x-x_mean).^2),sum((x-x_mean).*(y-y_mean));sum((x-x_mean).*(y-y_mean)),sum((y-y_mean).^2)];
%     [V,~]=eigs(D,1);
%     theta=atan2(V(2),V(1));
%     
%     x1=[x_mean;y_mean]-L/2*[cos(theta);sin(theta)];
%     x2=[x_mean;y_mean]+L/2*[cos(theta);sin(theta)];
%     
%     X=[X,[x1,x2]];
%     pn=cross(A*[x1;1],A*[x2;1]);
%     PN(:,i)=pn/norm(pn);
%     
% end

%Expectation Maximization

%Initialization
nVp=3;
zigma=[0.1;0.1;0.1];
p=[1/3;1/3;1/3];

niter=20;

fVp1=@(alpha,beta) [cos(alpha)*cos(beta);...
        sin(alpha)*cos(beta);...
        -sin(beta)];
fVp2=@(alpha,beta,gamma) [cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma);...
        sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma);...
        cos(beta)*sin(gamma)];
fVp3=@(alpha,beta,gamma) [cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);...
        sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);...
        cos(beta)*cos(gamma)];
%%%%%%derivatives%%%%%%%%
dVp1_a=@(alpha,beta) [-sin(alpha)*cos(beta);...
        cos(alpha)*cos(beta);...
        0];
dVp1_b=@(alpha,beta) [-cos(alpha)*sin(beta);...
        -sin(alpha)*sin(beta);...
        -cos(beta)];
%%%
dVp2_a=@(alpha,beta,gamma) [-sin(alpha)*sin(beta)*sin(gamma)-cos(alpha)*cos(gamma);...
        cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma);...
        0];
dVp2_b=@(alpha,beta,gamma) [cos(alpha)*cos(beta)*sin(gamma);...
        sin(alpha)*sin(beta)*sin(gamma);...
        -sin(beta)*sin(gamma)];
dVp2_g=@(alpha,beta,gamma) [cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);...
        sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);...
        cos(beta)*cos(gamma)];    
%%%
dVp3_a=@(alpha,beta,gamma) [-sin(alpha)*sin(beta)*cos(gamma)+cos(alpha)*sin(gamma);...
        cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);...
        0];
dVp3_b=@(alpha,beta,gamma) [cos(alpha)*cos(beta)*cos(gamma);...
        sin(alpha)*cos(beta)*cos(gamma);...
        -sin(beta)*cos(gamma)];
dVp3_g=@(alpha,beta,gamma) [-cos(alpha)*sin(beta)*sin(gamma)+sin(alpha)*cos(gamma);...
        -sin(alpha)*sin(beta)*sin(gamma)-cos(alpha)*cos(gamma);...
        -cos(beta)*sin(gamma)];
%%%%%%%%%

%%%% Residual %%%%
res=@(Vp1,Vp2,Vp3,A1,A2,A3) Vp1'*A1*Vp1+Vp2'*A2*Vp2+Vp3'*A3*Vp3; 
%%%%%%%%%%%%%%

% alpha=0;
% beta=0;
% gamma=0;
s=[0.001;0.0001;0.01]; %gradient descent step s=[0.001;0.0001;0.01];
%note:by setting s, we supress gradient directions in alpha and beta
%directions (gamma direction is more probable)

Vp=[fVp1(alpha,beta),...
    fVp2(alpha,beta,gamma),...
    fVp3(alpha,beta,gamma)];

for i=1:niter
    
    %f=zeros(nVp,numel(CnCm));
    f=zeros(nVp,length(lines));
    for k=1:nVp
        f(k,:)=p(k)*normpdf(PN'*Vp(:,k),0,zigma(k));
    end
    %calculate membership probability
    W=f./sum(f);
    W(isnan(W))=0;
    %update prior probability
    %p=mean(W,2);
    %update vanishing point
    
    A1=PN*diag(W(1,:))*PN';
    A2=PN*diag(W(2,:))*PN';
    A3=PN*diag(W(3,:))*PN';
    
    for j=1:10
           
    Vp1=Vp(:,1);Vp2=Vp(:,2);Vp3=Vp(:,3);
    err=res(Vp1,Vp2,Vp3,A1,A2,A3);
    
    grad_a=2*(dVp1_a(alpha,beta)'*A1*Vp1+...
        dVp2_a(alpha,beta,gamma)'*A2*Vp2+...
        dVp3_a(alpha,beta,gamma)'*A3*Vp3);
    grad_b=2*(dVp1_b(alpha,beta)'*A1*Vp1+...
        dVp2_b(alpha,beta,gamma)'*A2*Vp2+...
        dVp3_b(alpha,beta,gamma)'*A3*Vp3);
    grad_g=2*(dVp2_g(alpha,beta,gamma)'*A2*Vp2+...
        dVp3_g(alpha,beta,gamma)'*A3*Vp3);
    grad=[grad_a;grad_b;grad_g];
    
    angle=[alpha;beta;gamma];
    angle=angle-s.*grad;
    alpha=angle(1);beta=angle(2);gamma=angle(3);
    
    Vp=[fVp1(alpha,beta),...
    fVp2(alpha,beta,gamma),...
    fVp3(alpha,beta,gamma)];
    
    end
    
    %update zigma
    %zigma=sqrt(sum(W.*(Vp'*PN).^2,2)./sum(W,2));
    
end

 %nVp=4;
 %W = SpectralClustering(abs(corr(double(PN))), nVp, 1)';
 %W=gpca_pda_spectralcluster(PN,nVp); %rgpca_mvt %gpca_pda_spectralcluster

% imshow(I);
% map=jet(nVp);
% hold on
% for i=1:numel(X)
%     [~,idx]=max(W(:,i));
%     mem(i)=idx;
%     %idx=W(i);
%     if idx==-1, idx=nVp; end
%     plot(X{1,i}(2,:),X{1,i}(1,:),'Linewidth',2,'color',map(idx,:));
% end