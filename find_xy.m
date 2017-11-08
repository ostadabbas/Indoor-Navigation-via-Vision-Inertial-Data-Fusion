function p=find_xy(point,z,Vp)
web_cam=[848,0,424;0,848,232;0,0,1];
A=web_cam*Vp;
A=[A(1,:)-point(1)*A(3,:);A(2,:)-point(2)*A(3,:)];
b=-z*A(:,3);
A=A(:,1:2);
p=A^-1*b;
end