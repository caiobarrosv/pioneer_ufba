function DrawRobot(Xr,col);

p=0.02; % percentage of axes size 
a=axis;
size = 0.3;

P=[-1 1 0 -1; -1 -1 3 -1];%basic triangle
theta = Xr(3)-pi/2;%rotate to point along x axis (theta = 0) 
c=cos(theta);
s=sin(theta);
P=[c -s; s c]*P; %rotate by theta
P(1,:)=P(1,:)*size +Xr(1); %scale and shift to x
P(2,:)=P(2,:)*size +Xr(2);
hold on
H = plot(P(1,:),P(2,:),col,'LineWidth',0.05, 'HandleVisibility', 'off');% draw 
hold on
plot(Xr(1),Xr(2),sprintf('%s+',col), 'HandleVisibility', 'off');

end

