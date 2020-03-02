%Main

clc
clear


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Grid and path parameters

%Grid size [y,x,z] and resolution
sizeE=[80 80 20];
d_grid=1;

%Starting point
x0=5;
y0=7;
z0=12;

%Arrival point
xend=68;
yend=66;
zend=6;

%Number of points with low elevation around start and end point area 
n_low=3;

%A* or Theta* cost weights
kg=1;
kh=1.25;
ke=sqrt((xend-x0)^2+(yend-y0)^2+(zend-z0)^2);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Map definition

%Average flight altitude
h=max(z0,zend);

%Points coordinates in [y,x,z] format
P0=[y0 x0 z0];
Pend=[yend xend zend];

%Generate map
[E,E_safe,E3d,E3d_safe]=grid_3D_safe_zone(sizeE,d_grid,h,P0,Pend,n_low);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Path generation

%Store gains in vector
K=[kg kh ke];

%Measure path computation time
tic

%Generate path (chose one)
% [path,n_points]=a_star_3D(K,E3d_safe,x0,y0,z0,xend,yend,zend,sizeE);
% [path,n_points]=theta_star_3D(K,E3d_safe,x0,y0,z0,xend,yend,zend,sizeE);
[path,n_points]=lazy_theta_star_3D(K,E3d_safe,x0,y0,z0,xend,yend,zend,sizeE);

T_path=toc;


%Path waypoints partial distance

%Initialize
path_distance=zeros(n_points,1);

for i=2:n_points 
	path_distance(i)=path_distance(i-1)+sqrt((path(i,2)-path(i-1,2))^2+(path(i,1)-path(i-1,1))^2+(path(i,3)-path(i-1,3))^2);      
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot

%Map grid
x_grid=1:d_grid:sizeE(2);
y_grid=1:d_grid:sizeE(1);

%Path on safe map
figure(1)
surf(x_grid(2:end-1),y_grid(2:end-1),E_safe(2:end-1,2:end-1))
hold on
plot3(x0,y0,z0,'gs')
plot3(xend,yend,zend,'rd')
plot3(path(:,2),path(:,1),path(:,3),'yx')
plot3(path(:,2),path(:,1),path(:,3),'w')
axis tight
axis equal
view(0,90);
colorbar

%Path on standard map
figure(2)
surf(x_grid(2:end-1),y_grid(2:end-1),E(2:end-1,2:end-1))
hold on
plot3(x0,y0,z0,'gs')
plot3(xend,yend,zend,'rd')
plot3(path(:,2),path(:,1),path(:,3),'yx')
plot3(path(:,2),path(:,1),path(:,3),'w')
axis tight
axis equal
view(0,90);
colorbar

%Path height profile
figure(3)
plot(path_distance,path(:,3))
grid on
xlabel('Path distance')
ylabel('Path height')
