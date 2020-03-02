%A star algorithm


function [path,n_points]=a_star_3D(K,E3d_safe,x0_old,y0_old,z0_old,xend_old,yend_old,zend_old,sizeE)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Cost weights

kg=K(1);
kh=K(2);
ke=K(3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Algorithm


%If start and end points are not integer, they are rounded for the path genertion. Use of floor and ceil so they are different even if very close
x0=floor(x0_old);
y0=floor(y0_old);
z0=floor(z0_old);
xend=ceil(xend_old);
yend=ceil(yend_old);
zend=ceil(zend_old);


%Initialize

%Size of environment matrix
y_size=sizeE(1);
x_size=sizeE(2);
z_size=sizeE(3);


%Node from which the good neighbour is reached
came_fromx=zeros(sizeE);
came_fromy=zeros(sizeE);
came_fromz=zeros(sizeE);
came_fromx(y0,x0,z0)=x0;
came_fromy(y0,x0,z0)=y0;
came_fromz(y0,x0,z0)=z0;

%Nodes already evaluated
closed=[];

%Nodes to be evaluated
open=[y0,x0,z0];

%Cost of moving from start point to the node
G=Inf*ones(sizeE);
G(y0,x0,z0)=0;

%Initial total cost
F=Inf*ones(sizeE);
F(y0,x0,z0)=sqrt((xend-x0)^2+(yend-y0)^2+(zend-z0)^2);

%Initialize
exit_path=0;

%While open is not empy
while isempty(open)==0 && exit_path==0
    
    %Find node with minimum f in open
    
    %Initialize
    f_open=zeros(size(open,1),1);
    
    %Evaluate f for the nodes in open
    for i=1:size(open,1)
        f_open(i,:)=F(open(i,1),open(i,2),open(i,3));
    end
    
    %Find the index location in open for the node with minimum f
    [~,i_f_open_min]=min(f_open);
    
    %Location of node with minimum f in open
    ycurrent=open(i_f_open_min,1);
    xcurrent=open(i_f_open_min,2);
    zcurrent=open(i_f_open_min,3);
    
    point_current=[ycurrent, xcurrent, zcurrent];
    
    
    %Check if the arrival node is reached
    if xcurrent==xend && ycurrent==yend && zcurrent==zend
        
        %Arrival node reached, exit and generate path
        exit_path=1;
        
    else
        
        %Add the node to closed
        closed(size(closed,1)+1,:)=point_current;
        
        %Remove the node from open
        i_open_keep=horzcat(1:i_f_open_min-1,i_f_open_min+1:size(open,1));
        open=open(i_open_keep,:);
        
        %Check neighbour nodes
        for i=-1:1
            for j=-1:1
                for k=-1:1                    
                    
                    %If the neighbour node is within grid
                    if xcurrent+i>0 && ycurrent+j>0 && zcurrent+k>0 && xcurrent+i<=x_size && ycurrent+j<=y_size && zcurrent+k<=z_size
                        
                        %If the neighbour node does not belong to open and to closed

                        check_open=max(sum([ycurrent+j==open(:,1) xcurrent+i==open(:,2) zcurrent+k==open(:,3)],2));
                        check_closed=max(sum([ycurrent+j==closed(:,1) xcurrent+i==closed(:,2) zcurrent+k==closed(:,3)],2));
                        
                        if isempty(check_open)==1
                            check_open=0;
                        end
                        
                        if isempty(check_closed)==1
                            check_closed=0;
                        end

                        if check_open<3 && check_closed<3                            
                            
                            
                            %Add the neighbour node to open
                            open(size(open,1)+1,:)=[ycurrent+j, xcurrent+i, zcurrent+k];
                            
                            %Evaluate the distance from start to the current node + from the current node to the neighbour node
                            g_try=G(ycurrent,xcurrent,zcurrent)+sqrt(i^2+j^2+k^2);
                            
                            %If this distance is smaller than the neighbour distance
                            if g_try<G(ycurrent+j,xcurrent+i,zcurrent+k)
                                
                                %We are on the good path, save information
                                
                                %Record from which node the good neighbour is reached
                                came_fromy(ycurrent+j,xcurrent+i,zcurrent+k)=ycurrent;
                                came_fromx(ycurrent+j,xcurrent+i,zcurrent+k)=xcurrent;
                                came_fromz(ycurrent+j,xcurrent+i,zcurrent+k)=zcurrent;
                                
                                %Evaluate the cost function
                                G(ycurrent+j,xcurrent+i,zcurrent+k)=g_try;
                                H=sqrt((xend-(xcurrent+i))^2+(yend-(ycurrent+j))^2+(zend-(zcurrent+k))^2);
                                F(ycurrent+j,xcurrent+i,zcurrent+k)=kg*G(ycurrent+j,xcurrent+i,zcurrent+k)+kh*H+ke*E3d_safe(ycurrent+j,xcurrent+i,zcurrent+k);
                                                                
                            end
                        end
                    end                   
                end
            end
        end
    end
end


%Reconstruct path backwards knowing from which node the good neighbour is reached

%First element is the arrival point
path_backwards=[ycurrent,xcurrent,zcurrent];

%Initialize
i=2;

%While the start point is not reached
while xcurrent~=x0 || ycurrent~=y0 || zcurrent~=z0
    
    path_backwards(i,:)=[came_fromy(ycurrent,xcurrent,zcurrent) came_fromx(ycurrent,xcurrent,zcurrent) came_fromz(ycurrent,xcurrent,zcurrent)];
    ycurrent=path_backwards(i,1);
    xcurrent=path_backwards(i,2);    
    zcurrent=path_backwards(i,3); 
    i=i+1;
    
end

%Number of waypoints
n_points=size(path_backwards,1);


%Reverse path sequence
path=path_backwards(n_points+1-(1:n_points),:);


%Reassign the original start and end points
path(1,:)=[y0_old x0_old z0_old];
path(n_points,:)=[yend_old xend_old zend_old];
