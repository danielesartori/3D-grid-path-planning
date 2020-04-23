%3D grid map generator


function [E,E_safe,E3d,E3d_safe]=grid_3D_safe_zone(sizeE,d_grid,h,P0,Pend,n_low)


%Grid size
y_size=sizeE(1);
x_size=sizeE(2);
z_size=sizeE(3);

%Vertical grid vector
z_grid=1:d_grid:z_size;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Random grid generation, discrete height in big blocks

%Create random points with discrete 0-1 altitude
mean_E=0;
sigma=1;
k_sigma=2.2; %edit this to change obstacles density
E=random('Normal',mean_E,sigma,y_size,x_size);
sigma_obstacle=k_sigma*sigma;
E=double(E>sigma_obstacle);


%Assign random altitude to blocks around points 

%Minimum altitude
hh_min=3;

%Initialize temporary matrix for evaluation
EE=E;

for i=1:x_size
    for j=1:y_size
        
        %Block boundaries with random dimension (max -2:+2)
        k=i-1-round(random('beta',0.5,0.5)):1:i+1+round(random('beta',0.5,0.5));
        l=j-1-round(random('beta',0.5,0.5)):1:j+1+round(random('beta',0.5,0.5));        
        
        %If block boundaries within the grid and if the node point value is high,
        if min(k)>0 && min(l)>0 && max(k)<=x_size && max(l)<=y_size && EE(j,i)==1            
            
            %Assign random value to block
            hh=round(random('Normal',0.75*h,0.5*h));
            
            %Give a minimum value to the altitude and limit maximum altitude
            if hh<hh_min
                hh=hh_min;
            elseif hh>z_size
                hh=z_size;
            end
            
            E(l,k)=hh;  
                
        end
    end
end


%Assign low elevation to and around start and end points
E(P0(1)-n_low:P0(1)+n_low,P0(2)-n_low:P0(2)+n_low)=0;
E(Pend(1)-n_low:Pend(1)+n_low,Pend(2)-n_low:Pend(2)+n_low)=0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create 3D grid

%Initialize
E3d=zeros(y_size,x_size,z_size);

%Create 3D grid matrix occupancy index (0=free, 1=obstacle)
for i=1:z_size  
	E3d(:,:,i)=E>=z_grid(i);       
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create safe zone near high elevation elements

%Initialize
E_safe=E;

for i=1:x_size
    for j=1:y_size
        
        %Check neighbour nodes
        for k=-1:1
            for l=-1:1
                
                %If neighbours are within the grid 
                if (i+k)>0 && (j+l)>0 && (i+k)<=x_size && (j+l)<=y_size 
                
                    %If among all the neighbours there is one high element and we are not in a high node or safe node                
                    if E(j+l,i+k)>0 && E(j,i)==0
                     
                        %Assign the maximum value of the neighbour nodes
                        E_safe(j,i)=E(j+l,i+k);
                        
                    end                    
                end
            end
        end
        
        %If point is elevated add one safe step in altitude
        if E_safe(j,i)>0 && E_safe(j,i)<z_size-1
            E_safe(j,i)=E_safe(j,i)+1;            
        end        
        
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create 3D safe grid matrix occupancy index (0=free, 0.5=safe, 1=obstacle)

%Initialize
E3d_safe=E3d;

for i=1:x_size
    for j=1:y_size
        for k=1:z_size
            
            %Check neighbour nodes
            l=i-1:i+1;
            m=j-1:j+1;
            n=k-1:k+1;
            
            %Limit neighbours within the grid
            if min(l)<1
                l=i:i+1;
            elseif  max(l)>x_size
                l=i-1:i;
            end
            
            if min(m)<1
                m=j:j+1;
            elseif  max(m)>y_size
                m=j-1:j;
            end
            
            if min(n)<1
                n=k:k+1;
            elseif  max(n)>z_size
                n=k-1:k;
            end
            
            
            %Evaluation matrix
            E_eval=E3d(m,l,n);            
            
            %If we are in a free point and nearby there is obstacle
            if E3d(j,i,k)==0 && max(E_eval(:))==1
               E3d_safe(j,i,k)=0.5;
            end

        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create safe zone near borders

E([1 end],:)=z_size;
E(:,[1 end])=z_size;

E_safe([1 end],:)=z_size;
E_safe(:,[1 end])=z_size;

E3d([1 end],:,:)=1;
E3d(:,[1 end],:)=1;
E3d(:,:,[1 end])=1;

E3d_safe([1 end],:,:)=1;
E3d_safe(:,[1 end],:)=1;
E3d_safe(:,:,[1 end])=1;

