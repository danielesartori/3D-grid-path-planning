%Check if there is free line of sight between two cells on a 3D map.
%Returns 1 if there is line of sight, 0.5 if
%partial line of sight, 0 otherwise

%From:
%Daniel, Nash - Theta star, any-angle path planning on grids
%Modification:
%Inverted x and y to account for our grid coordinate system
%Check if the evaluation of E remains within the grid limits
%Extension to 3D case

%Limitation:
%Allows a straight line to pass between diagonally touching blocked cells


function sight=line_sight_partial_3D(E3d_safe,xb_bound,yb_bound,zb_bound,sizeE)


%Size of environment matrix
y_size=sizeE(1);
x_size=sizeE(2);
z_size=sizeE(3);

%Rename
x1_0=xb_bound(1);
x2=xb_bound(2);
y1_0=yb_bound(1);
y2=yb_bound(2);
z1_0=zb_bound(1);
z2=zb_bound(2);


%Distance
dy=y2-y1_0;
dx=x2-x1_0;
dz=z2-z1_0;

if dy<0
    dy=-dy;
    sy=-1;
else
    sy=1;
end

if dx<0
    dx=-dx;
    sx=-1;
else
    sx=1;
end

%Angle between height and horizontal trace
gamma=atan2(dz,sqrt(dx^2+dy^2));

%Initialize
x1=x1_0;
y1=y1_0;
sight=1;

f=0;
if dy>=dx
    while y1~=y2
        f=f+dx;
        if f>=dy && 0<y1+(sy-1)/2 && y1+(sy-1)/2<=y_size && 0<x1+(sx-1)/2 && x1+(sx-1)/2<=x_size
            
            z=floor(z1_0+tan(gamma)*sqrt((x1+(sx-1)/2-x1_0)^2+(y1+(sy-1)/2-y1_0)^2));
            if z<=0
                z=1;
            elseif z>z_size
                z=z_size;
            end
            
            if E3d_safe(y1+(sy-1)/2,x1+(sx-1)/2,z)>0
                if E3d_safe(y1+(sy-1)/2,x1+(sx-1)/2,z)==1
                    sight=0;
                    return
                else
                    sight=0.5;
                end
            end
            x1=x1+sx;
            f=f-dy;
        end
        
        if 0<y1+(sy-1)/2 && y1+(sy-1)/2<=y_size && 0<x1+(sx-1)/2 && x1+(sx-1)/2<=x_size
            
            z=floor(z1_0+tan(gamma)*sqrt((x1+(sx-1)/2-x1_0)^2+(y1+(sy-1)/2-y1_0)^2));
            if z<=0
                z=1;
            elseif z>z_size
                z=z_size;
            end
            
            if f~=0 && E3d_safe(y1+(sy-1)/2,x1+(sx-1)/2,z)>0
                if E3d_safe(y1+(sy-1)/2,x1+(sx-1)/2,z)==1
                    sight=0;
                    return
                else
                    sight=0.5;
                end
            end
        end
        
        if 0<y1+(sy-1)/2 && y1+(sy-1)/2<=y_size && 1<x1 && x1<=x_size
            
            z_1=floor(z1_0+tan(gamma)*sqrt((x1-x1_0)^2+(y1+(sy-1)/2-y1_0)^2));
            if z_1<=0
                z_1=1;
            elseif z_1>z_size
                z_1=z_size;
            end
            
            z_2=floor(z1_0+tan(gamma)*sqrt((x1-1-x1_0)^2+(y1+(sy-1)/2-y1_0)^2));
            if z_2<=0
                z_2=1;
            elseif z_2>z_size
                z_2=z_size;
            end
            
            if dx==0 && E3d_safe(y1+(sy-1)/2,x1,z_1)>0 && E3d_safe(y1+(sy-1)/2,x1-1,z_2)>0 
                if E3d_safe(y1+(sy-1)/2,x1,z_1)==1 && E3d_safe(y1+(sy-1)/2,x1-1,z_2)==1
                    sight=0;
                    return
                else
                    sight=0.5;
                end
            end
        end
        
        y1=y1+sy;
    end
else
    while x1~=x2
        f=f+dy;
        if f>=dx && 0<y1+(sy-1)/2 && y1+(sy-1)/2<=y_size && 0<x1+(sx-1)/2 && x1+(sx-1)/2<=x_size
            
            z=floor(z1_0+tan(gamma)*sqrt((x1+(sx-1)/2-x1_0)^2+(y1+(sy-1)/2-y1_0)^2));
            if z<=0
                z=1;
            elseif z>z_size
                z=z_size;
            end
            
            
            if E3d_safe(y1+(sy-1)/2,x1+(sx-1)/2,z)>0
                if E3d_safe(y1+(sy-1)/2,x1+(sx-1)/2,z)==1
                    sight=0;
                    return
                else
                    sight=0.5;
                end
            end
            y1=y1+sy;
            f=f-dx;
        end
        if 0<y1+(sy-1)/2 && y1+(sy-1)/2<=y_size && 0<x1+(sx-1)/2 && x1+(sx-1)/2<=x_size
            
            z=floor(z1_0+tan(gamma)*sqrt((x1+(sx-1)/2-x1_0)^2+(y1+(sy-1)/2-y1_0)^2));
            if z<=0
                z=1;
            elseif z>z_size
                z=z_size;
            end
            
            if f~=0 && E3d_safe(y1+(sy-1)/2,x1+(sx-1)/2,z)>0
                if E3d_safe(y1+(sy-1)/2,x1+(sx-1)/2,z)==1
                    sight=0;
                    return
                else
                    sight=0.5;
                end
            end
        end
        if 1<y1 && y1<=y_size && 0<x1+(sx-1)/2 && x1+(sx-1)/2<=x_size
            
            z_1=floor(z1_0+tan(gamma)*sqrt((x1+(sx-1)/2-x1_0)^2+(y1-y1_0)^2));
            if z_1<=0
                z_1=1;
            elseif z_1>z_size
                z_1=z_size;
            end
            
            z_2=floor(z1_0+tan(gamma)*sqrt((x1+(sx-1)/2-x1_0)^2+(y1-1-y1_0)^2));
            if z_2<=0
                z_2=1;
            elseif z_2>z_size
                z_2=z_size;
            end
            
            if dy==0 && E3d_safe(y1,x1+(sx-1)/2,z_1)>0 && E3d_safe(y1-1,x1+(sx-1)/2,z_2)>0
                if E3d_safe(y1,x1+(sx-1)/2,z_1)==1 && E3d_safe(y1-1,x1+(sx-1)/2,z_2)==1
                    sight=0;
                    return
                else
                    sight=0.5;
                end
            end
        end
        x1=x1+sx;
    end
end
