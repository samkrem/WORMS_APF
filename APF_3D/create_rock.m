% Function for creating cylinders 
% The follwing function is refered from the Dr Lyuba ALBOUL lecture notes.

function [hs,  direction] = create_rock(pos,r,sides, color)
%side=6;
[X,Y,Z] = sphere(sides);
if(mod(sides,2)==0)
    disp("true")
    topsides=(sides/2)+1;
    X = r*X(topsides:end,:);
    Y = r*Y(topsides:end,:);
    Z = r*Z(topsides:end,:);
    disp(Z);
else
    
    topsides=floor(sides/2)+1; %this makes object on ground
    X = r*X(topsides:end,:);
    Y = r*Y(topsides:end,:);
    Z = r*Z(topsides:end,:);
   % fprintf('Z= %i\n', Z)
end
Xpos = pos(1);
Ypos = pos(2);

X = X + Xpos;
Y = Y + Ypos;
Z=Z - Z(1); %-z(1) makes z height makes rocks base at z=0
%fprintf('%s , %d \n',X,Y);

hs = surf(X,Y,Z, 'facecolor',color,'LineStyle','--', 'EdgeColor',color); %# Plot the surface
for i =1:topsides
   fill3(X(i,:),Y(i,:),Z(i,:),color); 
end
for i=1:sides   
    fill3(X(:,i),Y(:,i),Z(:,i),color); 

end

%floorSide side-1
%h2 = fill3(X(1,:),Y(1,:),Z(1,:),color); %makes lines
%h3 = fill3(X(2,:),Y(2,:),Z(2,:),color); %makes lines
direction = [0 1 0];            % Specify Direction
axis equal;                     %# Make the scaling on the x, y, and z axes equal

end

