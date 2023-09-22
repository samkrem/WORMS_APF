[x,y,z] = sphere(5);               %# Makes a 21-by-21 point sphere
%x = x(11:end,:);                %# Keep top 11 x points
%y = y(11:end,:);                %# Keep top 11 y points
%z = z(11:end,:);
r = 1;  
x = r*x(3:end,:);
y = r*y(3:end,:);
z = r*z(3:end,:);
disp(z(1,:));

                        %# A radius value
hs = surf(x,y,z);      %# Plot the surface
direction = [0 1 0];            % Specify Direction
%axis equal;                     %# Make the scaling on the x, y, and z axes equal
%Ax = get(gca);                  % Axes Handle
%XD = Ax.Children.XData;         % Get %XData’
%Ax.Children.XData = XD + 0.5;   % Add 0.5 To %XData’ To Shift It To All > 0