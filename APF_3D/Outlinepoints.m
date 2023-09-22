
curr_foot = [10,10,6,14,6 ; 10,14,14,10,10 ; 0,8,8,8,8];

curr_foot = curr_foot';

% Compute the surface boundary of the point cloud
TR = delaunayTriangulation(curr_foot);
[B, ~] = freeBoundary(TR);

% Plot the boundary using plot3
figure;
plot3(curr_foot(:,1), curr_foot(:,2), curr_foot(:,3), 'k.');
hold on;
plot3(curr_foot(B(:,1),1), curr_foot(B(:,1),2), curr_foot(B(:,1),3), 'r-', 'LineWidth', 2);
axis equal;
