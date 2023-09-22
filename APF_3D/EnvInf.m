%Environment code
clf;
close all;
clear;
% Defining Goal position of the UAV.
goal = [185,120,0];
goals=[10; 10; 0];

%(10, 10, 0), (10,14,4), (6,14,4) , (14,10,4), (6, 10, 4)
start_foot = [10,10,6,14,6 ; 10,14,14,10,10 ; 0,8,8,8,8];
goal_foot = [185, 185, 181, 189, 181 ;120, 124, 124, 120, 120;0, 8, 8, 8, 8];

%start_foot = [10,10,6,14,6 ; 10,14,14,10,10 ; 0,0,0,0,0];
%goal_foot = [185, 185, 181, 189, 181 ;120, 124, 124, 120, 120;0, 0, 0, 0, 0];
numpoints=size(start_foot,2);
disp(numpoints)





% Defining intial position of the UAV.


numObst=5;
% Buildings Position for Cylinder
%Cpos = [70,50,60; 20,60,40; 60,90,60; 140,40,50; 180,190,60; 30,180,60;100,20,30; 30,110,20; 150,100,35; 70,160,40; 110,140,20];
%Buildings Position for Rock
rng('shuffle');
Cpos = randi([0,200],numObst,2);
zcoords=zeros(numObst,1);
Cpos=[Cpos zcoords];
%disp(Cpos);
figure; hold on
view(3);
x = 0:4:200;
y = 0:4:200;
xlabel("x");
ylabel("y");
zlabel("z");
xlim([0 200]);
ylim([0 200]);
zlim([0 100]);
rng("shuffle")
%rock
radius = randi([8 15], numObst, 1); % Radius of the buildings
fprintf('Radius is %d\n', radius);
%cylinder
%radius=[10;15;13;15;11;15;16;14;19;17;15];

rng("shuffle")
nSides=randi([4 6],numObst,1);
for i =1:numObst
    %create_cylinder(radius(i,1), Cpos(i,:), [0.25, 0.58, 0.96])
    create_rock(Cpos(i,:),radius(i,1),nSides(i,1),[0.25, 0.58, 0.96])
end
grid on;
text(start_foot(1,1)-1, start_foot(1,2), start_foot(1,3)+2,"UAV Initial Position")
for i=1:numpoints
    plot3(start_foot(1, i),start_foot(2, i), start_foot(3, i),'MarkerSize',10,"Marker","*","Color","cyan") 
    plot3(goal_foot(1, i),goal_foot(2, i), goal_foot(3, i),'MarkerSize',10,"Marker","*","Color","cyan") 

end
text(goal_foot(1,1), goal_foot(1,2), goal_foot(1,3)+2,"UAV Target")



%Path Planning

obstacles = transpose(Cpos); %turning 
disp(obstacles)
iteration = 1000;                         %Iterations count
curr_foot=start_foot;


prev_foot=curr_foot;

Krep = 0.1;                             %Gain factor of repulsive potential field
Katt = 0.04;                             
delta = 0;

F=zeros(3,length(obstacles),5);

Urep = 0;
figure(1)
title('Path Planning of a UAV')


iterations=zeros(1,iteration);
for i=1:iteration
    p_Fr = 0;
    %robot_height = current_pos(3,1); %third element is z comp
    foot_heights=curr_foot(3,:);
    goal_heights=goal_foot(3,:);
    %flag = 0;
    flags=zeros(1,5);
    
    Fatt_list=zeros(3,5);
    for j=1:numpoints
        Fatt_list(:, j)=potential_attraction(Katt, curr_foot(:, j), goal_foot(:,j));
    end
    %Fatt_bp=potential_attraction(Katt, curr_bp_foot, bp_foothold_goal);
    %Fatt_t1=potential_attraction(Katt, curr_t1_foot, t1_foothold_goal);
    %Fatt1 = potential_attraction(Katt, current_pos, goal);
    %Fatt2 = potential_attraction(Katt, current_pos, goal);
    

    for k = 1: numObst
        %fprintf('The variable k is %d\n', k);
        zeta = 3.5*radius(k,1);
        n = 2;
        % Measuring the horizantal distance between UAV and centre axis of the building 
        %rou = sqrt((current_pos(1,1)-obstacles(1,k))^2+(current_pos(2,1)-obstacles(2,k))^2); 
        rous=zeros(1, numpoints);
        d_rous=zeros(2,numpoints);
        Frep1list=zeros(3,5);
        Frep2list=zeros(3,5);
        for l=1:numpoints %change rous to rou
            rou= sqrt((curr_foot(1,l)-obstacles(1,k))^2+(curr_foot(2,l)-obstacles(2,k))^2); 
            d_rou = [curr_foot(1,l)-obstacles(1,k); curr_foot(2,l)-obstacles(2,k)]/rou;
            %disp(rou)
            
            if rou<=zeta
                %disp(obstacles(3,k))
                if foot_heights(1,l) <= 100%obstacles(3,k)
                    flags(1,l)=1;
                    Frep1list(:, l)= vertcat((Krep*((1/rou)-(1/zeta))*(1/rou^2)*dist_factor(curr_foot(:,l), goal_foot(:,l), n, flags(1,l))*d_rou),0);
                    Frep2list(:,l) = -(n/2)*Krep*((1/rou)-(1/zeta))^2*dist_factor(curr_foot(:,l), goal_foot(:,1), n-1, flags(1,l))*diff_distance_factor(curr_foot(:,l), goal_foot(:,1), n, flags(1,l));
                    %disp(l)
                    %disp(Frep1list)
                    %disp(Frep2list)
                    F(:,k,l)=Frep1list(:,l)+Frep2list(:,l);
                else
                   F(:,k,l)=0;
                end
             
            elseif rous(1,l)>zeta
                F(:,k,l)=0;
            end

        end

        % Threshold value to judge whether the UAV near to buidling or not?
        
       
    end
    
    %Frep = sum(F,2); % summation of all repulsive forces, column vector
    Frep=zeros(3, numpoints);
    Ft=zeros(3, numpoints);
    for j=1:numpoints
        Frep(:,j)=sum(F(:,:,j),2);
    end
    disp(Frep)
    for j=1:numpoints
        Ft(:,j)=Frep(:,j)+Fatt_list(:,j);
        if flags(1, j) ==1
            flags(1,j)=0;
            Ft(3,j)=0;
        end
    end
    
    prev_foot=curr_foot;
    for j=1:numpoints
       curr_foot(:,j)=curr_foot(:,j)+Ft(:,j);
    end

    iterations(i,1)=i;
    
   
    
   % title(printf('Iterations %d', i))
    for j=1:numpoints
     plot3(curr_foot(1,j), curr_foot(2,j), curr_foot(3,j),"Marker","*","Color","black");
    
    end
    %plot3(curr_foot(1,:), curr_foot(2,:), curr_foot(3,:), 'r-','LineWidth', 2);
    %fill3(curr_foot(1,:), curr_foot(2,:), curr_foot(3,:), 'b');
    % Define the x, y, z coordinates of the points
    combs = nchoosek(1:size(curr_foot,2),2);

% plot lines between each pair of points
    for ii = 1:size(combs,1)
        x = curr_foot(1,combs(ii,:));
        y = curr_foot(2,combs(ii,:));
        z = curr_foot(3,combs(ii,:));
        line(x,y,z);
    end


    %bd = boundary(curr_foot', 1);
    %plot3(curr_foot(1,bd), curr_foot(2,bd), curr_foot(3,bd), 'r-', 'LineWidth', 2);
    %curr_foot_trans=curr_foot';
    %shp = alphaShape(curr_foot_trans);
   % k = boundaryFacets(shp);
    
    %plot3(curr_foot_trans(k,:)', curr_foot_trans(k,:)', 'r-', 'LineWidth', 2);

    %alpha_shape = alphaShape(curr_foot');
    %boundary_facets = alpha_shape.boundaryFacets();
    %plot3(curr_foot(boundary_facets',1), curr_foot(boundary_facets',2), curr_foot(boundary_facets',3), 'r-', 'LineWidth', 2);





   
   
    %plot3([]);
    pause(0.07)
    drawnow
end
%Plotting of graphs
figure(2)
plot(iterations, Frepx_vals,'r.');
xlabel('iterations')
ylabel('X respulsive force')
figure(3)
plot(iterations, Frepy_vals, 'r.');
xlabel('iterations')
ylabel('Y respulsive force')
figure(4)
plot(iterations, Fattrx_vals, 'r.');
xlabel('iterations')
ylabel('X attractive force')
figure(5)
plot(iterations, Fattry_vals, 'r.');
xlabel('iterations')
ylabel('Y attractive force')
for i = 1:length(obstacles)
    % Plotting other usefull plots to analyse the UAV behaviour
    %potential_plots(x,y,obstacles(:,i));
end