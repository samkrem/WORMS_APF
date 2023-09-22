%Environment code
clf;
close all;
clear;
% Defining Goal position of the UAV.
goal = [185,120,0];
% Defining intial position of the UAV.
start = [10,10,10];
numObst=25;
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
%cylinder
%radius=[10;15;13;15;11;15;16;14;19;17;15];
%{
create_cylinder(radius(1,1),Cpos(1,:),[0.25, 0.58, 0.96]) %[Radius, X-position, Y-position, Color]
create_cylinder(radius(2,1),Cpos(2,:),[0.25, 0.58, 0.96])
create_cylinder(radius(3,1),Cpos(3,:),[0.25, 0.58, 0.96])
create_cylinder(radius(4,1),Cpos(4,:),[0.25, 0.58, 0.96])
create_cylinder(radius(5,1),Cpos(5,:),[0.25, 0.58, 0.96])
create_cylinder(radius(6,1),Cpos(6,:),[0.25, 0.58, 0.96])
create_cylinder(radius(7,1),Cpos(7,:),[0.25, 0.58, 0.96]) %[Radius, X-position, Y-position, Color]
create_cylinder(radius(8,1),Cpos(8,:),[0.25, 0.58, 0.96])
create_cylinder(radius(9,1),Cpos(9,:),[0.25, 0.58, 0.96])
create_cylinder(radius(10,1),Cpos(10,:),[0.25, 0.58, 0.96])
create_cylinder(radius(11,1),Cpos(11,:),[0.25, 0.58, 0.96])
%}
rng("shuffle")
nSides=randi([4 6],numObst,1);
for i =1:numObst
    %create_cylinder(radius(i,1), Cpos(i,:), [0.25, 0.58, 0.96])
    create_rock(Cpos(i,:),radius(i,1),nSides(i,1),[0.25, 0.58, 0.96])
end
grid on;
text(start(1,1)-1, start(1,2), start(1,3)+2,"UAV Initial Position")
plot3(start(1,1), start(1,2), start(1,3),'MarkerSize',10,"Marker","*","Color","cyan")
text(goal(1,1), goal(1,2), goal(1,3)+2,"UAV Target")
plot3(goal(1,1), goal(1,2), goal(1,3),'-s','MarkerSize',10,'MarkerFaceColor','green')
%Path Planning
obstacles = transpose(Cpos); %turning 
iteration = 1000;                         %Iterations count
current_pos = transpose(start); %turning into pos vector?
goal = transpose(goal);
previous_pos = current_pos;             %Intialising Previous position of the UAV  
Krep = 0.1;                             %Gain factor of repulsive potential field
Katt = 0.04;                             
delta = 0;
data_points = zeros(iteration,3); % storing the iteration values positions of UAV
F = zeros(3,length(obstacles));
Urep = 0;
figure(1)
title('Path Planning of a UAV')
%Graphing repulsive and attractive forces
Frepx_vals=zeros(1, iteration);
Frepy_vals=zeros(1, iteration);
Fattrx_vals=zeros(1, iteration);
Fattry_vals=zeros(1, iteration);
iterations=zeros(1,iteration);
for i=1:iteration
    p_Fr = 0;
    robot_height = current_pos(3,1); %third element is z comp
    goal_height = goal(3,1);
    flag = 0;
    Fatt = potential_attraction(Katt, current_pos, goal);
    for k = 1: length(obstacles)
        % Measuring the horizantal distance between UAV and centre axis of the building 
        rou = sqrt((current_pos(1,1)-obstacles(1,k))^2+(current_pos(2,1)-obstacles(2,k))^2); 
        % differentiation of variable rou 
        d_rou = [current_pos(1,1)-obstacles(1,k); current_pos(2,1)-obstacles(2,k)]/rou; 
        
        % Threshold value to judge whether the UAV near to buidling or not?
        zeta = 3.5*radius(k,1);
        n = 2;
        if rou<=zeta
            if robot_height <= obstacles(3,k)
                % Flag - that tells the UAV to move in xy plane
                % no increment in height. 
                flag = 1;
                Frep1 = Krep*((1/rou)-(1/zeta))*(1/rou^2)*dist_factor(current_pos, goal, n, flag)*d_rou;
                %fprintf('Frep1= %i\n', Frep1)
                Frep2 = -(n/2)*Krep*((1/rou)-(1/zeta))^2*dist_factor(current_pos, goal, n-1, flag)*diff_distance_factor(current_pos, goal, n, flag);
                F(:,k) = vertcat(Frep1,0)+Frep2; 
                Fatt = Katt*[goal(1,1)-current_pos(1,1);goal(2,1)-current_pos(2,1); 0];
            else
                F(:,k) = 0;
            end
        elseif rou > zeta
            
            F(:,k) = 0;
        end
    end
    Frep = sum(F,2); % summation of all repulsive forces, column vector
    %disp(Frep)
    %disp(Fatt);
    %disp(class(Frep_vals));
    
    Frepx_vals(i,1)=Frep(1,1);
    Frepy_vals(i,1)=Frep(2,1);
   
    Fattrx_vals(i,1)=Fatt(1,1);
    Fattry_vals(i,1)=Fatt(2,1);
    iterations(i,1)=i;
    
    Ft = Fatt + Frep;
    if flag == 1
        flag = 0;
        % changing the 'z' axis based on the UAV position if robot is near to obstracle it only moves in xy plane.
        Ft(3,1) = 0; 
    end
    previous_pos = current_pos;
    current_pos = current_pos+Ft;
    data_points(i,:)=transpose(current_pos);
    title(sprintf('Iterations %d', i))
    % Plotting the UAV position in real time
    plot3(current_pos(1,1), current_pos(2,1), current_pos(3,1),"Marker","*","Color","black");
    plot3(current_pos(1,1), current_pos(2,1), current_pos(3,1)+10,"Marker","*","Color","black");
    plot3(current_pos(1,1)-5, current_pos(2,1)-5, current_pos(3,1)+15,"Marker","*","Color","black");
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