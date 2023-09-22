%Environment code
clf;
close all;
clear;
% Defining Goal position of the UAV.
goal = [185,120,0];
bp_foothold_goal= goal;
t1_foothold_goal= [160,180,0];%bp_foothold_goal+2*[0,1,1]; 
% Defining intial position of the UAV.
start = [10,10,0];
bp_foothold_start= start;
t1_foothold_start= [20, 80,10]; 

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

rng("shuffle")
nSides=randi([4 6],numObst,1);
for i =1:numObst
    %create_cylinder(radius(i,1), Cpos(i,:), [0.25, 0.58, 0.96])
    create_rock(Cpos(i,:),radius(i,1),nSides(i,1),[0.25, 0.58, 0.96])
end
grid on;
text(start(1,1)-1, start(1,2), start(1,3)+2,"UAV Initial Position")

plot3(bp_foothold_start(1,1), bp_foothold_start(1,2), bp_foothold_start(1,3),'MarkerSize',10,"Marker","*","Color","cyan")
plot3(t1_foothold_start(1,1), t1_foothold_start(1,2), t1_foothold_start(1,3),'MarkerSize',10,"Marker","*","Color","cyan")
text(bp_foothold_goal(1,1), bp_foothold_goal(1,2), bp_foothold_goal(1,3)+2,"UAV Target")

plot3(t1_foothold_start(1,1), t1_foothold_start(1,2), t1_foothold_start(1,3),'MarkerSize',10,"Marker","*","Color","cyan")
plot3(t1_foothold_goal(1,1), t1_foothold_goal(1,2), t1_foothold_goal(1,3),'-s','MarkerSize',10,'MarkerFaceColor','green')

%Path Planning
obstacles = transpose(Cpos); %turning 
iteration = 1000;                         %Iterations count
%current_pos = transpose(start); %turning into pos vector?
curr_bp_foot=transpose(bp_foothold_start);
curr_t1_foot=transpose(t1_foothold_start);

bp_foothold_goal = transpose(bp_foothold_goal);
t1_foothold_goal = transpose(t1_foothold_goal);

%previous_pos = current_pos;             %Intialising Previous position of the UAV  
prev_bp_foot=curr_bp_foot;
prev_t1_foot=curr_t1_foot;

Krep = 0.1;                             %Gain factor of repulsive potential field
Katt = 0.04;                             
delta = 0;
%data_points = zeros(iteration,3); % storing the iteration values positions of UAV
data_points_bp = zeros(iteration,3); % storing the iteration values positions of UAV
data_points_t1 = zeros(iteration,3); % storing the iteration values positions of UAV

%F = zeros(3,length(obstacles));
F_bp=zeros(3,length(obstacles));
F_t1=zeros(3, length(obstacles));
Urep = 0;
figure(1)
title('Path Planning of a UAV')
%Graphing repulsive and attractive forces
%Frepx_vals=zeros(1, iteration);
%Frepy_vals=zeros(1, iteration);
%Fattrx_vals=zeros(1, iteration);
%Fattry_vals=zeros(1, iteration);

iterations=zeros(1,iteration);
for i=1:iteration
    p_Fr = 0;
    %robot_height = current_pos(3,1); %third element is z comp
    bp_foot_height = curr_bp_foot(3,1); %third element is z comp
    t1_foot_height = curr_t1_foot(3,1); %third element is z comp

    %goal_height = goal(3,1);
    bp_foot_goal_height=bp_foothold_goal(3,1);
    t1_foot_goal_height=t1_foothold_goal(3,1); %error
    %flag = 0;
    flag_bp=0;
    flag_t1=0;
    %Fatt = potential_attraction(Katt, current_pos, goal);
    Fatt_bp=potential_attraction(Katt, curr_bp_foot, bp_foothold_goal);
    Fatt_t1=potential_attraction(Katt, curr_t1_foot, t1_foothold_goal);
    %Fatt1 = potential_attraction(Katt, current_pos, goal);
    %Fatt2 = potential_attraction(Katt, current_pos, goal);


    for k = 1: length(obstacles)
        % Measuring the horizantal distance between UAV and centre axis of the building 
        %rou = sqrt((current_pos(1,1)-obstacles(1,k))^2+(current_pos(2,1)-obstacles(2,k))^2); 
        rou_bp=sqrt((curr_bp_foot(1,1)-obstacles(1,k))^2+(curr_bp_foot(2,1)-obstacles(2,k))^2); 
        % differentiation of variable rou 
        %d_rou = [current_pos(1,1)-obstacles(1,k); current_pos(2,1)-obstacles(2,k)]/rou; 
        d_rou_bp = [curr_bp_foot(1,1)-obstacles(1,k); curr_bp_foot(2,1)-obstacles(2,k)]/rou_bp; 
        % Threshold value to judge whether the UAV near to buidling or not?
        zeta = 3.5*radius(k,1);
        n = 2;
        if rou_bp<=zeta
            if bp_foot_height <= obstacles(3,k)
                % Flag - that tells the UAV to move in xy plane
                % no increment in height. 
                %flag = 1;
                flag_bp=1;
                %Frep1 = Krep*((1/rou)-(1/zeta))*(1/rou^2)*dist_factor(current_pos, goal, n, flag)*d_rou;
                Frep1_bp = Krep*((1/rou_bp)-(1/zeta))*(1/rou_bp^2)*dist_factor(curr_bp_foot, bp_foothold_goal, n, flag_bp)*d_rou_bp;

                %fprintf('Frep1= %i\n', Frep1)
                %Frep2 = -(n/2)*Krep*((1/rou)-(1/zeta))^2*dist_factor(current_pos, goal, n-1, flag)*diff_distance_factor(current_pos, goal, n, flag);
                Frep2_bp = -(n/2)*Krep*((1/rou_bp)-(1/zeta))^2*dist_factor(curr_bp_foot, bp_foothold_goal, n-1, flag_bp)*diff_distance_factor(curr_bp_foot, bp_foothold_goal, n, flag_bp);
               
                %F(:,k) = vertcat(Frep1,0)+Frep2; 
                F_bp(:,k)=vertcat(Frep1_bp,0)+Frep2_bp;
                fprintf('Frep2_bp \n');
                disp(Frep2_bp);
                fprintf('vertcat(Frep1_bp,0) \n');
                disp(vertcat(Frep1_bp,0));
                fprintf('Frep1_bp \n');
                disp(Frep1_bp);
                Fatt_bp = Katt*[bp_foothold_goal(1,1)-curr_bp_foot(1,1);bp_foothold_goal(2,1)-curr_bp_foot(2,1); 0];

            else
                %F(:,k) = 0;
                F_bp(:,k)=0;
            end
        elseif rou_bp > zeta
            F_bp(:,k)=0;
        end
        rou_t1=sqrt((curr_t1_foot(1,1)-obstacles(1,k))^2+(curr_t1_foot(2,1)-obstacles(2,k))^2); 
        d_rou_t1 = [curr_t1_foot(1,1)-obstacles(1,k); curr_t1_foot(2,1)-obstacles(2,k)]/rou_t1; 

        if rou_t1<=zeta
            if t1_foot_height <= obstacles(3,k)
                % Flag - that tells the UAV to move in xy plane
                % no increment in height. 
                flag_t1=1;
                Frep1_t1 = Krep*((1/rou_t1)-(1/zeta))*(1/rou_t1^2)*dist_factor(curr_t1_foot, t1_foothold_goal, n, flag_t1)*d_rou_t1;

                Frep2_t1 = -(n/2)*Krep*((1/rou_t1)-(1/zeta))^2*dist_factor(curr_t1_foot, t1_foothold_goal, n-1, flag_t1)*diff_distance_factor(curr_t1_foot, t1_foothold_goal, n, flag_t1);

                F_t1(:,k)=vertcat(Frep1_t1,0)+Frep2_t1;

                Fatt_t1 = Katt*[t1_foothold_goal(1,1)-curr_t1_foot(1,1);t1_foothold_goal(2,1)-curr_t1_foot(2,1); 0];

            else
                %F(:,k) = 0;
                F_t1(:,k)=0;
            end
        elseif rou_t1 > zeta
            
            %F(:,k) = 0;
            F_t1(:,k)=0;
        end
    end
    disp(F_bp)
    Frep_bp = sum(F_bp,2); % summation of all repulsive forces, column vector
    Frep_t1 = sum(F_t1,2); % summation of all repulsive forces, column vector

 
    iterations(i,1)=i;
    
    %Ft = Fatt + Frep;
    Ft_bp=Fatt_bp+Frep_bp;
    Ft_t1=Fatt_t1+Frep_t1;
    
    if flag_bp == 1
        flag_bp = 0;
        % changing the 'z' axis based on the UAV position if robot is near to obstracle it only moves in xy plane.
        Ft_bp(3,1) = 0; 
    end
    if flag_t1 == 1
        flag_t1 = 0;
        % changing the 'z' axis based on the UAV position if robot is near to obstracle it only moves in xy plane.
        Ft_t1(3,1) = 0; 
    end
    %previous_pos = current_pos;
    prev_bp_foot=curr_bp_foot;
    prev_t1_foot=curr_t1_foot;
    %current_pos = current_pos+Ft;
    curr_bp_foot=curr_bp_foot+Ft_bp;    
    curr_t1_foot=curr_t1_foot+Ft_t1;

    %data_points(i,:)=transpose(current_pos);
    data_points_bp(i,:)=transpose(curr_bp_foot);

    title(sprintf('Iterations %d', i))
    % Plotting the UAV position in real time
    plot3(curr_bp_foot(1,1), curr_bp_foot(2,1), curr_bp_foot(3,1),"Marker","*","Color","black");
    plot3(curr_t1_foot(1,1), curr_t1_foot(2,1), curr_t1_foot(3,1),"Marker","*","Color","black");

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