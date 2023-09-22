%Environment code
clf;
close all;
clear;
% Defining Goal position of the UAV.
goal = [300,300,0];
goals=[300; 300; 0];
start_foot=zeros(3,21);

worm=[0,8,0,-8,0,1,1,-1,-1,1,1,-1,-1,3,3,3,3,7,7,7,7; 0,0,-8,0,8,1,-1,1,-1,1,-1,-1,1,-1,1,-1,1,-1,1,-1,1; 0,1,1,1,1,2,2,2,2,7,7,7,7,9,9,11,11,10,10,12,12];
start_foot(1,:)=worm(1,:)+10;
start_foot(2,:)=worm(2,:)+10;
start_foot(3,:)=worm(3,:);


goal_foot=zeros(3,21);


goal_foot(1,:)=worm(1,:)+350;
goal_foot(2,:)=worm(2,:)+300;
goal_foot(3,:)=worm(3,:);

numpoints=size(start_foot,2);





% Defining intial position of the UAV.


numObst=18;
%Buildings Position for Rock
rng('shuffle');
Cpos = randi([0,400],numObst,2);
zcoords=zeros(numObst,1);
Cpos=[Cpos zcoords];
figure; hold on
view(3);
x = 0:4:400;
y = 0:4:400;
xlabel("x");
ylabel("y");
zlabel("z");
xlim([0 400]);
ylim([0 400]);
zlim([0 100]);
rng("shuffle")
%rock
radius = randi([8 15], numObst, 1); % Radius of the buildings

rng("shuffle")
nSides=randi([4 6],numObst,1);
for i =1:numObst
    create_rock(Cpos(i,:),radius(i,1),nSides(i,1),[0.25, 0.58, 0.96])
end
grid on;
text(start_foot(1,1)-1, start_foot(1,2), start_foot(1,3)+2,"WORM Initial Position")
text(goal_foot(1,1), goal_foot(1,2), goal_foot(1,3)+2,"WORM Target")

for i=1:numpoints
    plot3(start_foot(1, i),start_foot(2, i), start_foot(3, i),'MarkerSize',10,"Marker","*","Color","cyan") 
    plot3(goal_foot(1, i),goal_foot(2, i), goal_foot(3, i),'MarkerSize',10,"Marker","*","Color","cyan") 

end



%Path Planning

obstacles = transpose(Cpos); %turning 
disp(obstacles)
iteration = 500;                         %Iterations count
curr_foot=start_foot;


prev_foot=curr_foot;

Krep = 0.1;                            %Gain factor of repulsive potential field
Katt = 0.04;                             
delta = 0;

F=zeros(3,length(obstacles),numpoints);

Urep = 0;
figure(1)
title('Path Planning of a WORM')

title('Path Planning of a UAV')
%Graphing repulsive and attractive forces
Frepx_vals=zeros(1, iteration);
Frepy_vals=zeros(1, iteration);
Fattrx_vals=zeros(1, iteration);
Fattry_vals=zeros(1, iteration);
iterations=zeros(1,iteration);
for i=1:iteration
    p_Fr = 0;
    %robot_height = current_pos(3,1); %third element is z comp
    foot_heights=curr_foot(3,:);
    goal_heights=goal_foot(3,:);
    %flag = 0;
    flags=zeros(1,numpoints);
    
    Fatt_list=zeros(3,numpoints);
    for j=1:numpoints
        Fatt_list(:, j)=potential_attraction(Katt, curr_foot(:, j), goal_foot(:,j));
    end
    

    for k = 1: numObst
        %fprintf('The variable k is %d\n', k);
        zeta = 4.5*radius(k,1); %3.5
        n = 2;
        % Measuring the horizantal distance between UAV and centre axis of the building 
        %rou = sqrt((current_pos(1,1)-obstacles(1,k))^2+(current_pos(2,1)-obstacles(2,k))^2); 
        rous=zeros(1, numpoints);
        d_rous=zeros(2,numpoints);
        Frep1list=zeros(3,numpoints);
        Frep2list=zeros(3,numpoints);
        for l=1:numpoints %change rous to rou
            rou= sqrt((curr_foot(1,l)-obstacles(1,k))^2+(curr_foot(2,l)-obstacles(2,k))^2); 
            d_rou = [curr_foot(1,l)-obstacles(1,k); curr_foot(2,l)-obstacles(2,k)]/rou;
            
            if rou<=zeta
                if foot_heights(1,l) <= radius(k,1)%obstacles(3,k)
                    flags(1,l)=1;
                    Frep1list(:, l)= vertcat((Krep*((1/rou)-(1/zeta))*(1/rou^2)*dist_factor(curr_foot(:,l), goal_foot(:,l), n, flags(1,l))*d_rou),0);
                    Frep2list(:,l) = -(n/2)*Krep*((1/rou)-(1/zeta))^2*dist_factor(curr_foot(:,l), goal_foot(:,1), n-1, flags(1,l))*diff_distance_factor(curr_foot(:,l), goal_foot(:,1), n, flags(1,l));
                  
                    F(:,k,l)=Frep1list(:,l)+Frep2list(:,l);

                else
                   F(:,k,l)=0; %if we are over the obstacle no repulsion 
                end
             
            elseif rous(1,l)>zeta
                F(:,k,l)=0;
            end

        end

        % Threshold value to judge whether the UAV near to buidling or not?
        
       
    end
  
    Frep=zeros(3, numpoints);
    Frep_mag=zeros(1, numpoints);
    Ft=zeros(3, numpoints);
    for j=1:numpoints
        Frep(:,j)=sum(F(:,:,j),2); %could put this in for loop below r vice versa.
        Frep_mag(:,j)=norm(Frep(:,j)); 
    end
    
    for j=1:numpoints 
         Ft(:,j)=Frep(:,j)+Fatt_list(:,j);    
        if flags(1, j) ==1
            flags(1,j)=0;
            Ft(3,j)=0;
        end
        
    end
    Frep_avg=mean(Frep,2);
    Fatt_avg=mean(Fatt_list,2);
    Ft_avg=mean(Ft,2);
    prev_foot=curr_foot;
    for j=1:numpoints       
        curr_foot(:,j)=curr_foot(:,j)+Ft_avg;      
    end    
    iterations(i,1)=i;
    Frepx_vals(i,1)=Frep_avg(1,1);
    Frepy_vals(i,1)=Frep_avg(2,1);
   
    Fattrx_vals(i,1)=Fatt_avg(1,1);
    Fattry_vals(i,1)=Fatt_avg(2,1);
    iterations(i,1)=i;
    [max_Frep_mag, max_Frep_Index] = max(Frep_mag);
    [sortedFrep_mag, sortedFrep_mag] = sort(Frep_mag, 'descend'); %. The function returns both the sorted maximum values in descending order (sortedMatrix) and their corresponding indexes (sortedIndexes). The sortedMatrix contains the maximum values in descending order, and sortedIndexes contains their respective indexes.
    
   title(sprintf('Iterations %d', i))
    for j=1:numpoints
     if(mod(iteration,10)==0)
         if j<6
            plot3(curr_foot(1,j), curr_foot(2,j), curr_foot(3,j),"Marker",".","Color","green");
        else
            plot3(curr_foot(1,j), curr_foot(2,j), curr_foot(3,j),"Marker",".","Color","black");
         end
     end
    end
  
    combs = nchoosek(1:size(curr_foot,2),2);

% plot lines between each pair of points
    for ii = 1:size(combs,1)
        x = curr_foot(1,combs(ii,:));
        y = curr_foot(2,combs(ii,:));
        z = curr_foot(3,combs(ii,:));
        %line(x,y,z);
    end
    pause(.05)
    drawnow
end
%Plotting of graphs
figure(2)
plot(iterations, Frepx_vals,'r.');
xlabel('iterations')
ylabel('X avg respulsive force')
figure(3)
plot(iterations, Frepy_vals, 'r.');
xlabel('iterations')
ylabel('Y avg respulsive force')
figure(4)
plot(iterations, Fattrx_vals, 'r.');
xlabel('iterations')
ylabel('X avg attractive force')
figure(5)
plot(iterations, Fattry_vals, 'r.');
xlabel('iterations')
ylabel('Y avg attractive force')
