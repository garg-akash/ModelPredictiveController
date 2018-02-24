%%IT is for 2d only
% p <= n; for increased planning steps 
%jerk cost added
clc;
clear all;
close all;

p = 40; %sensing range steps
n= 40; %planning range steps
sense = 5;
del_t = 0.1; % Time duration for which some command is executed.
%tot_time = n*del_t; %% Total time of flight

%% defining drone uncertainty
% drone_sig_orig = [0.02, 0.0, 0.0; 0.0, 0.02, 0.0; 0.0, 0.0, 0.02];
% drone_sig_orig = [0.02, 0.0; + jerk_cost0.0, 0.02];

%% Defining obstacle trajectory
%% For now we will assume that obstacle is moving along a straing line. it's starting from (0,0,0) and It's trajectory will be determined based on the destination of the drone

start_x = 0;
start_y = 0;
Px_not = start_x;
Py_not = start_y;
%% Drone Destination variables
dest_x = 40;
dest_y = 0;

% wp_ind = [1,0,20; 2,0,40; 3,0,60; 4,0,80; 5,0,100; 6,0,120; 7,0,140; 8,0,160; 9,0,180; 10,0,200; 
%     11,0,220; 12,0,240; 13,0,260; 14,0,280; 15,0,300; 16,0,320; 17,0,340; 18,0,360; 19,0,380; 20,0,400;
%     21,0,420; 22,0,440; 23,0,460; 24,0,480; 25,0,500; 26,0,520; 27,0,540; 28,0,560; 29,0,580; 30,0,600;
%     31,0,620; 32,0,640; 33,0,660; 34,0,680; 35,0,700; 36,0,720; 37,0,740; 38,0,760; 39,0,780];

mit_dist = 39;
mit_inter_dist = 0.25;
mit_avg_speed = 0.5;
mit_x = 1:mit_inter_dist:mit_dist;
mit_step = mit_inter_dist/(mit_avg_speed*del_t);
mit_y = linspace(0,0,size(mit_x,2));
mit_r = mit_step:mit_step:(size(mit_x,2))*mit_step;
wp_ind = [mit_x',mit_y',mit_r'];



i = 1;
ln = 0;
while (i<=size(wp_ind,1))
    if i==1
        ln = ln + sqrt((wp_ind(1,1)-start_x)^2 + (wp_ind(1,2)-start_y)^2);
        arc_ind(1,1) = ln;
        arc_ind(1,2) = wp_ind(1,3);
    else
        ln = ln + sqrt((wp_ind(i,1)-wp_ind(i-1,1))^2 + (wp_ind(i,2)-wp_ind(i-1,2))^2);
        arc_ind(i,1) = ln;
        arc_ind(i,2) = wp_ind(i,3);
    end
    i = i+1;
end

%%%%%%%%%%%%%% Obstacle stuff starts here!!!! %%%%%%%%%%%%%%%%%
Ob_start=[4,0];
Obx_not = Ob_start(1);
Oby_not = Ob_start(2);
% Ob_end=[start_x,start_y,start_z];

%% Calculating velocity of the obstacle in all the directions
vel_obs = 0;
theta_obs = -pi;

R_obs = 0.5;
R_drone = 0.5;

%%%%%%%%%Obstacle stuff ends here %%%%%%%%%%%%%%%

%% Difference between subsequent velocities
del_Vx = 0.5;
del_Vy = 0.5;
% del_Vz = 0.5;

dist = 1;
c = 0;

Px_arr = [];
Py_arr = [];
Ox_arr = [];
Oy_arr = [];
Vx_arr = [];
Vy_arr = [];
ax = 3;
ay = 2;

while (dist>0.8)
    l = 0;
    myflag = 1;
    prev_optval = 0;
    R = 0.2;
    while (myflag)
        qn = 0;
        cost = 0;
        jerk_cost = 0;
        j = [];  %%To track WP in planning range
        cvx_begin quiet
        clear Vx Vy Vz Px Py Pz
        variables Vx(n) Vy(n) %% Velocities
        %variables Px(n) Py(n) Pz(n) %% Drone locations at each timestep
        
        for i = 1:n
            Px(i,1) = Px_not + sum(Vx(1:i))*del_t;
            Px(i,2) = i;
            Py(i,1) = Py_not + sum(Vy(1:i))*del_t;
            Py(i,2) = i;
%             if i==1
%                 qn = qn + sqrt((Px(1,1)-start_x)^2 + (Py(1,1)-start_y)^2);
%                 q_ind(1,1) = qn;
%                 q_ind(1,2) = i;
%             else
%                 qn = qn + sqrt((Px(i,1)-Px(i-1,1))^2 + (Py(i,1)-Py(i-1,1))^2);
%                 q_ind(i,1) = qn;
%                 q_ind(i,2) = i;
%             end
            
            Ox(i,1) = Obx_not + i*vel_obs*cos(theta_obs)*del_t;
            Ox(i,2) = i;
            Oy(i,1) = Oby_not + i*vel_obs*sin(theta_obs)*del_t;
            Oy(i,2) = i;
        end
        
        %% Cost function
        for i = 1:size(wp_ind,1)
            if (0< wp_ind(i,3) && wp_ind(i,3)<=n)       %%Don't put equals to 0
                j = [j;i];
                cost = cost + (Px(wp_ind(i,3),1) - wp_ind(i,1))^2 + (Py(wp_ind(i,3),1) - wp_ind(i,2))^2;
%                 if l > 0
%                     for q = 2:(n-1)
%                         jerk_cost = jerk_cost + ((-Vx(q-1) + 2*Vx(q) - Vx(q+1))/(del_t)^2)^2 + ((-Vy(q-1) + 2*Vy(q) - Vy(q+1))/(del_t)^2)^2;
%                     end
%                 end
            end
        end
        minimise(cost)
        
        %%Constraint for onstacle avoidance
        if(l~=0)
            fprintf('Distance: %f l: %d\n',sqrt((x(1,1) - Ox(1,1))^2 + (y(1,1) - Oy(1,1))^2),l)
            %if (eucldFunc(Px(find(Px(:,2))==tc), Py(find(Py(:,2))==tc), Ox(find(Ox(:,2))==tc), Oy(find(Oy(:,2))==tc))< sense)
            if (sqrt((x(1,1) - Ox(1,1))^2 + (y(1,1) - Oy(1,1))^2) < sense)
                for i = 1:p
                    (x(i) - Ox(i))^2 + (y(i) - Oy(i))^2 + 2*(x(i) - Ox(i))*(Px(i) -  x(i)) + 2*(y(i) - Oy(i))*(Py(i) -  y(i)) >= R^2;
                end
                fprintf('Entered!!\n');
                fprintf('l: %d R: %d\n',l,R);
%                 if (R<0.2)
%                     R = R + 0.05;
%                 end
            end
        end
        %%%%%%%% Constraints %%%%%%%%
        Vx <= 1;
        Vy <= 1;
       
        %% The velocity during initialization shouldn't be much higher
%         0 <= Vx(1) <= del_Vx;
%         -del_Vy <= Vy(1) <= del_Vy;
        %     0 <= Vz(1) <= del_Vz;
        
%         0 <= Vx(n) <= 1.0;
%         0 <= Vy(n) <= 1.0;
        %     0 <= Vz(n) <= 1.0;
        
        %	% Ensuring that subsequent X vel commands do not have difference more than del_Vx

        -ax*del_t <= Vx(2:n) - Vx(1:n-1);
        
        Vx(2:n) - Vx(1:n-1) <= ax*del_t;
        
        %% Ensuring that subsequent Y vel commands do not have difference more than del_Vy
        -ay*del_t <= Vy(2:n) - Vy(1:n-1);
        
        Vy(2:n) - Vy(1:n-1) <= ay*del_t;
        
        %% Ensuring that subsequent Z vel commands do not have difference more than del_Vz
        %     0 <= Vz(2:n) - Vz(1:n-1) <= del_Vz;
        
        %%%%%%%% Constraints definition over!! %%%%%%%%%
        cvx_end
        disp(cvx_optval);
        fprintf('Vx1:%f Vx2:%f Vx3:%f\n',Vx(1),Vx(2),Vx(3));
        fprintf('Vy1:%f Vy2:%f Vy3:%f\n\n',Vy(1),Vy(2),Vy(3));
        %% Updating the linearization points!
        git_x = double(Vx);
        git_y = double(Vy);
        
        x = Px_not + cumsum(git_x*del_t);
        y = Py_not + cumsum(git_y*del_t);
        if(l == 0)
            x = x + 0.1*rand(n,1);
            y = y + 0.1*rand(n,1);
        end
        %     z = Pz;
   
        l = l + 1;
     
        if ((cvx_optval - prev_optval) < 0.1 && l>4)
            myflag = 0;
            for i = 1:size(wp_ind,1)
                wp_ind(i,3) = wp_ind(i,3) - 1;
            end
        end
        prev_optval = cvx_optval;
        
        figure(4);
        cla;
        hold on;
        axis equal;
        p1 = plot(Px(:,1),Py(:,1),'r','LineWidth',2);
        hold on
        p2 = plot(wp_ind(j,1), wp_ind(j,2),'o','MarkerEdgeColor','b');
        pause(1);
%         delete(p1);
%         delete(p2);
        
        %R = R + 1
    end
    c = c + 1;
    Px_not = Px(1);
    Py_not = Py(1);
    Obx_not = Ox(1);
    Oby_not = Oy(1);
    
    %%Plotting
    
    Px_arr = [Px_arr;Px_not];   
    Py_arr = [Py_arr;Py_not];
    Ox_arr = [Ox_arr;Obx_not];
    Oy_arr = [Oy_arr;Oby_not];
    
    figure(10);
    hold on;
    axis equal;
    plot(Px_arr,Py_arr,'r','LineWidth',3)
    plot(Ox_arr,Oy_arr,'b','LineWidth',3)
    pause(1);
 
    Vx_arr = [Vx_arr;Vx(1)];
    Vy_arr = [Vy_arr;Vy(1)];
    
    dist = sqrt((Px(1)-dest_x)^2 + (Py(1)-dest_y)^2);
end %% End of while loop


