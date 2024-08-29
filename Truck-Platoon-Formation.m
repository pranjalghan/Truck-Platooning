%% Scenario 1
% Simulation and animation of a four-truck platooning at 100km/h with 1 second time gap.
% Truck platoon formation for optimal platoon spacing and safety


clear ; close all ; clc

N_trucks = 4;                   % Number of trucks

% Parameters
tf      = 40;                   % Final time                            [s]
fR      = 30;                   % Frame rate                            [fps]
dt      = 1/fR;                 % Time resolution                       [s]
time    = linspace(0,tf,tf*fR); % Time                                  [s]

% Road
distance_analysis   = 200;      % Distance of analysis                  [m]
trackWidth          = 20;       % Track width                           [m]
laneWidth           = 4.3;      % Lane width                            [m]
laneMargin          = 2;        % Margin lane                           [m]

% Truck 1 (Leading truck)
truck_1_length      = 20;       % Length of the leading truck           [m]
truck_1_width       = 3.0;        % Width of the leading truck            [m]
truck_1_initial_speed    = 100/3.6;   % Speed of the leading truck       [m/s]
truck_1_initial_position = 150; % Initial position of the leading truck [m]

% Truck 2 (Second truck)
truck_2_length      = 20;       % Length of the second truck            [m]
truck_2_width       = 3.0;        % Width of the second truck             [m]
truck_2_initial_speed    = 100/3.6;   % Speed of the second truck        [m/s]
truck_2_initial_position = 120;  % Initial position of the second truck  [m]

% Truck 3 (Third truck)
truck_3_length      = 20;       % Length of the third truck            [m]
truck_3_width       = 3.0;        % Width of the third truck             [m]
truck_3_initial_speed    = 100/3.6;   % Speed of the third truck        [m/s]
truck_3_initial_position = 80;  % Initial position of the third truck  [m]

% Truck 4 (Fourth truck)
truck_4_length      = 20;       % Length of the fourth truck           [m]
truck_4_width       = 3.0;        % Width of the fourth truck            [m]
truck_4_initial_speed    = 100/3.6;   % Speed of the fourth truck       [m/s]
truck_4_initial_position = 30;  % Initial position of the fourth truck [m]

% Car 1
car_length          = 5;        % Length of the car                     [m]
car_width           = 2.6;      % Width of the car                      [m]
car_init_speed      = 72/3.6;   % Speed of the car                      [m/s]
% Initial position of the car           [m]
car_init_lon_pos    = truck_1_initial_position-2*truck_1_length + 10;
% Initial lateral position of the car   [m]
car_init_lat_pos    = -4.3;  

% Car 2
car2_length         = 5;        % Length of the car2        [m]
car2_width          = 2.6;      % Width of the car2         [m]
car2_init_speed     = 72/3.6;   % Speed of the car2         [m/s]
car2_init_lon_pos   = truck_3_initial_position - 2 * truck_3_length + 10;
car2_init_lat_pos   = 4.3;

% PARAMETERS 
% Struct to ode45
parameters.truck_1_length = truck_1_length;
parameters.truck_2_length = truck_2_length;
parameters.truck_3_length = truck_3_length;
parameters.truck_4_length = truck_4_length;
% Car1
parameters.car_length       = car_length;
parameters.car_width        = car_width;
% Car2
parameters.car2_length      = car2_length;
parameters.car2_width       = car2_width;

%% Simulation

% TIME
TSPAN = time;                   % Simulation time span  [s]

% INITIAL CONDITIONS
% Pair position and speed of first truck, pair position and speed of the
% second truck and so on ...
states_initial_condition = [truck_1_initial_position 
                            truck_1_initial_speed
                            truck_2_initial_position
                            truck_2_initial_speed
                            truck_3_initial_position
                            truck_3_initial_speed
                            truck_4_initial_position
                            truck_4_initial_speed
                            car_init_lon_pos
                            car_init_speed
                            car_init_lat_pos
                            car2_init_lon_pos
                            car2_init_speed
                            car2_init_lat_pos];

% SIMULATION
options = odeset('RelTol',1e-9,'AbsTol',1e-9);
[TOUT,YOUT] = ode45(@(t,z) simulation(t,z,parameters),TSPAN,states_initial_condition,options);

% RETRIEVING STATES
% Truck 1
truck_1_position    = YOUT(:,1);
truck_1_speed       = YOUT(:,2);
% Truck 2
truck_2_position    = YOUT(:,3);
truck_2_speed       = YOUT(:,4);
% Truck 3
truck_3_position    = YOUT(:,5);
truck_3_speed       = YOUT(:,6);
% Truck 4
truck_4_position    = YOUT(:,7);
truck_4_speed       = YOUT(:,8);
% Car
car_pos_lon         = YOUT(:,9);
car_speed           = YOUT(:,10);
car_pos_lat         = YOUT(:,11);
% Car2
car2_pos_lon         = YOUT(:,12);
car2_speed           = YOUT(:,13);
car2_pos_lat           = YOUT(:,14);


% Acceleration
% Preallocating
truck_1_acc         = zeros(1,length(TOUT));
truck_2_acc         = zeros(1,length(TOUT));
truck_3_acc         = zeros(1,length(TOUT));
truck_4_acc         = zeros(1,length(TOUT));
for i=1:length(TOUT)
    [dz]    = simulation(TOUT(i),YOUT(i,:),parameters);
    truck_1_acc(i)  = dz(2);
    truck_2_acc(i)  = dz(4);
    truck_3_acc(i)  = dz(6);
    truck_4_acc(i)  = dz(8);
end

% Distances
dist_1_2 = truck_1_position - truck_2_position - truck_1_length;
dist_2_3 = truck_2_position - truck_3_position - truck_2_length;
dist_3_4 = truck_3_position - truck_4_position - truck_3_length;

%% Results

c = cool(N_trucks); % Colormap
    
figure
set(gcf,'Position',[50 50 1280 720]) % 720p
% set(gcf,'Position',[50 50 854 480]) % 480p

% Create and open video writer object
v = VideoWriter('truck_platoon_string.avi');
v.Quality = 100;
open(v);

for i=1:length(time)
    subplot(3,2,1)
        hold on ; grid on
        position_max = max(truck_1_position);
        set(gca,'xlim',[0 TOUT(end)],'ylim',[0 1.2*position_max])
        cla 
        plot(TOUT,truck_1_position,'color',c(1,:))
        plot(TOUT,truck_2_position,'color',c(2,:))
        plot(TOUT,truck_3_position,'color',c(3,:))
        plot(TOUT,truck_4_position,'color',c(4,:))
        plot([time(i) time(i)],[0 1.2*position_max],'k--') 
        xlabel('Time [s]')
        ylabel('Position [m]')
        title('Position')
        legend('Truck 1','Truck 2','Truck 3','Truck 4','location','SouthEast')
    subplot(3,2,3)
        hold on ; grid on
        speed_max = max(max([truck_1_speed truck_2_speed truck_3_speed truck_4_speed]));
        set(gca,'xlim',[0 TOUT(end)],'ylim',[0 1.2*speed_max])
        cla 
        plot(TOUT,truck_1_speed,'color',c(1,:))
        plot(TOUT,truck_2_speed,'color',c(2,:))
        plot(TOUT,truck_3_speed,'color',c(3,:))
        plot(TOUT,truck_4_speed,'color',c(4,:))
        plot([time(i) time(i)],[0 1.2*speed_max],'k--') 
        xlabel('Time [s]')
        ylabel('Speed [m/s]')
        title('Speed')
        legend('Truck 1','Truck 2','Truck 3','Truck 4','location','SouthEast')
    subplot(3,2,2)
        hold on ; grid on
        acc_min = min(min([truck_1_acc truck_2_acc truck_3_acc truck_4_acc]));
        acc_max = max(max([truck_1_acc truck_2_acc truck_3_acc truck_4_acc]));
        set(gca,'xlim',[0 TOUT(end)],'ylim',[1.2*acc_min 1.2*acc_max])
        cla 
        plot(TOUT,truck_1_acc,'color',c(1,:))
        plot(TOUT,truck_2_acc,'color',c(2,:))
        plot(TOUT,truck_3_acc,'color',c(3,:))
        plot(TOUT,truck_4_acc,'color',c(4,:))
        plot([time(i) time(i)],[1.2*acc_min 1.2*acc_max],'k--') 
        xlabel('Time [s]')
        ylabel('Acceleration [m/s2]')
        title('Acceleration')
        legend('Truck 1','Truck 2','Truck 3','Truck 4','location','SouthEast')
    subplot(3,2,4)
        hold on ; grid on
        dist_max = max(max([dist_1_2 dist_2_3 dist_3_4]));
        set(gca,'xlim',[0 TOUT(end)],'ylim',[0 1.1*dist_max])
        cla 
        plot(TOUT,dist_1_2,'color',c(2,:))
        plot(TOUT,dist_2_3,'color',c(3,:))
        plot(TOUT,dist_3_4,'color',c(4,:))
        plot([time(i) time(i)],[0 1.2*dist_max],'k--') 
        xlabel('Time [s]')
        ylabel('Distance [m]')
        title('Separation Distance')
        legend('Trucks 1 & 2','Trucks 2 & 3','Trucks 3 & 4','location','SouthEast')
    subplot(3,2,5:6)
        hold on ; axis equal
        cla 
        % POSITION AT INSTANT [m]
        % Trucks
        truck_1_position_inst   = truck_1_position(i);
        truck_2_position_inst   = truck_2_position(i);
        truck_3_position_inst   = truck_3_position(i);
        truck_4_position_inst   = truck_4_position(i);
        % Car 
        car_pos_lon_inst        = car_pos_lon(i);
        car_pos_lat_inst        = car_pos_lat(i);
        % Car2
        car2_pos_lon_inst = car2_pos_lon(i);
        car2_pos_lat_inst = car2_pos_lat(i);

        % ROAD MARKINGS
        sideMarkingsX = [truck_1_position_inst-distance_analysis truck_1_position_inst];
        set(gca,'xlim',[truck_1_position_inst-distance_analysis truck_1_position_inst],'ylim',[-trackWidth/2-laneMargin +trackWidth/2+laneMargin])
        set(gca,'XTick',0:20:truck_1_position(end))
        % Central lane left marking
        plot(sideMarkingsX,[+laneWidth/2 +laneWidth/2],'k--') 
        % Central lane right marking
        plot(sideMarkingsX,[-laneWidth/2 -laneWidth/2],'k--') 
        % left marking left lane
        plot(sideMarkingsX,[laneWidth+laneWidth/2 laneWidth+laneWidth/2],'k--')
        % right marking right lane
        plot(sideMarkingsX,[-laneWidth-laneWidth/2 -laneWidth-laneWidth/2],'k--')

        % DIMENSIONS
        % Truck 1
        truck_1_dimension_X = [truck_1_position_inst truck_1_position_inst truck_1_position_inst-truck_1_length truck_1_position_inst-truck_1_length];
        truck_1_dimension_Y = [+truck_1_width/2 -truck_1_width/2 -truck_1_width/2 +truck_1_width/2];
        % Truck 2
        truck_2_dimension_X = [truck_2_position_inst truck_2_position_inst truck_2_position_inst-truck_2_length truck_2_position_inst-truck_2_length];
        truck_2_dimension_Y = [+truck_2_width/2 -truck_2_width/2 -truck_2_width/2 +truck_2_width/2];
        % Truck 3
        truck_3_dimension_X = [truck_3_position_inst truck_3_position_inst truck_3_position_inst-truck_3_length truck_3_position_inst-truck_3_length];
        truck_3_dimension_Y = [+truck_3_width/2 -truck_3_width/2 -truck_3_width/2 +truck_3_width/2];
        % Truck 4
        truck_4_dimension_X = [truck_4_position_inst truck_4_position_inst truck_4_position_inst-truck_4_length truck_4_position_inst-truck_4_length];
        truck_4_dimension_Y = [+truck_4_width/2 -truck_4_width/2 -truck_4_width/2 +truck_4_width/2];
        % Car
        car_dimension_X = [car_pos_lon_inst car_pos_lon_inst car_pos_lon_inst-car_length car_pos_lon_inst-car_length];
        car_dimension_Y = [car_pos_lat_inst+car_width/2 car_pos_lat_inst-car_width/2 car_pos_lat_inst-car_width/2 car_pos_lat_inst+car_width/2];
        % Car 2 dimensions for plotting
        car2_dimension_X = [car2_pos_lon_inst car2_pos_lon_inst car2_pos_lon_inst-car2_length car2_pos_lon_inst-car2_length];
        car2_dimension_Y = [car2_pos_lat_inst+car2_width/2 car2_pos_lat_inst-car2_width/2 car2_pos_lat_inst-car2_width/2 car2_pos_lat_inst+car2_width/2];

        % Plotting trucks
        fill(truck_1_dimension_X,truck_1_dimension_Y,c(1,:))
        fill(truck_2_dimension_X,truck_2_dimension_Y,c(2,:))
        fill(truck_3_dimension_X,truck_3_dimension_Y,c(3,:))
        fill(truck_4_dimension_X,truck_4_dimension_Y,c(4,:))
        fill(car_dimension_X,car_dimension_Y,'g')
        fill(car2_dimension_X,car2_dimension_Y,'b')
    
        xlabel('Lon. distance [m]')
        ylabel('Lat. distance [m]')
        
    frame = getframe(gcf);
    writeVideo(v,frame);
    
end

close(v);

%% Auxiliary functions

function dz = simulation(t,z,parameters)
    % PARAMETERS
    truck_1_length = parameters.truck_1_length;
    truck_2_length = parameters.truck_2_length;
    truck_3_length = parameters.truck_3_length;
    truck_4_length = parameters.truck_4_length;

    % RETRIEVING STATES
    % Truck 1
    truck_1_position    = z(1);
    truck_1_speed       = z(2);
    truck_1_states      = [truck_1_position truck_1_speed];
    % Truck 2
    truck_2_position    = z(3);
    truck_2_speed       = z(4);
    truck_2_states      = [truck_2_position truck_2_speed];
    % Truck 3
    truck_3_position    = z(5);
    truck_3_speed       = z(6);
    truck_3_states      = [truck_3_position truck_3_speed];
    % Truck 4
    truck_4_position    = z(7);
    truck_4_speed       = z(8);
    truck_4_states      = [truck_4_position truck_4_speed];
    % Car
    car_pos_lon         = z(9);
    car_speed           = z(10);
    car_states          = [car_pos_lon car_speed];
    car_pos_lat         = z(11);
    % Car 2
    car2_pos_lon        = z(12);  % Assuming this is the correct order
    car2_speed          = z(13);
    car2_states         = [car2_pos_lon car2_speed];
    car2_pos_lat        = z(14);
    
   % SENSORS
    % Truck 2
    Truck_2_sensors.distance_preceding = (truck_1_position-truck_1_length) - truck_2_position;
    Truck_2_sensors.speed_preceding = truck_1_speed;
    % Truck 3
    Truck_3_sensors.distance_preceding = (truck_2_position-truck_2_length) - truck_3_position;
    Truck_3_sensors.speed_preceding = truck_2_speed;
    % Truck 4
    Truck_4_sensors.distance_preceding = (truck_3_position-truck_3_length) - truck_4_position;
    Truck_4_sensors.speed_preceding = truck_3_speed;

    % DYNAMIC MODELS
    % Truck 1
    truck_1_derivative_states = truck_model(t,truck_1_states,1,1);
    % Truck 2
    truck_2_derivative_states = truck_model(t,truck_2_states,2,Truck_2_sensors);
    % Truck 3
    truck_3_derivative_states = truck_model(t,truck_3_states,3,Truck_3_sensors);
    % Truck 4
    truck_4_derivative_states = truck_model(t,truck_4_states,4,Truck_4_sensors);
    car_derivative_states     = car_model(t,car_states);
    car2_derivative_states    = car_model(t,car2_states);


    % OUTPUT STATES
    % Truck 1
    dz(1,1) = truck_1_derivative_states(1,1);
    dz(2,1) = truck_1_derivative_states(2,1);
    % Truck 2
    dz(3,1) = truck_2_derivative_states(1,1);
    dz(4,1) = truck_2_derivative_states(2,1);
    % Truck 3
    dz(5,1) = truck_3_derivative_states(1,1);
    dz(6,1) = truck_3_derivative_states(2,1);
    % Truck 4
    dz(7,1) = truck_4_derivative_states(1,1);
    dz(8,1) = truck_4_derivative_states(2,1);
    % Car
    dz(9,1)     = car_derivative_states(1,1);
    dz(10,1)    = car_derivative_states(2,1);
    % Car2
    dz(12,1)    = car2_derivative_states(1,1);
    dz(13,1)    = car2_derivative_states(2,1);
    % Car 1 and Car 2 lateral speed [m/s]
    dz(11,1) = 0;
    dz(14,1) = 0;

end

function dstates = truck_model(~,states,truck_flag,truck_sensors)
    % Parameters
    m   = 40000;                % Mass                      [kg]
    g   = 9.81;                 % Gravity                   [m/s2]
    Cd  = 0.78;                 % Drag coefficient          [-]
    A   = 10;                   % Frontal area              [m2]
    rho = 1.2265;               % Air density               [kg/m3]

    % States
    V = states(2);

    % Drag resistance
    C  = 0.5 * rho * Cd * A;
    Dx = C * V^2;

    % Rolling resistance (simplified, assuming coefficient of rolling resistance is included in C)
    Rx = 0;
    
    % Gravity force
    theta   = 0;                % Road slope                [rad]
    Gx      = m * g * sin(theta);   %                           [N]

    % Initialize force variable
    Ft = 0;
    
    % Logic for the leader truck
    if truck_flag == 1
        % Cruise Control (CC)
        Vr = 20;               % Reference speed           [m/s]
        Kp = 500;              % Controller gain
        Ft = Kp * (Vr - V) + Dx; % Thrust force           [N]
    else
        % Adaptive Cruise Control (ACC) for the following trucks
        th = 1.0;              % Time gap                  [s]
        desired_distance = th * V; % Desired distance from the preceding truck

        % Controller gains
        Kp = 10000;
        Kd = 15000;

        % Sensor readings
        sensor_distance_preceding = truck_sensors.distance_preceding;
        sensor_speed_preceding = truck_sensors.speed_preceding;

        % Control force to maintain safe following distance and speed
        Ft = Kp * (sensor_distance_preceding - desired_distance) + Kd * (sensor_speed_preceding - V) + Dx;
    end

    % Vehicle Dynamics
    dstates(1,1) = V;                          % Derivative of position is speed
    dstates(2,1) = (Ft - Dx - Rx - Gx) / m;    % Acceleration
    
end

function dstates = car_model(~,states)

    % Parameters
    m   = 1500;                 % Mass                      [kg]
    g   = 9.81;                 % Gravity                   [m/s2]
    Cd  = 0.4;                  % Drag coefficient          [-]
    A   = 2.5;                  % Frontal area              [m2]
    rho = 1;                    % Air density               [kg/m2]

    % States
%     X = states(1);
    V = states(2);

    % Drag resistance
    C  = 0.5*rho*Cd*A;
    Dx = C*V^2;

    % Rolling resistance
    Rx=0;
    
    % Gravity force
    theta   = 0;                % Road slope                [rad]
    Gx      = m*g*sin(theta);   %                           [N]

    % CC
    V_r = 20;                   % Reference speed           [m/s]
    Kp  = 500;                  % Controller gain
    Ft  = Kp*(V_r - V) + Dx;    % Longitudinal force        [N]

    % Vehicle Dynamics
    dstates(1,1) = V;
    dstates(2,1) = (Ft - Dx - Rx - Gx)/m;
    
end
