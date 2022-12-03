%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION

K = [311.0520,0,201.8724;0,311.3885,113.6210;0,0,1];
T_b2c = [-0.04,0.0,-0.03];
estimatedV(:,1) = zeros(6,1);
pointTracker = vision.PointTracker;

for n = 2:length(sampledData)
    %% Initalize Loop load images

    img_prev = sampledData(n-1).img;
    img_curr = sampledData(n).img;

    %% Detect good points

    corners = detectHarrisFeatures(img_prev).selectStrongest(50);
    points_prev = corners.Location;

    %% Initalize the tracker to the last frame.
    
    pointTracker.release();
    initialize(pointTracker,points_prev,img_prev);

    %% Find the location of the next points;

    points_curr = pointTracker(img_curr);

    %% Calculate velocity
    % Use a for loop

    points_curr = [points_curr,ones(50,1)]'; 
    points_prev = [points_prev,ones(50,1)]';  

    p = K\(points_curr - points_prev);

    delta_t = sampledData(n).t - sampledData(n-1).t;

    p_dot = p/delta_t; 

    optV = p_dot;
    optPos = K\points_curr;

    %% Calculate Height

    [position,orientation,R_c2w] = estimatePose(sampledData,n);
    Z = position(3)*cos(orientation(2)*cos(orientation(3)));

    %% RANSAC    
    % Write your own RANSAC implementation in the file velocityRANSAC

    e = 0.5;
    Vel = velocityRANSAC(optV,optPos,Z,R_c2w,e);

    %% Thereshold outputs into a range.
    % Not necessary
   
    %% Fix the linear velocity
    % Change the frame of the computed velocity to world frame

    angular_velocity = R_c2w' * Vel(4:6);
    linear_velocity = R_c2w' * Vel(1:3) + (cross(Vel(4:6),T_b2c))';
    Vel = vertcat(linear_velocity,angular_velocity);

    %% ADD SOME LOW PASS FILTER CODE
    % Not neceessary but recommended 

    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
    estimatedV(:,n) = Vel; % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z
end

plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
