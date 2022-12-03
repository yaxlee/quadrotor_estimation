clear; % Clear variables
addpath('../data')
datasetNum = 4; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime,proj2Data] = init(datasetNum);

Z = sampledVicon(1:6,:);
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = 0.1*eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %Just for saving state his.
prevTime = 0; %last time step in real time
pos = proj2Data.position;
pose = proj2Data.angle;
for i = 1:length(sampledTime)
    %% Fill in the FOR LOOP

    if(sampledData(i).is_ready == 1)

    curTime = sampledData(i).t;
    dt = curTime-prevTime;
    angVel = sampledData(i).omg;
    acc = sampledData(i).acc;

    z_t = [pos(i,:)';pose(i,:)'];

    [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt);
    [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);
    savedStates(:,i) = uCurr;
    prevTime = curTime;
    uPrev = uCurr;
    covarPrev = covar_curr;
    end
end

plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);