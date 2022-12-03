clear; % Clear variables
datasetNum = 1; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime] = init(datasetNum);
Z = sampledVicon(1:6,:);%all the measurements that you need for the update
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %J ust for saving state his.
prevTime = 0; %last time step in real time
%write your code here calling the pred_step.m and upd_step.m functions
for i = 1:length(sampledTime)
    if(sampledData(i).is_ready == 1)
    curTime = sampledData(i).t;
    dt = curTime-prevTime;
    angVel = sampledData(i).omg;
    acc = sampledData(i).acc;
    z_t = Z(:,i);
    [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt);
    [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);
    savedStates(:,i) = uCurr;
    prevTime = curTime;
    uPrev = uCurr;
    covarPrev = covar_curr;
    end
end
plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);