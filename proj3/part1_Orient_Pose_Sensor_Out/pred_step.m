function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
%% BEFORE RUNNING THE CODE CHANGE NAME TO pred_step
    %% Parameter Definition
    % uPrev - is the mean of the prev state
    %covarPrev - covar of the prev state
    %angVel - angular velocity input at the time step
    %acc - acceleration at the timCurrep
    %dt - difference in time

    %% Parameters
    alpha = 0.01;
    beta = 2;
    k =2;

    Q = 0.001 * eye(12);

    uEst = zeros(15,1);
    covarEst = zeros(15,15);
  
    %% Non Additive Noise

    n = length(uPrev); % dimensionality of x
    n_q = 12; % dimensionality of noise
    n = n + n_q; 

    lambda = (alpha ^ 2) * (n + k) - n; 

    sigma_points_num = 2 * n + 1; % number of sigma points

    uPrev_aug = vertcat(uPrev,zeros(12,1));

    covarPrev_aug = horzcat(vertcat(covarPrev,zeros(12,15)),vertcat(zeros(15,12),Q));

    covarPrev_aug_sqrt = chol(covarPrev_aug,'lower');

    sigma_pointsPrev = zeros(length(uPrev_aug),sigma_points_num);

    sigma_pointsPrev(:,1) = uPrev_aug;

    % 2n+1 sigma points
    for i = 1:n
        sigma_pointsPrev(:,2*i) = uPrev_aug + sqrt(n + lambda) * covarPrev_aug_sqrt(:,i);
        sigma_pointsPrev(:,2*i+1) = uPrev_aug - sqrt(n + lambda) * covarPrev_aug_sqrt(:,i);
    end

    sigma_pointsCurr = zeros(length(uPrev),sigma_points_num);
    
    for i = 1:sigma_points_num

        x = sigma_pointsPrev(1:15,i); 
        q_noise = sigma_pointsPrev(16:end,i);

        p_dot = x(7:9); % velocity
        b_g = x(10:12); % gyroscope bias
        b_a = x(13:15); % accelerometer bias

        n_g = q_noise(1:3);
        n_a = q_noise(4:6);
        n_bg = q_noise(7:9);
        n_ba = q_noise(10:12);

        G = [1 0 -sin(x(5));0 cos(x(4)) cos(x(5))*sin(x(4));0 -sin(x(4)) cos(x(5))*cos(x(4))];

        eul = [x(6) x(5) x(4)];
        R = eul2rotm(eul);

        g = [0;0;-9.81];

        x_dot = [p_dot;G\(angVel - b_g - n_g);g + R*(acc - b_a - n_a);n_bg;n_ba];

        sigma_pointsCurr(:,i) = x + dt * x_dot;
    end
    
    W_0m = lambda/(n + lambda);
    W_im = 1/(2*(n + lambda));    
    W_0c = W_0m + (1 - alpha^2 + beta);
    W_ic = W_im;

    for i = 1:sigma_points_num      
        if i == 1
            uEst = uEst + W_0m * sigma_pointsCurr(:,i);
        else
            uEst = uEst + W_im * sigma_pointsCurr(:,i);
        end
    end
    
    for i = 1:sigma_points_num
        if i == 1
        covarEst = covarEst + W_0c * (sigma_pointsCurr(:,i) - uEst) * (sigma_pointsCurr(:,i) - uEst)'; 
        else 
        covarEst = covarEst + W_ic * (sigma_pointsCurr(:,i) - uEst) * (sigma_pointsCurr(:,i) - uEst)'; 
        end
    end
end

