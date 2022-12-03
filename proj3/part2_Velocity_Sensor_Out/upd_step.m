function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state
    %% Parameters
    alpha = 0.01;
    beta = 2;
    k =2;

    R = 0.015 * eye(3);

    %% Additive Noise

    n = length(uEst); % dimensionality of x

    lambda = (alpha ^ 2) * (n + k) - n; 

    sigma_points_num = 2 * n + 1; % number of sigma points

    covarEst_sqrt = chol(covarEst,'lower');

    sigma_points(:,1) = uEst;

    % 2n+1 sigma points
    for i = 1:n
        sigma_points(:,2*i) = uEst + sqrt(n + lambda) * covarEst_sqrt(:,i);
        sigma_points(:,2*i+1) = uEst - sqrt(n + lambda) * covarEst_sqrt(:,i);
    end
    
    %% Transformation

    ang_vel = z_t(4:6); 

    T_b2c = [-0.04;0;-0.03];
    R_b2c = eul2rotm([-pi/4,0,pi]);
    R_c2b = R_b2c';

    skewTb2c = [0,-T_b2c(3),T_b2c(2);T_b2c(3),0,-T_b2c(1);-T_b2c(2),T_b2c(1),0];

    Z_t = zeros(3,sigma_points_num); 

    for i = 1:sigma_points_num

        R_b2w = eul2rotm([sigma_points(6,i),sigma_points(5,i),sigma_points(4,i)]);

        R_w2b = R_b2w';
        
        Z_t(:,i) = R_b2c * (R_w2b * sigma_points(7:9,i)) - (R_b2c * skewTb2c) * (R_c2b * ang_vel); 

    end
    
    %%

    W_0m = lambda/(n + lambda);
    W_im = 1/(2*(n + lambda));    
    W_0c = W_0m + (1 - alpha^2 + beta);
    W_ic = W_im;

    z_ut = zeros(3,1);
    for i = 1:sigma_points_num
        if i == 1
            z_ut = z_ut + W_0m * Z_t(:,i);      
        else
            z_ut = z_ut + W_im * Z_t(:,i);     
        end
    end

    Ct =zeros(15,3);
    St = zeros(3,3);
    for i = 1:sigma_points_num
        if i == 1
            Ct = Ct + W_0c * (sigma_points(:,i) - uEst) * (Z_t(:,i)- z_ut)';  
            St = St + W_0c * (Z_t(:,i)- z_ut) * (Z_t(:,i)- z_ut)';     
        else 
            Ct = Ct + W_ic * (sigma_points(:,i) - uEst) * (Z_t(:,i)- z_ut)';  
            St = St + W_ic * (Z_t(:,i)- z_ut) * (Z_t(:,i)- z_ut)'; 
        end
    end
    St = St + R;

    %%

    Kt = Ct/St;
    uCurr = uEst + Kt * (z_t(1:3) - z_ut);
    covar_curr = covarEst - Kt * St * Kt';
end