function [Vel] = velocityRANSAC(optV,optPos,Z,R_c2w,e)
%% CHANGE THE NAME OF THE FUNCTION TO velocityRANSAC
    %% Input Parameter Description
    % optV = The optical Flow
    % optPos = Position of the features in the camera frame 
    % Z = Height of the drone
    % R_c2w = Rotation defining camera to world frame
    % e = RANSAC hyper parameter    
    
    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector


    %% RANSAC Flag
    RANSAC_Flag = 1;  

    %%
    if RANSAC_Flag == 0
        H = [];
        P_dot = [];
        for i=1:length(optV)
            x = optPos(1,i);
            y = optPos(2,i);
            H = [H;-1/Z,0,(1/Z)*x,x*y,-(1+x^2),y;0,-1/Z,(1/Z)*y,1+y^2,-x*y,-x];
            P_dot = [P_dot;optV(1:2,i)];
        end
        Vel = pinv(H) * P_dot;
    end

    %%
    if RANSAC_Flag == 1
        
        p_sucess = 0.99;
        k = round(log(1 - p_sucess)/log(1 - e^3)); 
        
        inlier_num_prev = 0;

        for i = 1:k
            points_rand = randperm(length(optPos),3);
            H = [];
            P_dot = [];
            for j = 1:3
                x = optPos(1,points_rand(j));
                y = optPos(2,points_rand(j)); 
                H = [H;-1/Z,0,(1/Z)*x,x*y,-(1+x^2),y;0,-1/Z,(1/Z)*y,1+y^2,-x*y,-x];
                P_dot = [P_dot;optV(1:2,points_rand(j))];
            end

            Vel_RANSAC = pinv(H) * P_dot;

            inlier_num_curr = 0;
            inlier_id = [];

            for m = 1:length(optPos)
                x = optPos(1,m);
                y = optPos(2,m);
                H = [-1/Z,0,(1/Z)*x,x*y,-(1+x^2),y;0,-1/Z,(1/Z)*y,1+y^2,-x*y,-x];

                p_dot = H * Vel_RANSAC;
                
                error = norm(optV(1:2,m) - p_dot);

                if error < 0.1
                    inlier_num_curr = inlier_num_curr + 1;
                    inlier_id = [inlier_id,m];
                end
            end

            if inlier_num_curr > inlier_num_prev
                inlier_num_prev = inlier_num_curr;
                inlier_ids = inlier_id;
            end
        end
        H = [];
        P_dot = [];
        for i = 1:length(inlier_ids)
                x = optPos(1,inlier_ids(i));
                y = optPos(2,inlier_ids(i));
                H = [H;-1/Z,0,(1/Z)*x,x*y,-(1+x^2),y;0,-1/Z,(1/Z)*y,1+y^2,-x*y,-x];
                P_dot = [P_dot;optV(1:2,inlier_ids(i))];
        end
            Vel = pinv(H) * P_dot;
    end

end