function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)
    
    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method
    %% Parameters
    l = 0.152;
    l_special = 0.178;
    mat_xrow = 12;
    res = struct("p0",{},"p1",{},"p2",{},"p3",{},"p4",{}); 
    %% Get Corners
    for i = 1:length(id)
        x_index = rem(id(i),mat_xrow) + 1;
        y_index = fix(id(i)./mat_xrow) + 1;                         
        
        % find the number of special space
        if (y_index < 4)    
            num_l_special = 0;                      
        elseif (y_index > 3 && y_index < 7)                  
            num_l_special = 1;  
        else
            num_l_special = 2;
        end
        
        % p1 corner
        p1_x = (2 * x_index - 1) * l;
        p1_y = (2 * y_index - 2 - num_l_special) * l + num_l_special * l_special;

        % p2 corner
        p2_x = p1_x;
        p2_y = (2 * y_index - 1 - num_l_special) * l + num_l_special * l_special;
        
        % p3 corner
        p3_x = (2 * x_index - 2) * l;
        p3_y = p2_y;
        
        % p4 corner
        p4_x = p3_x;
        p4_y = p1_y;
        
        % p0 corner
        p0_x = p4_x + l/2; 
        p0_y = p4_y + l/2;

        res(i).p0 = [p0_x p0_y]; 
        res(i).p1 = [p1_x p1_y];                     
        res(i).p2 = [p2_x p2_y];
        res(i).p3 = [p3_x p3_y];
        res(i).p4 = [p4_x p4_y];
    end
end