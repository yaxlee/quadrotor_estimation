function [position, orientation, R_c2w] = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    
    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order XYZ
    
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order XYZ
    
    %R_c2w = Rotation which defines camera to world frame
    %%
    ids = data(t).id;
    len = length(ids);
    res = getCorner(ids);

    p1_data = data(t).p1;
    x1_data = p1_data(1,:);
    y1_data = p1_data(2,:);

    p2_data = data(t).p2;
    x2_data = p2_data(1,:);
    y2_data = p2_data(2,:);

    p3_data = data(t).p3;
    x3_data = p3_data(1,:);
    y3_data = p3_data(2,:); 

    p4_data = data(t).p4;
    x4_data = p4_data(1,:);
    y4_data = p4_data(2,:);
    
    x_data = vertcat(x1_data,x2_data,x3_data,x4_data);
    y_data = vertcat(y1_data,y2_data,y3_data,y4_data);

    x_data = (x_data)';
    y_data = (y_data)';

    for i = 1:len
        x1(i) = res(i).p1(1);
        x2(i) = res(i).p2(1);
        x3(i) = res(i).p3(1);
        x4(i) = res(i).p4(1);
        y1(i) = res(i).p1(2);
        y2(i) = res(i).p2(2);
        y3(i) = res(i).p3(2);
        y4(i) = res(i).p4(2);
    end

    x = vertcat(x1,x2,x3,x4);
    y = vertcat(y1,y2,y3,y4);
    x = x';
    y = y';
    
    A = [];

    for i = 1:len
        for j = 1:4
            xi = x(i,j);
            yi = y(i,j);
            xi_data = x_data(i,j);
            yi_data = y_data(i,j);
            row1 = [xi, yi, 1, 0, 0, 0, -(xi_data)*xi, -(xi_data)*yi, -xi_data];
            row2 = [0, 0, 0, xi, yi, 1, -(yi_data)*xi, -(yi_data)*yi, -yi_data];
            A = vertcat(A,row1,row2);
        end
    end

    [~,~,V] = svd(A);


    H = ([V(1,9),V(2,9),V(3,9);V(4,9),V(5,9),V(6,9);V(7,9),V(8,9),V(9,9)]) * sign(V(9,9));
    K = [311.0520,0,201.8724;0,311.3885,113.6210;0,0,1];
    K_invH = K\H;
    R1 = K_invH(:,1);
    R2 = K_invH(:,2);
    R3 = cross(R1,R2);
    R = horzcat(R1,R2,R3);
    [U,~,V] = svd(R);
    R = U * [1,0,0;0,1,0;0,0,det(U * (V)')] * (V)';

    T = K_invH(:,3)/norm(R1);

    H_w2c = vertcat(horzcat(R,T),[0,0,0,1]);
    H_c2w = vertcat(horzcat((R)',-(R).'*T),[0,0,0,1]);

    T_b2c = [-0.04;0;-0.03];
    R_b2c = eul2rotm([-pi/4,0,pi]);
    H_b2c = vertcat(horzcat(R_b2c,T_b2c),[0,0,0,1]);

    H_b2w = H_c2w * H_b2c;
    R_b2w = H_b2w(1:3,1:3);
    T_b2w = H_b2w(1:3,end);

    position = T_b2w;
    orientation = rotm2eul(R_b2w);
    R_c2w = H_c2w(1:3,1:3);
end