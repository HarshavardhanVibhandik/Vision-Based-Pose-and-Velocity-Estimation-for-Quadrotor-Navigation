function [Velocity] = velocityRANSAC(optV,optPos,Z,R_c2w,e,dt)
%% CHANGE THE NAME OF THE FUNCTION TO velocityRANSAC
    %% Input Parameter Description
    % optV = The optical Flow
    % optPos = Position of the features in the camera frame 
    % Z = Height of the drone
    % R_c2w = Rotation defining camera to world frame
    % e = RANSAC hyper parameter   
    
    M=3;
    %K(Camera Matrix) is given in parameters.txt file
    K = [311.0520 0 201.8724;
         0 311.3885 113.6210;
         0 0 1];

    best_Vel = zeros(6,1);
    best_pc =0;
    for KCount = 1:5
        A = [];
        B=[];
        p_dot= [];

        for m_count = 1:M
            idx_random = randi([1,length(optPos)],1);
            p1_temp= inv(K)*[optPos(idx_random,:)';1];
            p2_temp =inv(K)*[optV(idx_random,:)';1];
            
            x = p1_temp(1);
            y = p1_temp(2);

            p = [x;y;1];
            pw = R_c2w*p;
            cos_theta = dot(pw,[0,0,-1]);
            Z_temp = Z/cos_theta;        
            x_prev = p2_temp(1);
            y_prev = p2_temp(2);
            A = [A;(1/Z_temp)*[-1 0 x ; 0 -1 y]];
            B = [B;x*y -(1+x^2) y; (1+y^2) -x*y -x];
            p_dot = [p_dot;((x-x_prev)/dt) ; ((y-y_prev)/dt)];

        end

        H =[A,B];
        Vel_c_world_camera = pinv(H) * p_dot;
        ransac_count = 1;

        for p_count = 1:length(optPos)
            p_dot =[];

            p1_temp= inv(K)*[optPos(p_count,:)';1];
            p2_temp = inv(K)*[optV(p_count,:)';1];
            
            x = p1_temp(1);
            y = p1_temp(2);

            p = [x;y;1];
            pw = R_c2w*p;
            cos_theta = dot(pw,[0,0,-1]);
            Z_temp = Z/cos_theta;        
            x_prev = p2_temp(1);
            y_prev = p2_temp(2);
            a = [-1 0 x ; 0 -1 y];
            b = [x*y -(1+x^2) y; (1+y^2) -x*y -x];
            A = [A;(1/Z_temp)*a];
            B = [B;b];
            p_dot = [p_dot;((x-x_prev)/dt) ; ((y-y_prev)/dt)];

            if ((norm([a/Z_temp b]*Vel_c_world_camera - p_dot)) <e )
                ransac_count = ransac_count +1;
            end
        end

        if(ransac_count > best_pc)
            best_pc = ransac_count;
            best_Vel (:,:)= Vel_c_world_camera(:,:);
        end
    end
    skew_camera_body = [0 0.03 0 ; -0.03 0  0.04; 0 -0.04 0 ];
    adjoint= [R_c2w  -R_c2w*skew_camera_body ; 
                zeros(3,3) R_c2w] ;
    Velocity = adjoint * best_Vel;



    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector
    
end