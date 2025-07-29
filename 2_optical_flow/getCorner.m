function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)

    %To calculate the row and column based on its ID
    row = mod(id,12);
    column = floor(id/12);
    
    for i = 0:column
        if column <= 2 %To check if the column is less than or equal to 2 
            p4 = [row*2*0.152, column*2*0.152];
        elseif column <= 5 %To check if the column is less than or equal to 5
            p4 = [row*2*0.152, 0.178 + (2*column-1)*0.152]; 
        else %To check if the column is greater than 5
            p4 = [row*2*0.152, 0.178*2 + (2*column-2)*0.152]
        end 
        p3 = [p4(1), 0.152 + p4(2)]; %To calculate the coordinates of p3 bsaed on the position of p4
        p2 = [p3(1) + 0.152, p3(2)]; %To calculate the coordinates of p2 bsaed on the position of p4
        p1 = [p4(1) + 0.152, p4(2)]; %To calculate the coordinates of p1 bsaed on the position of p4
        p0 = [(p4(1) + p1(1))/2 , (p4(2) + p3(2))/2];  % To calculate the coordinates of the center p0 based on the average of other positions
    end
        res = [p0; p1; p2; p3; p4];%To construct the variable 'res' which has the corners coordinates 
    
    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method
end