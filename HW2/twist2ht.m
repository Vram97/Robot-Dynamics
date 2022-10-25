%Function to calculate Homogenous transformation matrix from omega, velocity and theta values
function T = twist2ht(S,theta)
    %Skew symmetric matrix from omega values
    omega_square = [0 -S(3) S(2);S(3) 0 -S(1);-S(2) S(1) 0];
    
    %Using function from Q1 to calculate Rotation matrix
    R=axisangle2rot(S(1:3),theta);
    
    %Direct formula for calculation of translation vector
    Trans=(eye(3)*theta + ((1 - cos(theta))*omega_square) + ((theta-sin(theta))*(omega_square * omega_square))) * S(4:6);
    
    %Putting everything together to form the homogenous matrix
    T=[R Trans; 0 0 0 1];

end