%%Function to calculate the velocity about a different axis given the Transformation matrix
function adj=adjoint(T)

    %Calculating the rotation matrix from the homogenous matrix
    R=T(1:3,1:3);

    %Skew symmetric matrix of the translation vector
    P_skew=[0 -T(3,4) T(2,4); T(3,4) 0 -T(1,4); -T(2,4) T(1,4) 0];

    %Calculating the bottom term of the adjoint matrix
    bottom_term=P_skew*R;
    
    %Calculating the adjoint matrix
    adj=[R zeros(3);
         bottom_term R];
end