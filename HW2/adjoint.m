%Function to calculate the velocity about a different axis given the Transformation matrix
function V_new=adjoint(V,T)
    %Extracting the rotation matrix
    R=T(1:3,1:3);
    
    %Skew symmetric matrix of position vector
    P_bracket=[0 -T(3,4) T(2,4); T(3,4) 0 -T(1,4); -T(2,4) T(1,4) 0];

    % [p]*R part of the formula
    bottom_term=P_bracket*R;

    % Transforming frames
    V_new=[R zeros(3);bottom_term R]*V;
    
end