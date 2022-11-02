%Function to calculate the twist about the body frame given the space frame
function V_b=twistspace2body(V_s,T)

    %Extracting the rotation matrix
    R=T(1:3,1:3);

    %Inverse transformation
    T_inv=[R' -R'*T(1:3,4);
            0 0 0 1;];
    
    %Skew symmetric matrix of position vector
    P_bracket=[0 -T_inv(3,4) T_inv(2,4); T_inv(3,4) 0 -T_inv(1,4); -T_inv(2,4) T_inv(1,4) 0];

    % [p]*R part of the formula
    bottom_term=P_bracket*R';

    % Transforming frames
    V_b=[R' zeros(3);bottom_term R']*V_s;
    
end