%Function to calculate rotation matrices
function T=trot(theta,axis)

    %Rotation matrix about x axis
    if axis=='x'
        T=[1 0 0;0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
    
    %Rotation matrix about y axis
    elseif axis=='y'
        T=[cos(theta) 0 sin(theta);0 1 0; -sin(theta) 0 cos(theta)];

    %Rotation matrix about z axis 
    elseif axis=='z'
        T=[cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
    
    else
        %If the user types an axis other than xyz
        disp('Invalid axis encountered')
    
    end
end
 
