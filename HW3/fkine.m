%Function to calculate the Homogenous matrix from base frame to end-effector and vice-versa for an n joint manipulator
function F=fkine(S,M,q,frame)

    %Initializing Homogenous matrix as an identity
    prod=eye(4);
    
    %Looping through all joints for calculating forward kinematics
    for n=1:size(S,2)
        %Updating the values on the matrix
        prod=prod*twist2ht(S(:,n),q(n));
    end
    %Multiplying with the M matrix for the final answer
    T=prod*M;

    % If the answer needs to be calculated wrt space
    if strcmp(frame,'space')
        F=T;
    %If the answer needs to be calculated wrt body
    elseif strcmp(frame,'body')

        %For the body frame multiply M first
        F=M*prod;
        
    else
     
    end

end