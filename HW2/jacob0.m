%Function to calculate the Jacobian for a manipulator of a certain configuration 
function J=jacob0(S,q)
    %Initializing the solution with J1 since it is fixed
    ans=[S(:,1)];
    %Initializing transormation matrix so that for loop can be used
    T=1;
    
    %Looping through all the joints
    for n=2:size(S,2)
        %Calculating homogenous matrix using the function defined in Q2
        Trans=twist2ht(S(:,n-1),q(n-1))
        %Updating transformation
        T=T*Trans;
        %Extracting Rotation matrix
        R=T(1:3,1:3);
        %Skew symmetric matrix from position vector
        P_skew=[0 -T(3,4) T(2,4); T(3,4) 0 -T(1,4); -T(2,4) T(1,4) 0];
        
        % [p]*R term from equation
        bottom_term=P_skew*R;
    
        %Putting everything together to form the adjoint matrix
        adj=[R zeros(3);bottom_term R];

        %Multiplying with S and updating each column of the Jacobian
        ans=[ans adj*S(:,n)];

        
    end
    
    %Final answer
    J=ans;
    
end
    