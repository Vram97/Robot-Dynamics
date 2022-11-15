%%Function to calculate the adjoint matrix from the twist
function adj=ad(V1)

    %Calculation of rotation matrix required to form the adjoint
    skew_omega=[0 -V1(3,1) V1(2,1); V1(3,1) 0 -V1(1,1); -V1(2,1) V1(1,1) 0];

    %Bottom term of adjoint
    bottom_term=[0 -V1(6,1) V1(5,1); V1(6,1) 0 -V1(4,1); -V1(5,1) V1(4,1) 0]*skew_omega;

    %Adjoint calculation
    adj=[skew_omega zeros(3);
         bottom_term skew_omega];
    
end