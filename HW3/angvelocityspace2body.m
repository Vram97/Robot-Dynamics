%%Function to calculate omega in body frame
function omega_b=angvelocityspace2body(omega_s,R)
    %This is the same as adjoint excluding the bottom 3*6 terms
    omega_b=R'*omega_s;
end