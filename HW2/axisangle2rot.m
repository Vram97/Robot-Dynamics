%Q1

function rot=axisangle2rot(omega,theta)
    
    omega_square=[0 -omega(1,3) omega(1,2); omega(1,3) 0 -omega(1,1); -omega(1,2) +omega(1,1) 0];
    rot=eye(3)+sin(theta).*omega_square + cos(theta).*(omega_square*omega_square);
end
% 
% omega=[0 0 1];
% theta=pi/2;
% omega(3)
% ans=axisangle2rot(omega,theta)