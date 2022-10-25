% %Calculation of analytical Jacobian

%The velocity components of the body Jacobian calulated from question 3 
J_vb=[-0.2369   -0.2369   -0.2369    0.0360   -0.0509    0.0000
     -0.2369    0.2369    0.2369    0.0360    0.0509    0.0000
     -0.0000    0.1060   -0.1640    0.0000    0.0000    0.0000];

%Rotation matrix from space to body frame
R_sb=  [0.5000    0.5000    0.7071;
       -0.5000   -0.5000    0.7071;
        0.7071   -0.7071         0];

% Analytical Jacobian
J_anytic=R_sb *J_vb

  % -0.2369    0.0750   -0.1160    0.0360         0         0
   % 0.2369    0.0750   -0.1160   -0.0360         0         0
   %-0.0000   -0.3350   -0.3350   -0.0000   -0.0720         0