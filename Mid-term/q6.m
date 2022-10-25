% % Calculation of torques on all joints of the manipulator

%This was calculated this way since the direction of the tip force changes
%with the body frame
F_tip=[0; 0; 0;-100; 0; 0;];

%Jacobian with respect to body frame
J=  [0.7071   -0.7071   -0.7071   -0.5000   -0.7071    0.0000;
    -0.7071   -0.7071   -0.7071    0.5000   -0.7071    0.0000;
         0    0.0000    0.0000    0.7071    0.0000    1.0000;
    -0.2369   -0.2369   -0.2369    0.0360   -0.0509    0.0000;
    -0.2369    0.2369    0.2369    0.0360    0.0509    0.0000;
    -0.0000    0.1060   -0.1640    0.0000    0.0000    0.0000];


%Torques for each joint
Tau= J' * F_tip