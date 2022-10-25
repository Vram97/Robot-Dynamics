% % Calculation of first iteration of inverse kinematics

% Jacobian with respect to body frame
J_b=  [0.7071   -0.7071   -0.7071   -0.5000   -0.7071    0.0000;
      -0.7071   -0.7071   -0.7071    0.5000   -0.7071    0.0000;
         0    0.0000    0.0000    0.7071    0.0000    1.0000;
      -0.2369   -0.2369   -0.2369    0.0360   -0.0509    0.0000;
      -0.2369    0.2369    0.2369    0.0360    0.0509    0.0000;
     -0.0000    0.1060   -0.1640    0.0000    0.0000    0.0000];

% Twist with respect to body as calculated in question 7
V_b=[1.7600
    -0.7290
     1.7600
    -0.0418
    -0.1437
    0.0341];

%Initial guess for q as given in the question
theta_initial=[pi/4 0 pi/4 0 -pi/4 pi/4]';

%Newton raphson equation
theta_new=theta_initial + inv(J_b)*V_b


