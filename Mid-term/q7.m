% Calculation of V with respect to the body frame

%Desired transformation as given in the question
T_sd=[sqrt(2)/2 -sqrt(2)/2 0 200/1000;
      sqrt(2)/2 sqrt(2)/2 0 200/1000;
      0 0 1 500/1000
      0 0 0 1];

%Transformation matrix from space to body frame as done 
T_sb=[0.5000    0.5000    0.7071    0.2369;
      -0.5000   -0.5000    0.7071    0.2369;
      0.7071   -0.7071         0    0.3960;
         0         0         0    1.0000];

%Transformation  from body to desired frame 
T_bd= inv(T_sb)*T_sd

%Skew symmetric matrix of V with respect to body frame
V_b_skew=logm(T_bd)

%V with respect to body
V_b=[V_b_skew(3,2) V_b_skew(1,3) V_b_skew(2,1) V_b_skew(1,4) V_b_skew(2,4) V_b_skew(3,4)]'


