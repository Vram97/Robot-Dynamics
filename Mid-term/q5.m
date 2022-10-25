% % Calculation of singularity and manipulability

%Jacobian with respect to body frame as calculated previously
J=  [0.7071   -0.7071   -0.7071   -0.5000   -0.7071    0.0000;
    -0.7071   -0.7071   -0.7071    0.5000   -0.7071    0.0000;
         0    0.0000    0.0000    0.7071    0.0000    1.0000;
    -0.2369   -0.2369   -0.2369    0.0360   -0.0509    0.0000;
    -0.2369    0.2369    0.2369    0.0360    0.0509    0.0000;
    -0.0000    0.1060   -0.1640    0.0000    0.0000    0.0000];

%Calculation of rank of the Jacobian
Rank=rank(J)

%Calculation of manipulability

%Splitting into omega and v components of the Jacobian
J_omega=J(1:3,1:6);
J_v=J(4:6,1:6);
 
%Calculation of A matrix for both linear velocity and omega 
A_omega= J_omega * J_omega';
A_v=J_v * J_v';

%Eigen values for both As
lambda_omg= eig(A_omega)
lambda_v=eig(A_v)

%Rotational conditional number
mu_omg=max(lambda_omg,[],'all')/min(lambda_omg,[],'all')

%Linear conditional number
mu_lin=max(lambda_v,[],'all')/min(lambda_v,[],'all')