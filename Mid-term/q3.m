% % Calculation of Jacobian with respect to the body frame

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initializing the given lengths
l0=165/1000;
l1=125/1000;
l2=270/1000;
l3=70/1000;
l4=134/1000;
l5=168/1000;
l6=72/1000;

%Initializing the given configuration
q=[pi/4 0 pi/4 0 -pi/4 pi/4];

%T_sb calculated from question 2
T_sb=[0.5000    0.5000    0.7071    0.2369;
      -0.5000   -0.5000    0.7071    0.2369;
      0.7071   -0.7071         0    0.3960;
         0         0         0    1.0000];
T_bs=inv(T_sb);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Calculated values for omega, v and M as per POE

omega_1=[0 0 1]';
omega_2=[0 1 0]';
omega_3=[0 1 0]';
omega_4=[1 0 0]';
omega_5=[0 1 0]';
omega_6=[1 0 0]';

v1=-[0 0 0]';
v2=-[l0+l1 0 0]';
v3=-[l0 + l1 + l2 0 0]';
v4=-[0 -(l0+l1+l2+l3) 0]';
v5=-[l0+l1+l2+l3 0 -(l4 + l5)]';
v6=-[0 -(l0+l1+l2+l3) 0]';

M=[0 0 1 l4+l5+l6;
    0 -1 0 0;
    1 0 0 l0+l1+l2+l3;
    0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Putting all the values together
omega=[omega_1 omega_2 omega_3  omega_4 omega_5 omega_6]';
v=[v1 v2 v3 v4 v5 v6];

%Skew symmetric matrix of translation vector from T_sb
skew_p=[0 -T_bs(3,4) T_bs(2,4); T_bs(3,4) 0 -T_bs(1,4); -T_bs(2,4) T_bs(1,4) 0];

%Adjoint of T_sb
ad=[T_bs(1:3,1:3) zeros(3);
    skew_p*T_bs(1:3,1:3) T_bs(1:3,1:3)]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Initializing the Homogenous transformation matrix
mul=eye(4);

%Twist of home configuration
S=[omega';
    v];

%Initializing the Jacobian matrix
ans=[S(:,1)];

%Looping over all joints
for n=1:size(omega,1)

    %Omega skew matrix calculation based on omega for the particular joint
    omega_square=[0 -omega(n,3) omega(n,2); omega(n,3) 0 -omega(n,1); -omega(n,2) omega(n,1) 0];

    %Rotational matrix calculation
    rot=eye(3)+sin(q(:,n))*omega_square + (1 - cos(q(:,n)))*(omega_square*omega_square);
    
    %Translational matrix calculation
    trans=(eye(3)*q(:,n) + (1 - cos(q(:,n)))*omega_square + (q(:,n)-sin(q(:,n)))*(omega_square * omega_square)) * v(:,n);
    
    %Formation of homogenous matrix
    hom= [rot(1,:) trans(1,:);
          rot(2,:)  trans(2,:);
          rot(3,:)  trans(3,:);
          0 0 0 1];
    
    %Updating POE
    mul=mul*hom;
    
    %Calculation of the latest adjoint matrices for the calculation of
    %Jacobian
    if n<size(omega,1)

        %Skew symmetric matrix of the translation vector of the latest
        %transformation matrix
        P_skew=[0 -mul(3,4) mul(2,4); mul(3,4) 0 -mul(1,4); -mul(2,4) mul(1,4) 0];

        %Rotation matrix
        R=[mul(1:3,1:3)];

        %Adjoint matrix
        adj=[R zeros(3);
            P_skew*R R];

        %Updating Jacobian
        ans=[ans adj*S(:,n+1)];
    end
end

%Transforming Jacobian from space frame to body frame 
J_b=ad*ans

