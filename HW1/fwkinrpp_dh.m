%Function to calculate the Forward kinematics for the specific RPP robot
%configuration
function T = fwkinrpp(q)
    
    %Pre-defined constants
    l1=0.5;
    d2=q(:,2);
    d3=q(:,3);

    alpha_vals=[0 pi/2 0];
    a_vals=[0 0 0];
    d_vals=[l1 d2 d3];
    phi_vals=[q(:,1) pi/2 -pi/2];

    function hom = tdh(alpha, a, d, phi)

    %Pre defined formula derived in lectures
    hom=[cos(phi) -sin(phi) 0 a; 
       sin(phi)*cos(alpha) cos(phi)*cos(alpha) -sin(alpha) -d*sin(alpha); 
       sin(phi)*sin(alpha) cos(phi)*sin(alpha) cos(alpha) d*cos(alpha);
       0 0 0 1];

    end
    
    mul=eye(4);

    for n=1:size(alpha_vals,2)
        hom_joint=tdh(alpha_vals(:,n), a_vals(:,n), d_vals(:,n), phi_vals(:,n))
        mul=mul * hom_joint;
        
    end
    
    T=mul;

end