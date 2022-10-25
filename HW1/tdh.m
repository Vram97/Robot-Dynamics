%Function that calculates the transformation matrix given the DH parameters
function T = tdh(alpha, a, d, phi)

    %Pre defined formula derived in lectures
    T=[cos(phi) -sin(phi) 0 a; 
       sin(phi)*cos(alpha) cos(phi)*cos(alpha) -sin(alpha) -d*sin(alpha); 
       sin(phi)*sin(alpha) cos(phi)*sin(alpha) cos(alpha) d*cos(alpha);
       0 0 0 1]
end