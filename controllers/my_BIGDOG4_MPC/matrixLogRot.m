function omega = matrixLogRot(R)
tmp = (R(1, 1) + R(2, 2) + R(3, 3) - 1) / 2;
    if (tmp >= 1) 
        theta = 0;  
    elseif tmp <= -1
        theta = 3.1415926;  
    else 
        theta = acos(tmp);     
    end
   omega =[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
    if (theta > 10e-5) 
        omega = omega*theta / (2 * sin(theta));
    else 
        omega = omega/ 2;
    end
   
end