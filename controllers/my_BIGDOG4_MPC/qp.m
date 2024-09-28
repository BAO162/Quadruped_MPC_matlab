function x=qp(rbf,b_control,flag)

uf=0.5;
ub= [0; 0;0;0; 0;0;0; 0;0;0; 0;0];
lb= [0; 0;0;0; 0;0;0; 0;0;0; 0;0];
ubA=[0; 1000000; 0; 1000000;160;0; 1000000; 0; 1000000;160;0; 1000000; 0; 1000000;160;0; 1000000; 0; 1000000;160];%上约束
lbA=[0; 1000000; 0; 1000000;10;0; 1000000; 0; 1000000;10;0; 1000000; 0; 1000000;10;0; 1000000; 0; 1000000;10];%下约束

A_control=[flag(1)*eye(3), flag(2)*eye(3), flag(3)*eye(3), flag(4)*eye(3);
   flag(1)*Skew(rbf(:,1)),flag(2)*Skew(rbf(:,2)), flag(3)*Skew(rbf(:,3)), flag(4)*Skew(rbf(:,4))];

S= diag([1,1,10,50,30,10]);
W=0.001*eye(12);
 alpha=0.01;
H = 2*A_control'*S*A_control+2*alpha*W;
g = -2*A_control'*S*b_control;
% H = 2*A_control'*A_control;
% f = -2*b_control'*A_control;
% f=f';

c=[1,0,-uf;
   1,0,uf;
    0,1,-uf;
    0,1,uf;
    0,0,1];

O=[0,0,0;
    0,0,0;
    0,0,0;
    0,0,0;
    0,0,0];
A=[flag(1)*c,O,O,O;
    O,flag(2)*c,O,O;
    O,O,flag(3)*c,O;
    O,O,O,flag(4)*c];
for i=1:4
    for j=1:3
        
    ub(3 * (i-1) + j, 1)=flag(i)*100000;
    lb(3 * (i-1) + j, 1)=-flag(i)*100000;
    end
    lbA(5 * (i-1)+1,1)=-flag(i)*100000;
    lbA(5 * (i-1)+2,1)=0;
    lbA(5 * (i-1)+3,1)=-flag(i)*100000;
    lbA(5 * (i-1)+4,1)=0;
    lbA(5 * (i-1)+5,1)=flag(i)*10;
    
    ubA(5 * (i-1)+1,1)=0;
    ubA(5 * (i-1)+2,1)=flag(i)*100000;
    ubA(5 * (i-1)+3,1)=0;
    ubA(5 * (i-1)+4,1)=flag(i)*100000;
    ubA(5 * (i-1)+5,1)=flag(i)*160;
end

coder.extrinsic('qpOASES');
[x,fval,exitflag,iter,lambda] = qpOASES(H,g,A,lb,ub,lbA,ubA,[]);



end