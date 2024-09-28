function x=solveMpc(R_yaw,I_inv,rbf,x0,xd,dt,horizon,gait,mass)%#codegen
f_max=140;
mu=1/0.4;
alpha=0.00002;

Ac=zeros(13);
Ac(1:3,7:9)=R_yaw';
Ac(4:6,10:12)=eye(3);
Ac(12,13)=1;

I3=eye(3);
Bc=zeros(13,12);

f_block=[mu,   0,1;
        -mu,   0,1;
           0, mu,1;
           0,-mu,1;
           0,  0,1];
A=zeros(20*horizon,12*horizon);

for i=1:4*horizon
A(5*(i-1)+1:5*i,3*(i-1)+1:3*i)=f_block;
end
for i=1:4
    Bc(7:9,3*i-2:3*i)=I_inv*Skew(rbf(:,i));
    Bc(10:12,3*i-2:3*i)=I3/mass;
end

[Aqp,Bqp]=c2qp(Ac,Bc,dt,horizon);
%full_weight=[0.25;0.25;10;2;2;50;0;0;0.3;0.2;0.2;0.1;0];
%full_weight=[6;5;10;2;2;50;0;0;0.3;0.2;0.2;0.1;0];
full_weight=[25;25;10;2;2;100;0;0;0.3;10;10;20;0];%过崎岖路面，和单边桥
%full_weight=[5;8;15;5;0;120;0;0;0.3;0.5;0.1;0.5;0];
%full_weight=[25;25;10;1;1;100;0;0;0.3;0.2;0.2;20;0];

S=zeros(13*horizon);

for i=1:horizon
for j=1:13
 S((i-1)*13+j,(i-1)*13+j)=full_weight(j);
end
end

H=2*(Bqp'*S*Bqp + alpha*eye(12*horizon));
g = 2*Bqp'*S*(Aqp*x0 - xd);

lbA=zeros(20*horizon,1);
ubA=zeros(20*horizon,1);
k=1;
for i=1:horizon
    for j=1:4
    lbA(5 * (k-1)+1,1)=0;
    lbA(5 * (k-1)+2,1)=0;
    lbA(5 * (k-1)+3,1)=0;
    lbA(5 * (k-1)+4,1)=0;
    lbA(5 * (k-1)+5,1)=0;
    
    ubA(5 * (k-1)+1,1)=100000;
    ubA(5 * (k-1)+2,1)=100000;
    ubA(5 * (k-1)+3,1)=100000;
    ubA(5 * (k-1)+4,1)=100000;
    ubA(5 * (k-1)+5,1)=gait(4*(i-1)+j)*f_max;
    k=k+1;
    end   
end

 coder.extrinsic('qpOASES');
 [x,~,~,~,~] = qpOASES(H,g,A,[],[],lbA,ubA,[],[]);



end


function [A_qp,B_qp]=c2qp(A,B,dt,horizon)


ABc=zeros(25);
ABc(1:13,1:13)=A;
ABc(1:13,14:25)=B;
ABc=dt*ABc;
expmm=expm(ABc);

Adt=expmm(1:13,1:13);
Bdt=expmm(1:13,14:25);
powerMats=zeros(13,13,20);
powerMats(:,:,1)=eye(13);

for i=1:horizon
    powerMats(:,:,i+1)=Adt * powerMats(:,:,i);
end
A_qp=zeros(13*horizon,13);
B_qp=zeros(13*horizon,12*horizon);
for m=1:horizon
    A_qp(13*(m-1)+1:13*m,1:13)= powerMats(:,:,m+1);
    for n=1:horizon
        if m>=n
            a_num=m-n;
            B_qp(13*(m-1)+1:13*m,12*(n-1)+1:12*n)=powerMats(:,:,a_num+1)* Bdt;
        end
    end
end
end


