function [tao1, tao2, tao3, tao4] = mpcController(R, w, x, y, z, v, q1, q2, q3, q4, w1, w2, w3, w4,  t,foot_senser,vx,vy,offsets,durations,bodyheight,d_roll,d_pitch,d_yaw, ...
    rbs,mass,I,offset,Kpcom,Kdcom,Kpbase,Kdbase,dt,IterationsBetweenMpc,stancetime,swingtime,height,horizon,Kp_cartesian,Kd_cartesian,vz,nIterations)

persistent wf;
persistent x_s;
persistent x_desire;
persistent f;
persistent state;
persistent timer;
persistent iterationcounter;
persistent x0;
persistent y0;
persistent z0;
persistent v0;
persistent firstswing;
persistent pf_init;
persistent pf_final;
persistent swingTimeRemaining;

if isempty(state)
    state = 0;
end
if isempty(x0)
    x0 = 0;
end
if isempty(y0)
    y0 = 0;
end
if isempty(z0)
    z0 = 0;
end
if isempty(v0)
    v0 = 0;
end
if isempty(iterationcounter)
    iterationcounter = 0;
end
if isempty(x_s)
    x_s =zeros(13,1);
end
if isempty(x_desire)
    x_desire =zeros(13*horizon,1);
end
if isempty(swingTimeRemaining)
    swingTimeRemaining =zeros(4,1);
end
if isempty(pf_init)
    pf_init =zeros(3,4);
end
if isempty(pf_final)
    pf_final =zeros(3,4);
end
if isempty(f)
    f=zeros(12*horizon);
end
if isempty(firstswing)
    firstswing = [0;0;0;0];
end
if isempty(wf)
    wf = [0;0;0];
end

if isempty(timer)
    timer = 0;
end

trajAll=zeros(12*horizon);
tao=[0,0,0,0;0,0,0,0;0,0,0,0];
Fex=[0,0,0,0;0,0,0,0;0,0,0,0];
p=zeros(3,4);
pv=zeros(3,4);
pa=zeros(3,4);
rsf_des=zeros(3,4);
vleg_des=zeros(3,4);
v_leg=zeros(3,4);
w_leg=[w1,w2,w3,w4];

dtMpc=dt*IterationsBetweenMpc;  
offsetsFloat=offsets/nIterations;
durationsFloat=durations/nIterations;

roll=atan2(R(3,2),R(3,3));%验算一下
pitch=atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
yaw=atan2(R(2,1),R(1,1));

R_yaw=[cos(yaw) , -sin(yaw),  0; 
        sin(yaw),  cos(yaw),  0; 
               0,        0 ,  1];
I_world=R*I*R';
I_inv=inv(I_world);



if t>3&&t<5
    v_ref=[0;0;vz];
    r_ref=[0;0;0.1+vz*(t-3)];
else
    if t==5
        x0=x;
        y0=y;
        z0=z;
    end
    v_ref=[vx;vy;0];
    r_ref=[x0;y0;bodyheight];
end
    q_ref=[d_roll;d_pitch;d_yaw];
    w_ref=[0;0;0];
r=[x;y;z];
a=Kpcom*(r_ref-r)+Kdcom*(v_ref-v);
wd=[0;0;0];
qw=matrixLogRot(R');
aw=Kpbase*qw+Kdbase*(wd-w);
F=mass*(a+[0;0;9.81]);
Tao=I_world*aw;
b_control=[F;Tao];

q=[q1;q2;q3;q4];

[rsf_body,rbf_body]=forwardKinematics(q);
rbf_world=R*rbf_body;
J=JacobianMatrix(q);
if state==0
    flag=[1;1;1;1];
    timer = timer + 1;
    


    %QP优化
    f=qp(rbf_world,b_control,flag);
    
    for i=1:4
        tao(:,i)=-J(:,i*3-2:i*3)'*R'*f(i*3-2:i*3,1);
    end
    if timer>1000 %站立过程给定2s,除以计算周期dt
        state=1;
        timer=0;
        firstswing=[0;1;1;0];
    end
 
elseif state==1
    for i=1:4
        if firstswing(i)==1
            swingTimeRemaining(i)=swingtime;
            pf_init(:,i)=[x;y;z]+rbf_world(:,i);
            pf_init(3,i)=0.015; 
            wf=0.5*stancetime*[v(1);v(2);0];
            if wf(1)>0.35
                 wf(1)=0.35;
            end
        else
            swingTimeRemaining(i)=swingTimeRemaining(i)-dt;
        end
        if  swingTimeRemaining(i) < 0
            swingTimeRemaining(i)=0;
        end
        pf_final(:,i)=[x;y;z]+v_ref*swingTimeRemaining(i)+R*offset(:,i)+wf;
        pf_final(3,i)=0.015;
    end
   
    [iteration,phase]=setIterations(nIterations,iterationcounter,IterationsBetweenMpc);
    
    swingstate=getSwingState(phase,offsetsFloat,durationsFloat);

    [mpctable]=getMpcTable(iteration,nIterations,offsets,durations);    
    
    if mod(iterationcounter,IterationsBetweenMpc)==0
 
        for i=1 :horizon
             trajinit=[q_ref;x;y;bodyheight;w_ref;v_ref];
            for j=1:12
                trajAll(12*(i-1)+j)=trajinit(j);
                if i==1
                    trajAll(12*(i-1)+4)=trajinit(4)+dtMpc*v_ref(1);
                    trajAll(12*(i-1)+5)=trajinit(5)+dtMpc*v_ref(2);
                    trajAll(12*(i-1)+6)=trajinit(6)+dtMpc*v_ref(3);
                    
                else
                   
                    trajAll(12*(i-1)+4)=trajAll(12*(i-2)+4)+dtMpc*v_ref(1);
                    trajAll(12*(i-1)+5)=trajAll(12*(i-2)+5)+dtMpc*v_ref(2);
                    trajAll(12*(i-1)+6)=trajAll(12*(i-2)+6)+dtMpc*v_ref(3);
                end
            end
        end  
        x_s=[roll;pitch;yaw;x;y;z;w;v;-9.8];
        for i=1 :horizon
            for j=1:12
                x_desire(13*(i-1)+j)=trajAll(12*(i-1)+j);

            end
            x_desire(13*(i-1)+13)=-9.8;
        end 
        f=solveMpc(R_yaw,I_inv,rbf_world,x_s,x_desire,dtMpc,horizon,mpctable,mass);
    end
    iterationcounter=iterationcounter+1;

    for i=1:4
        if swingstate(i)>0
            if firstswing(i)==1
                firstswing(i)=0;
            end
            [p(:,i),pv(:,i),pa(:,i)]=SwingTrajectoryBezier(pf_init(:,i),pf_final(:,i),swingstate(i),swingtime,height);
            rsf_des(:,i)=R'*(p(:,i)-[x;y;z])-rbs(:,i);
            vleg_des(:,i)=R'*(pv(:,i)-v);

            v_leg(:,i)=J(:,i*3-2:i*3)*w_leg(:,i);

            Fex(:,i)=Kp_cartesian*(rsf_des(:,i)-rsf_body(:,i))+Kd_cartesian*(vleg_des(:,i)- v_leg(:,i));
            tao(:,i)=J(:,i*3-2:i*3)'*Fex(:,i);  
        elseif foot_senser(i)==1
            firstswing(i)=1;
            tao(:,i)=-J(:,i*3-2:i*3)'*R'*f(i*3-2:i*3,1);
        else
            firstswing(i)=1;
            tao(:,i)=-J(:,i*3-2:i*3)'*R'*f(i*3-2:i*3,1);
        end

    end

end
tao1=tao(:,1);
tao2=tao(:,2);
tao3=tao(:,3);
tao4=tao(:,4);


end



