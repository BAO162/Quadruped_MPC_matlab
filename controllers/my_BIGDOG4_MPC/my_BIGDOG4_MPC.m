%如果您有用请给个star  -- "Create from Bao and Meng"
TIME_STEP = 2;
t=0;
wb_keyboard_enable(TIME_STEP);
bodyheight=0.3;


vx = 0;
vy = 0;
vz = 0.1;

d_roll=0;
d_pitch=0;
d_yaw=0;

rsf=[0.082,0.082,-0.048,-0.048;
    -0.085,0.085,-0.085,0.085;
    -0.08,-0.08,-0.08,-0.08];

rbs=[0.139,0.139,-0.139,-0.139;
         -0.061,0.061,-0.061,0.061;
              0,    0,     0,    0];
rbf=rbs+rsf;

mass=13.5;%质量
I=diag([0.06150  0.1313 0.1646]);%转动惯量
offset=[0.204,0.204,-0.204,-0.204;
             -0.146,0.146,-0.146,0.146;
             0,     0,     0,       0];
Kpcom=diag([400 400 400]);
Kdcom=diag([160 160 160]);
Kpbase=diag([1000 1000 1000]);
Kdbase=diag([40 40 40]);

dt=0.002;%一般频率下时间间隔
IterationsBetweenMpc=15;%%迭代次数在mpc之间
stancetime=0.15;
swingtime=0.15;
height=0.07;
horizon=10;
Kp_cartesian=diag([100 100 100]);
Kd_cartesian=diag([10 20 10]);


nIterations=10;
offsets=[0;5;5;0];
durations=[5;5;5;5];


q1=zeros(3,1);
q2=zeros(3,1);
q3=zeros(3,1);
q4=zeros(3,1);
XYZ=zeros(3,1);
RPY=zeros(3,1);
tao1=[0;0;0];
tao2=[0;0;0];
tao3=[0;0;0];
tao4=[0;0;0];






 %%imu，陀螺仪，gps，加速度计
gyro=wb_robot_get_device('gyro');
wb_gyro_enable(gyro,TIME_STEP);
imu=wb_robot_get_device('imu');
wb_inertial_unit_enable(imu,TIME_STEP);
gps=wb_robot_get_device('gps');
wb_gps_enable(gps,TIME_STEP);
accelerometer=wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer, TIME_STEP)

%%RGBD相机
% imagesize = imagesc(image);
% image=cell(512,512);
% RGBD=wb_robot_get_device('RGBD');
% wb_range_finder_enable(RGBD,TIME_STEP*3);
% width = wb_range_finder_get_width(RGBD);
% height = wb_range_finder_get_height(RGBD);
% image= wb_range_finder_get_range_image(RGBD);
% depth = wb_range_finder_image_get_depth(image, width, 256, 256);

%%普通相机
% camera=wb_robot_get_device('camera');
% wb_camera_enable(camera, TIME_STEP*3);
% wb_camera_recognition_enable(camera, TIME_STEP*3);
% wb_camera_recognition_enable_segmentation(camera);

% 接触传感器
 FL_TOUCH= wb_robot_get_device('FL_TOUCH');
 wb_touch_sensor_enable(FL_TOUCH, TIME_STEP);
 FR_TOUCH= wb_robot_get_device('FR_TOUCH');
 wb_touch_sensor_enable(FR_TOUCH, TIME_STEP);
 BL_TOUCH= wb_robot_get_device('BL_TOUCH');
 wb_touch_sensor_enable(BL_TOUCH, TIME_STEP);
 BR_TOUCH= wb_robot_get_device('BR_TOUCH');
 wb_touch_sensor_enable(BR_TOUCH, TIME_STEP);  
  
FL_hip_motor= wb_robot_get_device('FL_hip_motor');
	wb_motor_set_position(FL_hip_motor,inf);
	wb_motor_set_velocity(FL_hip_motor,1);
FL_leg_motor= wb_robot_get_device('FL_leg_motor');
	wb_motor_set_position(FL_leg_motor,inf);
	wb_motor_set_velocity(FL_leg_motor,1);
FL_foot_motor= wb_robot_get_device('FL_foot_motor');
	wb_motor_set_position(FL_foot_motor,inf);
	wb_motor_set_velocity(FL_foot_motor,1);
 
FR_hip_motor= wb_robot_get_device('FR_hip_motor');
	wb_motor_set_position(FR_hip_motor,inf);
	wb_motor_set_velocity(FR_hip_motor,1);
FR_leg_motor= wb_robot_get_device('FR_leg_motor');
	wb_motor_set_position(FR_leg_motor,inf);
	wb_motor_set_velocity(FR_leg_motor,1);
FR_foot_motor= wb_robot_get_device('FR_foot_motor');
	wb_motor_set_position(FR_foot_motor,inf);
	wb_motor_set_velocity(FR_foot_motor,1);
  
BL_hip_motor= wb_robot_get_device('BL_hip_motor');
	wb_motor_set_position(BL_hip_motor,inf);
	wb_motor_set_velocity(BL_hip_motor,1);
BL_leg_motor= wb_robot_get_device('BL_leg_motor');
	wb_motor_set_position(BL_leg_motor,inf);
	wb_motor_set_velocity(BL_leg_motor,1);
BL_foot_motor= wb_robot_get_device('BL_foot_motor');
	wb_motor_set_position(BL_foot_motor,inf);
	wb_motor_set_velocity(BL_foot_motor,1);
  
BR_hip_motor= wb_robot_get_device('BR_hip_motor');
	wb_motor_set_position(BR_hip_motor,inf);
	wb_motor_set_velocity(BR_hip_motor,1);
BR_leg_motor= wb_robot_get_device('BR_leg_motor');
	wb_motor_set_position(BR_leg_motor,inf);
	wb_motor_set_velocity(BR_leg_motor,1);
BR_foot_motor= wb_robot_get_device('BR_foot_motor');
	wb_motor_set_position(BR_foot_motor,inf);
	wb_motor_set_velocity(BR_foot_motor,1);
 
FL_leg_position_sensor=wb_robot_get_device('FL_leg_position_sensor');
            wb_position_sensor_enable(FL_leg_position_sensor, TIME_STEP);
FL_foot_position_sensor=wb_robot_get_device('FL_foot_position_sensor');
            wb_position_sensor_enable(FL_foot_position_sensor, TIME_STEP);
FL_hip_position_sensor=wb_robot_get_device('FL_hip_position_sensor');
            wb_position_sensor_enable(FL_hip_position_sensor, TIME_STEP);
  
FR_leg_position_sensor=wb_robot_get_device('FR_leg_position_sensor');
            wb_position_sensor_enable(FR_leg_position_sensor, TIME_STEP);
FR_foot_position_sensor=wb_robot_get_device('FR_foot_position_sensor');
            wb_position_sensor_enable(FR_foot_position_sensor, TIME_STEP);
FR_hip_position_sensor=wb_robot_get_device('FR_hip_position_sensor');
            wb_position_sensor_enable(FR_hip_position_sensor, TIME_STEP);
  
BL_leg_position_sensor=wb_robot_get_device('BL_leg_position_sensor');
            wb_position_sensor_enable(BL_leg_position_sensor, TIME_STEP);
BL_foot_position_sensor=wb_robot_get_device('BL_foot_position_sensor');
            wb_position_sensor_enable(BL_foot_position_sensor, TIME_STEP);
BL_hip_position_sensor=wb_robot_get_device('BL_hip_position_sensor');
            wb_position_sensor_enable(BL_hip_position_sensor, TIME_STEP);
  
BR_leg_position_sensor=wb_robot_get_device('BR_leg_position_sensor');
            wb_position_sensor_enable(BR_leg_position_sensor, TIME_STEP);
BR_foot_position_sensor=wb_robot_get_device('BR_foot_position_sensor');
            wb_position_sensor_enable(BR_foot_position_sensor, TIME_STEP);
BR_hip_position_sensor=wb_robot_get_device('BR_hip_position_sensor');
            wb_position_sensor_enable(BR_hip_position_sensor, TIME_STEP);
   
  wb_motor_enable_torque_feedback(FL_leg_motor, TIME_STEP);
  wb_motor_enable_torque_feedback(FL_foot_motor, TIME_STEP);
  wb_motor_enable_torque_feedback(FR_leg_motor, TIME_STEP);
  wb_motor_enable_torque_feedback(FR_foot_motor, TIME_STEP);
  wb_motor_enable_torque_feedback(BL_leg_motor, TIME_STEP);
  wb_motor_enable_torque_feedback(BL_foot_motor, TIME_STEP);
  wb_motor_enable_torque_feedback(BR_leg_motor, TIME_STEP);
  wb_motor_enable_torque_feedback(BR_foot_motor, TIME_STEP);
  wb_motor_enable_torque_feedback(FL_hip_motor, TIME_STEP);
  wb_motor_enable_torque_feedback(FR_hip_motor, TIME_STEP);
  wb_motor_enable_torque_feedback(BL_hip_motor, TIME_STEP);
  wb_motor_enable_torque_feedback(BR_hip_motor, TIME_STEP);
  

 
while wb_robot_step(TIME_STEP) ~= -1
   key = wb_keyboard_get_key();
 
   d_roll=0;
   d_pitch=0;
   d_yaw=0;

   
   if key == double('Z') || key == double('z')
      d_roll=0.25;
   elseif key == double('X') || key == double('x')
      d_roll=-0.25;
   elseif key == double('C') || key == double('c')
      d_pitch=0.25;
   elseif key == double('V') || key == double('v')
      d_pitch=-0.25;
   elseif key == double('B') || key == double('b')
      d_yaw=0.25;
   elseif key == double('N') || key == double('n')
      d_yaw=-0.25;
   end

   
  if key == double('T') || key == double('t')
     if t>5
       vx=vx+(TIME_STEP/2000);
     if vx>2
          vx=2;
     end
     end
     bodyheight=0.25;
  else 
    % 检查按键并更新 vx 和 vy
     vx = 0;
     vy = 0;
     bodyheight=0.3;
  end
   
      %  fprintf("vx=%.4f\n",vx); 
    if key == double('W') || key == double('w')
        vx = 0.5;
    elseif key == double('S') || key == double('s')
        vx = -0.3;
    elseif key == double('A') || key == double('a')
        vy = 0.3;
    elseif key == double('D') || key == double('d')
        vy = -0.3;
    end
   
   % 检查按键并更新步态参数
    if key == double('U') || key == double('u')
        % 切换到 Trotting 步态
        offsets = [0; 5; 5; 0];
        durations = [5; 5; 5; 5];
       
        
    elseif key == double('I') || key == double('i')
        % 切换到 Bounding 步态
        offsets = [5; 5; 0; 0];
        durations = [4; 4; 4; 4];
       
        
    elseif key == double('O') || key == double('o')
        % 切换到 Pronking 步态
        offsets = [0; 0; 0; 0];
        durations = [4; 4; 4; 4];
      
        
    elseif key == double('P') || key == double('p')
        % 切换到 Galloping 步态
        offsets = [0; 2; 7; 9];
        durations = [4; 4; 4; 4];
      
        
    elseif key == double('J') || key == double('j')
        % 切换到 Standing 步态
        offsets = [0; 0; 0; 0];
        durations = [10; 10; 10; 10];
      
        
    elseif key == double('K') || key == double('k')
        % 切换到 Trot Running 步态
        offsets = [0; 5; 5; 0];
        durations = [4; 4; 4; 4];
     
        
    elseif key == double('L') || key == double('l')
        % 切换到 Walking 步态
        offsets = [5; 0; 5; 0];
        durations = [5; 5; 5; 5];
      
    end
    
    
   q1_u=q1;
   q2_u=q2;
   q3_u=q3;
   q4_u=q4;
   XYZ_F=XYZ;
   RPY_F=RPY;
 
   
   
   
   omega=wb_gyro_get_values(gyro); 
   RPY=wb_inertial_unit_get_roll_pitch_yaw(imu);
   XYZ=wb_gps_get_values(gps);

   
   FR_T=wb_touch_sensor_get_value(FR_TOUCH);   
   FL_T=wb_touch_sensor_get_value(FL_TOUCH);
   BR_T=wb_touch_sensor_get_value(BR_TOUCH);
   BL_T=wb_touch_sensor_get_value(BL_TOUCH);
   foot_senser=[FR_T;FL_T;BR_T;BL_T];

   R_z=[cos(RPY(3)),-sin(RPY(3)),0;sin(RPY(3)),cos(RPY(3)),0;0,0,1];
   R_y=[cos(RPY(2)),0,sin(RPY(2));0,1,0;-sin(RPY(2)),0,cos(RPY(2))];
   R_x=[1,0,0;0,cos(RPY(1)),-sin(RPY(1));0,sin(RPY(1)),cos(RPY(1))];

   R=(R_z*R_y*R_x);
 
  
   x=XYZ(1);
   y=XYZ(2);
   z=XYZ(3);
  
   w=[omega(1);omega(2);omega(3)];


   FR_hip_positionsensor0=wb_position_sensor_get_value(FR_hip_position_sensor);    
   FR_leg_positionsensor0=wb_position_sensor_get_value(FR_leg_position_sensor);
   FR_foot_positionsensor0=wb_position_sensor_get_value(FR_foot_position_sensor); 

FL_hip_positionsensor0=wb_position_sensor_get_value(FL_hip_position_sensor);     
FL_leg_positionsensor0=wb_position_sensor_get_value(FL_leg_position_sensor);
FL_foot_positionsensor0=wb_position_sensor_get_value(FL_foot_position_sensor); 


BR_hip_positionsensor0=wb_position_sensor_get_value(BR_hip_position_sensor);  
BR_leg_positionsensor0=wb_position_sensor_get_value(BR_leg_position_sensor);
BR_foot_positionsensor0=wb_position_sensor_get_value(BR_foot_position_sensor);  

BL_hip_positionsensor0=wb_position_sensor_get_value(BL_hip_position_sensor); 
BL_leg_positionsensor0=wb_position_sensor_get_value(BL_leg_position_sensor);
BL_foot_positionsensor0=wb_position_sensor_get_value(BL_foot_position_sensor);

q1(1)=FR_hip_positionsensor0;%趴姿态为0位
q1(2)=FR_leg_positionsensor0+0.45;
q1(3)=FR_foot_positionsensor0-1.40;

q2(1)=FL_hip_positionsensor0;
q2(2)=FL_leg_positionsensor0+0.45;
q2(3)=FL_foot_positionsensor0-1.40;

q3(1)=BR_hip_positionsensor0;
q3(2)=BR_leg_positionsensor0+0.45;
q3(3)=BR_foot_positionsensor0-1.40;

q4(1)=BL_hip_positionsensor0;
q4(2)=BL_leg_positionsensor0+0.45;
q4(3)=BL_foot_positionsensor0-1.40;


w1=(q1-q1_u)/(TIME_STEP/1000);
w2=(q2-q2_u)/(TIME_STEP/1000);
w3=(q3-q3_u)/(TIME_STEP/1000);
w4=(q4-q4_u)/(TIME_STEP/1000);



v=[(XYZ(1)-XYZ_F(1))/(TIME_STEP/1000);
   (XYZ(2)-XYZ_F(2))/(TIME_STEP/1000);
   (XYZ(3)-XYZ_F(3))/(TIME_STEP/1000)];


 if t<3
 wb_motor_set_position(FL_hip_motor,0);
 wb_motor_set_position(FL_leg_motor,-0.45);
 wb_motor_set_position(FL_foot_motor,1.4);
 wb_motor_set_position(FR_hip_motor,0);
 wb_motor_set_position(FR_leg_motor,-0.45);
 wb_motor_set_position(FR_foot_motor,1.4);
 wb_motor_set_position(BL_hip_motor,0);
 wb_motor_set_position(BL_leg_motor,-0.45);
 wb_motor_set_position(BL_foot_motor,1.4);
 wb_motor_set_position(BR_hip_motor,0);
 wb_motor_set_position(BR_leg_motor,-0.45);
 wb_motor_set_position(BR_foot_motor,1.4); 

 wb_motor_set_velocity(FL_hip_motor,1);
 wb_motor_set_velocity(FL_leg_motor,1);
 wb_motor_set_velocity(FL_foot_motor,1);
 wb_motor_set_velocity(FR_hip_motor,1);
 wb_motor_set_velocity(FR_leg_motor,1);
 wb_motor_set_velocity(FR_foot_motor,1);
 wb_motor_set_velocity(BL_hip_motor,1);
 wb_motor_set_velocity(BL_leg_motor,1);
 wb_motor_set_velocity(BL_foot_motor,1);
 wb_motor_set_velocity(BR_hip_motor,1);
 wb_motor_set_velocity(BR_leg_motor,1);
 wb_motor_set_velocity(BR_foot_motor,1);
 else
  

  %tic%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[tao1, tao2, tao3, tao4] =mpcController(R, w, x, y, z, v, q1, q2, q3, q4, w1, w2, w3, w4,t,foot_senser,vx,vy,offsets,durations,bodyheight,d_roll,d_pitch,d_yaw, ...
    rbs,mass,I,offset,Kpcom,Kdcom,Kpbase,Kdbase,dt,IterationsBetweenMpc,stancetime,swingtime,height,horizon,Kp_cartesian,Kd_cartesian,vz,nIterations);
  %toc%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tao1 = max(min(tao1, 30), -30);
tao2 = max(min(tao2, 30), -30);
tao3 = max(min(tao3, 30), -30);
tao4 = max(min(tao4, 30), -30);

         
         wb_motor_set_torque(FR_hip_motor,tao1(1));
         wb_motor_set_torque(FR_leg_motor,tao1(2)); 
         wb_motor_set_torque(FR_foot_motor,tao1(3));
       

         wb_motor_set_torque(FL_hip_motor,tao2(1));
         wb_motor_set_torque(FL_leg_motor,tao2(2));
         wb_motor_set_torque(FL_foot_motor,tao2(3));
       

         wb_motor_set_torque(BR_hip_motor,tao3(1));
         wb_motor_set_torque(BR_leg_motor,tao3(2));
         wb_motor_set_torque(BR_foot_motor,tao3(3));
       

         wb_motor_set_torque(BL_hip_motor,tao4(1));
         wb_motor_set_torque(BL_leg_motor,tao4(2));
         wb_motor_set_torque(BL_foot_motor,tao4(3));

    
 end

 t =t+TIME_STEP / 1000.0;
 
  drawnow;
end


