
%caso 2 3, se analiza la tray deseada y se realiza la transsformacion para la posicion del robot, considerando la log del intrumento
%caso 2 2 horizontal coloca la trayectoria paralela horizontal al robot

% caso 2 v1

% se cuenta con una herramienta que sostiene la aguja, ahora no se
% considera la rotación de la trayectoria solo el cambio de posicion  y se
% conserva la trayectoria del robot

%v4 traslacion y rotacion desde la 
% trayectoria normal al plano de la aguja
% v3 1 la funcion de ruta md_circculo_v3 entrega el plano normal 
% v3 los puntos se calculan cerca de la posicion inicial y se considera el
% error de orientacion 

%v2 se le pasan los puntos y se mueve el robot pero con una direccion
%determinada

%  ayuda para acoplar a robot real https://la.mathworks.com/matlabcentral/answers/1897750-how-to-compare-ur3-robot-model-to-hardware-test-positions-using-inverse-kinematics-in-robotics-toolb?s_tid=srchtitle

rosinit
% rosshutdown
%%
clearvars
clear all
close all
clc

% publicar
% Crear publicador
pub = rospublisher('/joint_states','sensor_msgs/JointState');
msg = rosmessage(pub);

pub2 = rospublisher('/joint_states2','sensor_msgs/JointState');
msg2 = rosmessage(pub2);



pub_aguja = rospublisher('/visibility_control', 'std_msgs/Bool');
msg_aguja = rosmessage(pub_aguja)


% Establecer valores de mensaje
msg.Header.Stamp = rostime('now');
msg.Header.FrameId = 'base_link';
msg.Name = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};

% Establecer valores de mensaje
msg2.Header.Stamp = rostime('now');
msg2.Header.FrameId = 'base_link';
msg2.Name = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};

% Cargar modelo 0.45robot UR3
[UR3, UR3Data] = loadrobot('universalUR3','DataFormat','row','Gravity',[0 0 -9.81]);
tformZYX = eul2tform([pi 0 0]);
setFixedTransform(UR3.Base.Children{1,1}.Joint,tformZYX);


% Cargar modelo 0.45robot UR3
[UR32, UR3Data2] = loadrobot('universalUR3','DataFormat','row','Gravity',[0 0 -9.81]);
tformZYX = eul2tform([pi 0 0]);
setFixedTransform(UR32.Base.Children{1,1}.Joint,tformZYX);








% crear objeto para cinemática inversa
ik_UR3 = inverseKinematics('RigidBodyTree', UR3); 
ik_UR32 = inverseKinematics('RigidBodyTree', UR32); 


% tolerancias para el cálculo de orientación (los tres primeros son para la
% orientación
weights=[1 1 1 1 1 1];

%%

    
con1=0
art_robot=[]

% posicion de reposo

h1=[-200, -67.210000, -90, -56.160000, 90, 0.000000]
Home=deg2rad(h1)

h2=[10, -67.210000, -90, -56.160000, 90, 0.000000]
Home2=deg2rad(h2)



msg2.Position = Home2;
msg2.Velocity = [1, 1, 1,1, 1, 1];
msg2.Effort = [];
send(pub2,msg2);


msg.Position = Home;
msg.Velocity = [1, 1, 1,1, 1, 1];
msg.Effort = [];
send(pub,msg);

Ts_inicial=getTransform(UR3,Home,"tool0")
Ts_inicial2=getTransform(UR32,Home2,"tool0")

pini_r=Ts_inicial(1:3,4);
pini_r2=Ts_inicial2(1:3,4);


h1=[-180, -67.210000, -90, -56.160000, 90, 0.000000]
Home=deg2rad(h1)
h2=[-0.000215, -62.909283, -89.023806, -61.436953, 90.000499, 0.000054]
Home2=deg2rad(h2)
Ts_inicial=getTransform(UR3,Home,"tool0")
Ts_inicial2=getTransform(UR32,Home2,"tool0")


pfin_r=Ts_inicial(1:3,4);
pfin_r2=Ts_inicial2(1:3,4);


vel=1
pts_intermedios = tray_lineal(pini_r, pfin_r, vel)
cont_ruta=length(pts_intermedios)
T_a=[1 0 0 0;0 1 0 0; 0 0 1 0;0 0 0 1];
for i=1:1:cont_ruta

        T_a(1:3,4)=pts_intermedios(i,:)';

        T_a(1:3,1:3)=Ts_inicial(1:3,1:3);
        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
     
        Tp=T_a
        [robot_pose, solnInfo]=ik_UR3('tool0',Tp, weights, Home);


        msg.Position = robot_pose;

        msg.Velocity = [1, 1, 1,1, 1, 1];
        msg.Effort = [];
        send(pub,msg);
        pause(0.01)

end




%%

h1=[-180, -67.210000, -90, -56.160000, 90, 0.000000]
Home=deg2rad(h1)

h2=[-0.000215, -62.909283, -89.023806, -61.436953, 90.000499, 0.000054]
Home2=deg2rad(h2)


% 
% msg2.Position = Home2;
% msg2.Velocity = [1, 1, 1,1, 1, 1];
% msg2.Effort = [];
% send(pub2,msg2);


msg.Position = Home;
msg.Velocity = [1, 1, 1,1, 1, 1];
msg.Effort = [];
send(pub,msg);





%
Ts_inicial=getTransform(UR3,Home,"tool0")

Tpq=[1 0 0 0;0 1 0 0; 0 0 1 0.30;0 0 0 1];
Tpq=Ts_inicial*Tpq;
PEe21=Tpq(1:3,4);
p1 = [PEe21(1)-0.000;PEe21(2)-0.0252;PEe21(3)-0.0005]
p2 = [PEe21(1)-0.0004;PEe21(2)-0.012;PEe21(3)-0.000502]

p3=(p1'-[0.01 0 0 ])'
p4=(p2'-[0.01 0 0 ])'


p5=(p1'-[0.02 0 0 ])'
p6=(p2'-[0.02 0 0 ])'


Rd=0.008;
Tp_ruta_r1=md_circulo_v5nuevo(p1,p2,Rd);
Tp_ruta_r2=md_circulo_v5nuevo(p3,p4,Rd);
Tp_ruta_r3=md_circulo_v5nuevo(p5,p6,Rd);





Ts_inicial2=getTransform(UR32,Home2,"tool0")

Tpq=[1 0 0 0;0 1 0 0; 0 0 1 0.30;0 0 0 1];
Tpq=Ts_inicial2*Tpq;
PEe21=Tpq(1:3,4);
p1 = [PEe21(1)-0.000;PEe21(2)-0.0252;PEe21(3)-0.0005]
p2 = [PEe21(1)-0.0004;PEe21(2)-0.012;PEe21(3)-0.000502]

p3=(p1'-[0.01 0 0 ])'
p4=(p2'-[0.01 0 0 ])'


p5=(p1'-[0.02 0 0 ])'
p6=(p2'-[0.02 0 0 ])'

Tp_ruta_a1=md_circulo_v5nuevo(p1,p2,Rd);
Tp_ruta_a2=md_circulo_v5nuevo(p3,p4,Rd);
Tp_ruta_a3=md_circulo_v5nuevo(p5,p6,Rd);



%%
% movimiento de primera  puntada
Tp_ruta=Tp_ruta_r1;
cont_ruta=length(Tp_ruta)
rota=-1.570 ;

for i=1:1:cont_ruta
        rota=rota-i*0.00032;
        % nota: rota debe cambiar desde 0 a 180 o aprox   
        T_a=Ts_inicial
        T_a(1:3,4)=Tp_ruta(i,:)';
        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
        Tpq=[1 0 0 0;0 1 0 0; 0 0 1 -0.3;0 0 0 1];
        Tp=T_a*Tpq;
        [robot_pose, solnInfo]=ik_UR3('tool0',Tp, weights, Home);
        Home=robot_pose;
        pose2=robot_pose+[0 0 0 0 0 rota];
        msg.Position = pose2;
        msg.Velocity = [1, 1, 1,1, 1, 1];
        msg.Effort = [];
        send(pub,msg);
        % ultima posicion 
        p_ult=Tp(1:3,4);
end




vel=1



T_a=Ts_inicial2
Tp_ruta2=Tp_ruta_a1 %flip(Tp_ruta_a1);
T_a(1:3,4)=Tp_ruta2(1,:)';
Tpq=[1 0 0 0;0 1 0 0; 0 0 1 -0.3;0 0 0 1];
Tp=T_a*Tpq;
pfin_r2=Tp(1:3,4)
pts_intermedios = tray_lineal(pini_r2, pfin_r2, vel)
cont_ruta=length(pts_intermedios)
T_a=[1 0 0 0;0 1 0 0; 0 0 1 0;0 0 0 1];
for i=1:1:cont_ruta

        T_a(1:3,4)=pts_intermedios(i,:)';

        T_a(1:3,1:3)=Ts_inicial2(1:3,1:3);
        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
     
        Tp=T_a
        [robot_pose, solnInfo]=ik_UR32('tool0',Tp, weights, Home2);

        p_ult2=Tp(1:3,4);
        msg2.Position = robot_pose;

        msg2.Velocity = [1, 1, 1,1, 1, 1];
        msg2.Effort = [];
        send(pub2,msg2);
        pause(0.01)

end


%
pause(0.01)
Tp_ruta2=Tp_ruta_a1 %flip(Tp_ruta_a1);
cont_ruta=length(Tp_ruta2);
rota=-1.570 ;


for i=1:1:cont_ruta
        rota=rota+i*0.00032;
        % nota: rota debe cambiar desde 0 a 180 o aprox   
     
        T_a=Ts_inicial2
        T_a(1:3,4)=Tp_ruta2(i,:)';


        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
        Tpq=[1 0 0 0;0 1 0 0; 0 0 1 -0.3;0 0 0 1];
        Tp=T_a*Tpq;
        [robot_pose, solnInfo]=ik_UR32('tool0',Tp, weights, Home2);
        Home2=robot_pose;
        pose2=robot_pose+[0 0 0 0 0 rota];
        msg2.Position = pose2;
        
        msg2.Velocity = [1, 1, 1,1, 1, 1];
        msg2.Effort = [];
        send(pub2,msg2);
        % ultima posicion 
        p_ult2=Tp(1:3,4);
%    pause(0.01);
end




T_a=Ts_inicial;
Tp_ruta=Tp_ruta_r1;
T_a(1:3,4)=Tp_ruta(1,:)';
Tpq=[1 0 0 0;0 1 0 0; 0 0 1 -0.3;0 0 0 1];
Tp=T_a*Tpq;
pfin_r1=Tp(1:3,4);
pin1_r1=p_ult;
pts_intermedios = tray_lineal(pin1_r1, pfin_r1, vel);
cont_ruta=length(pts_intermedios);
T_a=[1 0 0 0;0 1 0 0; 0 0 1 0;0 0 0 1];
for i=1:1:cont_ruta

        T_a(1:3,4)=pts_intermedios(i,:)';

        T_a(1:3,1:3)=Ts_inicial(1:3,1:3);
        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
     
        Tp=T_a;
        [robot_pose, solnInfo]=ik_UR3('tool0',Tp, weights, Home);

        p_ult=Tp(1:3,4);
        msg.Position = robot_pose;

        msg.Velocity = [1, 1, 1,1, 1, 1];
        msg.Effort = [];
        send(pub,msg);
        pause(0.01)

end
%
pause(1)



pfin_r2=(p_ult2'-[0.04 0 0 ])';

pts_intermedios = tray_lineal(p_ult2, pfin_r2, vel);
cont_ruta=length(pts_intermedios);
T_a=[1 0 0 0;0 1 0 0; 0 0 1 0;0 0 0 1];
for i=1:1:cont_ruta

        T_a(1:3,4)=pts_intermedios(i,:)';

        T_a(1:3,1:3)=Ts_inicial2(1:3,1:3);
        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
     
        Tp=T_a;
        [robot_pose, solnInfo]=ik_UR32('tool0',Tp, weights, Home2);

        p_ult2=Tp(1:3,4);
        msg2.Position = robot_pose;

        msg2.Velocity = [1, 1, 1,1, 1, 1];
        msg2.Effort = [];
        send(pub2,msg2);
        pause(0.01)

end

%% inicio segunda puntada


T_a=Ts_inicial;
Tp_ruta=Tp_ruta_r2;
T_a(1:3,4)=Tp_ruta(1,:)';
Tpq=[1 0 0 0;0 1 0 0; 0 0 1 -0.3;0 0 0 1];
Tp=T_a*Tpq;
pfin_r1=Tp(1:3,4);
pin1_r1=p_ult;
pts_intermedios = tray_lineal(pin1_r1, pfin_r1, vel);
cont_ruta=length(pts_intermedios);
T_a=[1 0 0 0;0 1 0 0; 0 0 1 0;0 0 0 1];
for i=1:1:cont_ruta

        T_a(1:3,4)=pts_intermedios(i,:)';

        T_a(1:3,1:3)=Ts_inicial(1:3,1:3);
        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
     
        Tp=T_a;
        [robot_pose, solnInfo]=ik_UR3('tool0',Tp, weights, Home);

        p_ult=Tp(1:3,4);
        msg.Position = robot_pose;

        msg.Velocity = [1, 1, 1,1, 1, 1];
        msg.Effort = [];
        send(pub,msg);
        pause(0.01)

end
%
pause(1)


Tp_ruta=Tp_ruta_r2;
cont_ruta=length(Tp_ruta)
rota=-1.570 ;

for i=1:1:cont_ruta
        rota=rota-i*0.00032;
        % nota: rota debe cambiar desde 0 a 180 o aprox   
        T_a=Ts_inicial
        T_a(1:3,4)=Tp_ruta(i,:)';
        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
        Tpq=[1 0 0 0;0 1 0 0; 0 0 1 -0.3;0 0 0 1];
        Tp=T_a*Tpq;
        [robot_pose, solnInfo]=ik_UR3('tool0',Tp, weights, Home);
        Home=robot_pose;
        pose2=robot_pose+[0 0 0 0 0 rota];
        msg.Position = pose2;
        msg.Velocity = [1, 1, 1,1, 1, 1];
        msg.Effort = [];
        send(pub,msg);
        % ultima posicion 
        p_ult=Tp(1:3,4);
end




vel=1



T_a=Ts_inicial2
Tp_ruta2=Tp_ruta_a2 %flip(Tp_ruta_a1);
T_a(1:3,4)=Tp_ruta2(1,:)';
Tpq=[1 0 0 0;0 1 0 0; 0 0 1 -0.3;0 0 0 1];
Tp=T_a*Tpq;
pfin_r2=Tp(1:3,4)
pts_intermedios = tray_lineal(pini_r2, pfin_r2, vel)
cont_ruta=length(pts_intermedios)
T_a=[1 0 0 0;0 1 0 0; 0 0 1 0;0 0 0 1];
for i=1:1:cont_ruta

        T_a(1:3,4)=pts_intermedios(i,:)';

        T_a(1:3,1:3)=Ts_inicial2(1:3,1:3);
        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
     
        Tp=T_a
        [robot_pose, solnInfo]=ik_UR32('tool0',Tp, weights, Home2);

        p_ult2=Tp(1:3,4);
        msg2.Position = robot_pose;

        msg2.Velocity = [1, 1, 1,1, 1, 1];
        msg2.Effort = [];
        send(pub2,msg2);
        pause(0.01)

end


%
pause(0.01)
Tp_ruta2=Tp_ruta_a2 %flip(Tp_ruta_a1);
cont_ruta=length(Tp_ruta2);
rota=-1.570 ;


for i=1:1:cont_ruta
        rota=rota+i*0.00032;
        % nota: rota debe cambiar desde 0 a 180 o aprox   
     
        T_a=Ts_inicial2
        T_a(1:3,4)=Tp_ruta2(i,:)';


        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
        Tpq=[1 0 0 0;0 1 0 0; 0 0 1 -0.3;0 0 0 1];
        Tp=T_a*Tpq;
        [robot_pose, solnInfo]=ik_UR32('tool0',Tp, weights, Home2);
        Home2=robot_pose;
        pose2=robot_pose+[0 0 0 0 0 rota];
        msg2.Position = pose2;
        
        msg2.Velocity = [1, 1, 1,1, 1, 1];
        msg2.Effort = [];
        send(pub2,msg2);
        % ultima posicion 
        p_ult2=Tp(1:3,4);
%    pause(0.01);
end




T_a=Ts_inicial;
Tp_ruta=Tp_ruta_r2;
T_a(1:3,4)=Tp_ruta(1,:)';
Tpq=[1 0 0 0;0 1 0 0; 0 0 1 -0.3;0 0 0 1];
Tp=T_a*Tpq;
pfin_r1=Tp(1:3,4);


pin1_r1=p_ult;



pts_intermedios = tray_lineal(pin1_r1, pfin_r1, vel);
cont_ruta=length(pts_intermedios);
T_a=[1 0 0 0;0 1 0 0; 0 0 1 0;0 0 0 1];
for i=1:1:cont_ruta

        T_a(1:3,4)=pts_intermedios(i,:)';

        T_a(1:3,1:3)=Ts_inicial(1:3,1:3);
        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
     
        Tp=T_a;
        [robot_pose, solnInfo]=ik_UR3('tool0',Tp, weights, Home);

        p_ult=Tp(1:3,4);
        msg.Position = robot_pose;

        msg.Velocity = [1, 1, 1,1, 1, 1];
        msg.Effort = [];
        send(pub,msg);
        pause(0.01)

end
%
pause(1)



pfin_r2=(p_ult2'-[0.04 0 0 ])';

pts_intermedios = tray_lineal(p_ult2, pfin_r2, vel);
cont_ruta=length(pts_intermedios);
T_a=[1 0 0 0;0 1 0 0; 0 0 1 0;0 0 0 1];
for i=1:1:cont_ruta

        T_a(1:3,4)=pts_intermedios(i,:)';

        T_a(1:3,1:3)=Ts_inicial2(1:3,1:3);
        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
     
        Tp=T_a;
        [robot_pose, solnInfo]=ik_UR32('tool0',Tp, weights, Home2);

        p_ult2=Tp(1:3,4);
        msg2.Position = robot_pose;

        msg2.Velocity = [1, 1, 1,1, 1, 1];
        msg2.Effort = [];
        send(pub2,msg2);
        pause(0.01)

end

%% tercera
%% inicio segunda puntada


T_a=Ts_inicial;
Tp_ruta=Tp_ruta_r3;
T_a(1:3,4)=Tp_ruta(1,:)';
Tpq=[1 0 0 0;0 1 0 0; 0 0 1 -0.3;0 0 0 1];
Tp=T_a*Tpq;
pfin_r1=Tp(1:3,4);
pin1_r1=p_ult;
pts_intermedios = tray_lineal(pin1_r1, pfin_r1, vel);
cont_ruta=length(pts_intermedios);
T_a=[1 0 0 0;0 1 0 0; 0 0 1 0;0 0 0 1];
for i=1:1:cont_ruta

        T_a(1:3,4)=pts_intermedios(i,:)';

        T_a(1:3,1:3)=Ts_inicial(1:3,1:3);
        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
     
        Tp=T_a;
        [robot_pose, solnInfo]=ik_UR3('tool0',Tp, weights, Home);

        p_ult=Tp(1:3,4);
        msg.Position = robot_pose;

        msg.Velocity = [1, 1, 1,1, 1, 1];
        msg.Effort = [];
        send(pub,msg);
        pause(0.01)

end
%
pause(1)


Tp_ruta=Tp_ruta_r3;
cont_ruta=length(Tp_ruta)
rota=-1.570 ;

for i=1:1:cont_ruta
        rota=rota-i*0.00032;
        % nota: rota debe cambiar desde 0 a 180 o aprox   
        T_a=Ts_inicial
        T_a(1:3,4)=Tp_ruta(i,:)';
        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
        Tpq=[1 0 0 0;0 1 0 0; 0 0 1 -0.3;0 0 0 1];
        Tp=T_a*Tpq;
        [robot_pose, solnInfo]=ik_UR3('tool0',Tp, weights, Home);
        Home=robot_pose;
        pose2=robot_pose+[0 0 0 0 0 rota];
        msg.Position = pose2;
        msg.Velocity = [1, 1, 1,1, 1, 1];
        msg.Effort = [];
        send(pub,msg);
        % ultima posicion 
        p_ult=Tp(1:3,4);
end




vel=1



T_a=Ts_inicial2
Tp_ruta2=Tp_ruta_a3 %flip(Tp_ruta_a1);
T_a(1:3,4)=Tp_ruta2(1,:)';
Tpq=[1 0 0 0;0 1 0 0; 0 0 1 -0.3;0 0 0 1];
Tp=T_a*Tpq;
pfin_r2=Tp(1:3,4)
pts_intermedios = tray_lineal(pini_r2, pfin_r2, vel)
cont_ruta=length(pts_intermedios)
T_a=[1 0 0 0;0 1 0 0; 0 0 1 0;0 0 0 1];
for i=1:1:cont_ruta

        T_a(1:3,4)=pts_intermedios(i,:)';

        T_a(1:3,1:3)=Ts_inicial2(1:3,1:3);
        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
     
        Tp=T_a
        [robot_pose, solnInfo]=ik_UR32('tool0',Tp, weights, Home2);

        p_ult2=Tp(1:3,4);
        msg2.Position = robot_pose;

        msg2.Velocity = [1, 1, 1,1, 1, 1];
        msg2.Effort = [];
        send(pub2,msg2);
        pause(0.01)

end


%
pause(0.01)
Tp_ruta2=Tp_ruta_a3 %flip(Tp_ruta_a1);
cont_ruta=length(Tp_ruta2);
rota=-1.570 ;


for i=1:1:cont_ruta
        rota=rota+i*0.00032;
        % nota: rota debe cambiar desde 0 a 180 o aprox   
     
        T_a=Ts_inicial2
        T_a(1:3,4)=Tp_ruta2(i,:)';


        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
        Tpq=[1 0 0 0;0 1 0 0; 0 0 1 -0.3;0 0 0 1];
        Tp=T_a*Tpq;
        [robot_pose, solnInfo]=ik_UR32('tool0',Tp, weights, Home2);
        Home2=robot_pose;
        pose2=robot_pose+[0 0 0 0 0 rota];
        msg2.Position = pose2;
        
        msg2.Velocity = [1, 1, 1,1, 1, 1];
        msg2.Effort = [];
        send(pub2,msg2);
        % ultima posicion 
        p_ult2=Tp(1:3,4);
%    pause(0.01);
end




T_a=Ts_inicial;
Tp_ruta=Tp_ruta_r3;
T_a(1:3,4)=Tp_ruta(1,:)';
Tpq=[1 0 0 0;0 1 0 0; 0 0 1 -0.3;0 0 0 1];
Tp=T_a*Tpq;
pfin_r1=Tp(1:3,4);


pin1_r1=p_ult;



pts_intermedios = tray_lineal(pin1_r1, pfin_r1, vel);
cont_ruta=length(pts_intermedios);
T_a=[1 0 0 0;0 1 0 0; 0 0 1 0;0 0 0 1];
for i=1:1:cont_ruta

        T_a(1:3,4)=pts_intermedios(i,:)';

        T_a(1:3,1:3)=Ts_inicial(1:3,1:3);
        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
     
        Tp=T_a;
        [robot_pose, solnInfo]=ik_UR3('tool0',Tp, weights, Home);

        p_ult=Tp(1:3,4);
        msg.Position = robot_pose;

        msg.Velocity = [1, 1, 1,1, 1, 1];
        msg.Effort = [];
        send(pub,msg);
        pause(0.01)

end
%
pause(1)



pfin_r2=(p_ult2'-[0.04 0 0 ])';

pts_intermedios = tray_lineal(p_ult2, pfin_r2, vel);
cont_ruta=length(pts_intermedios);
T_a=[1 0 0 0;0 1 0 0; 0 0 1 0;0 0 0 1];
for i=1:1:cont_ruta

        T_a(1:3,4)=pts_intermedios(i,:)';

        T_a(1:3,1:3)=Ts_inicial2(1:3,1:3);
        % con esta linea se encuentra la posicion relativa del terminal del robot, restando la long de la herramienta
     
        Tp=T_a;
        [robot_pose, solnInfo]=ik_UR32('tool0',Tp, weights, Home2);

        p_ult2=Tp(1:3,4);
        msg2.Position = robot_pose;

        msg2.Velocity = [1, 1, 1,1, 1, 1];
        msg2.Effort = [];
        send(pub2,msg2);
        pause(0.01)

end
