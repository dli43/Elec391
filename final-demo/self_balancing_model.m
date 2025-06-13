clear; clc;

k_speed = 0.202;    %Relation between pwm and rotor speed
wheel_radius = 0.037;
surface_friction = 12.26;
l_cog = 0.04;       %Distance from wheel pivot to COG
m = 1.25;
I = 8.667e-4;
g = 9.81;
rad2deg = 180/pi;

s = tf('s');

CF = 100;
Nt = 1;

A_p = 1/(I*s^2-m*g*l_cog);  %Inverted Pendulum tf

%% Find master gain
Dp = CF/(s*(Nt*s+CF));
K_master = 6
G = k_speed*wheel_radius*surface_friction*-l_cog*A_p
Hs = feedback(K_master*Dp*G,rad2deg);
pole(Hs)
    
%% Find crossover frequency
margin(K_master*Dp*G*rad2deg)
wxo = 2.84;

%% Find zeros placement
MagRes = 0.01;
wx = 2.84;
if 1
    for i = -20:25;
        wz = wx + i*MagRes;
        Dz = (s+wz)*(s+wz)/(s*wz^2);
        D = Dp*Dz;
        Gy = K_master*D*G*rad2deg;
        [Gm Pm] = margin(Gy);
        display([wz Pm]);
    end
end
wc = 20.23;

%Seperation Distance
if 0
    for i = 0:15;
        wz1 = wc + i*MagRes;
        wz2 = wc - i*MagRes;
        Dz = (s+wz1)*(s+wz2)/(wz1*wz2);
        D = Dp*Dz;
        Gy = K_master*D*G;
        [Gm Pm] = margin(Gy);
        display([wz1 wz2 2*i*MagRes Pm]);
    end
end

%Complex zeros
Res = 0.01;
wn = wc;
if 0
    for i = 0:100
        zeta = Res*i;
        Dz = tf([1 2*zeta*wn wn^2],[wn^2]);
        Gy = K_master*Dz*Dp*G;
        [Gm Pm] = margin(Gy);
        if(zeta <= 1)
            display([zeta Pm]);
        end
    end
end

%Get zeros
zeta = 0.9;
arg = acos(zeta);
z1 = wn*exp(j*(pi-arg));
z2 = conj(z1);
Z = [z1 z2];

%% Get PID Values
z1 = Z(1);
z2 = Z(2);
p = CF
Kp = K_master*(1/p - (z1+z2)/(z1*z2));
Ki = K_master;
Kd = K_master*(1/p^2 - (z1+z2-p)/(p*(z1*z2)));
