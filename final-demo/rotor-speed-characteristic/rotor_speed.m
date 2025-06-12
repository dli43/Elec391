clc; clear;

table = readtable("rotor_speed_data.csv");
duty_cycle = table.DutyCycle;
left_rotor_rads = table.LeftRotorSpeed_rad_s_;
right_rotor_rads = table.RightRotorSpeed_rad_s_;

figure(1); clf;
plot(duty_cycle,left_rotor_rads,"r-",duty_cycle,right_rotor_rads,"b-");
ylim([-55,55]);
xlim([-260,260]);
grid on;

rel1 = fitlm(duty_cycle,left_rotor_rads, "linear");
rel2 = fitlm(duty_cycle,right_rotor_rads, "linear");

