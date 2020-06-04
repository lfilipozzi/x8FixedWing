clear; clc;
close all;

%% Run Simulink file
sim('model')

%% Plot results
figure
subplot(2,1,1)
hold on
box on
plot(target.An_BodyRollRef * 180/pi,'-k')
plot(output.An_BodyRoll    * 180/pi)
ylabel('Roll angle (deg)')

subplot(2,1,2)
hold on
box on
plot(target.An_BodyPitchRollRef * 180/pi,'-k')
plot(output.An_BodyPitch * 180/pi)
ylabel('Pitch angle (deg)')
xlabel('Time (s)')
