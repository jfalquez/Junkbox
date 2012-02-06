% Ramp info
RampWidth = 1.0; % in meters
RampHeight = 1.0; % in meters
RampAngle = 40; % in deg


% Ramp positions in world reference frame
Ramp1Pos = [ 5.0 0 sind(RampAngle)*RampHeight/2 0 -RampAngle*pi/180 0]'; % centroid - [x y z p q r]
Twr1 = Cart2T(Ramp1Pos);
Ramp2Pos = [ 8.0 0 sind(RampAngle)*RampHeight/2 0 -RampAngle*pi/180 0]'; % centroid
Twr2 = Cart2T(Ramp2Pos);


% Car info
CarCurPos = [0.0 0 0.2 0 0 0]'; % centroid
CarCurVel = 0.0; % in meters per second
CarMaxVel = 5.0; % in meters per second
CarMaxAcc = 3.0; % in meters per second^2


% Point at which car leaves ramp
ReleasePos = Twr1 * [RampHeight/2 0 0 1]';
ReleasePos = ReleasePos(1:3);

% Calculate target distance
dH = ReleasePos(3) - Ramp1Pos(3);
R = norm(Ramp1Pos(1:3) - Ramp2Pos(1:3)) - 2*(dH/tand(RampAngle));

% Velocity required to make jump
G = 9.81; % gravity
V = sqrt(R * G/sind(2*RampAngle));
