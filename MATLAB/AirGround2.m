% Reference frame is X = forward, Y = left, Z = up

% Threshold to consider objects still on ground
zThreshold = .10;

% Ramp info - static info (can be inferred by MoCap)
RampWidth = 1.0; % in meters
RampHeight = 1.0; % in meters
RampAngle = 40; % in deg

% Ramp position + orientation in world reference frame (given by MoCap)
Ramp1Pos = [ 5.0 0 sind(RampAngle)*RampHeight/2 0 -RampAngle*pi/180 0]'; % centroid - [x y z p q r]
Twr1 = Cart2T(Ramp1Pos);
Ramp2Pos = [ 10.0 0 sind(RampAngle)*RampHeight/2 0 -RampAngle*pi/180 0]'; % centroid
Twr2 = Cart2T(Ramp2Pos);


% Point we wish to analyze; position of car (given by MoCap)
P = [1 2 0.08]';


%%%%%%%%%%%%%%%%%%%%%%%


% We bring the point to each of the ramp's reference frames
Pr1 = TInv(Twr1) * [P' 1]';
Pr2 = TInv(Twr2) * [P' 1]';

% Check if we are within ramp's "area"
% < 0 should work, but octave is acting up
if ( (abs(Pr1(1)) - RampHeight/2 < 0.0001) && (abs(Pr1(2)) - RampWidth/2 < 0.0001) && (Pr1(3) <= zThreshold) )
	printf("RAMP 1!\n");
	return;
endif

if ( (abs(Pr2(1)) - RampHeight/2 < 0.0001) && (abs(Pr2(2)) - RampWidth/2 < 0.0001) && (Pr2(3) <= zThreshold) )
	printf("RAMP 2!\n");
	return;
endif

% if on the floor
if ( P(3) < 0 + zThreshold )
	printf("GROUND\n");
	return;
endif

printf("AIR!\n");
