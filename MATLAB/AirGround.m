% Reference frame is X = forward, Y = left, Z = up

% Threshold to consider objects still on ground
zThreshold = .10;

% Ramp info
RampHeight = 1.0; % in meters
RampWidth = 1.0; % in meters
RampAngle = 40; % in deg

% Ramp positions in world reference frame
Ramp1Pos = [ 5.0 0 sind(RampAngle)*RampHeight/2 0 -RampAngle*pi/180 0]'; % centroid - [x y z p q r]
Twr1 = Cart2T(Ramp1Pos);
Ramp2Pos = [ 10.0 0 sind(RampAngle)*RampHeight/2 0 -RampAngle*pi/180 0]'; % centroid
Twr2 = Cart2T(Ramp2Pos);


% Point we wish to analyze (current position)
P = [1 2 3]';


%%%%%%%%%%%%%%%%%%%%%%%

% Four vertices with respect to ramp's reference frame
V1 = [RampHeight/2 RampWidth/2 0 1]'; % in homogeneous coordinates
V2 = [RampHeight/2 -RampWidth/2 0 1]';
V3 = [-RampHeight/2 RampWidth/2 0 1]';
V4 = [-RampHeight/2 -RampWidth/2 0 1]';

%%%%%%%%%%%%%%%%%%%%%%%

% Ramp1's four vertices in world's reference frame
V1w = Twr1 * V1;
V2w = Twr1 * V2;
V3w = Twr1 * V3;
V4w = Twr1 * V4;

% Vectors between point and vertices of Ramp1
PV1 = V1w(1:3) - P;
PV2 = V2w(1:3) - P;
PV3 = V3w(1:3) - P;
PV4 = V4w(1:3) - P;

AngleSum = acos(PV1'*PV2/norm(PV1) * norm(PV2)) + acos(PV2'*PV3/norm(PV2) * norm(PV3)) + acos(PV3'*PV4/norm(PV3) * norm(PV4)) + acos(PV4'*PV4/norm(PV4) * norm(PV1))

% if on Ramp1
if ( abs(real(AngleSum) - 2*pi) < 1 )
	printf("RAMP1\n");
	return;
endif



% Ramp2's four vertices in world's reference frame
V1w = Twr2 * V1;
V2w = Twr2 * V2;
V3w = Twr2 * V3;
V4w = Twr2 * V4;

% Vectors between point and vertices of Ramp2
PV1 = V1w(1:3) - P;
PV2 = V2w(1:3) - P;
PV3 = V3w(1:3) - P;
PV4 = V4w(1:3) - P;

AngleSum = acos(PV1'*PV2/norm(PV1) * norm(PV2)) + acos(PV2'*PV3/norm(PV2) * norm(PV3)) + acos(PV3'*PV4/norm(PV3) * norm(PV4)) + acos(PV4'*PV4/norm(PV4) * norm(PV1));

% if on Ramp2
if ( abs(real(AngleSum) - 2*pi) < 1 )
	printf("RAMP2\n");
	return;
endif



% if on the floor
if ( P(3) < 0 + zThreshold )
	printf("GROUND\n");
	return;
endif

printf("AIR!\n");
