%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GeneratePath4MeshLogger:
% Program used to generate a file of poses which can be fed
% to the C++ MeshLogger application to render images.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Clear
clear all

% Vehicle Pose
global Pose;

% File Descriptor
global FileID;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Rad = Deg2Rad( Deg )
    Rad = Deg * pi / 180;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function PrintTransform( NewPose )
    global Pose;
    global FileID;
    NewPose = NewPose - Pose;
    fprintf( FileID, "%g\t%g\t%g\t%g\t%g\t%g\n", NewPose );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Init( P )
    global Pose;
    global FileID;
    FileID = fopen( 'poses.txt', 'w' );
    Pose = P;
    fprintf( FileID, "%g\t%g\t%g\t%g\t%g\t%g\n", Pose );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function MoveAbs( NewPose, Steps )
    global Pose;
    DeltaPose = NewPose - Pose(1:3);
    MoveRel( DeltaPose, Steps );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function MoveRel( Delta, Steps )
    global Pose;
    Delta = [ Delta 0 0 0 ];
    DeltaInc = Delta .* 1/Steps;
    DeltaIncT = Cart2T( DeltaInc' );
    PoseT = Cart2T( Pose' );

    for ii=1:Steps
        PoseT = PoseT * DeltaIncT;
        PrintTransform( T2Cart( PoseT )' );
        Pose = T2Cart( PoseT )';
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Rotate( Delta, Steps, Forward = 0 )
    global Pose;
    Delta = [ Forward 0 0 Delta ];
    DeltaInc = Delta .* 1/Steps;
    DeltaIncT = Cart2T( DeltaInc' );
    PoseT = Cart2T( Pose' );

    for ii=1:Steps
        PoseT = PoseT * DeltaIncT;
        PrintTransform( T2Cart( PoseT )' );
        Pose = T2Cart( PoseT )';
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize vehicle pose
Init( [ 0 -7.5 -1.5 0 0 0 ] );

% Perform movements...
MoveAbs( [ 5.0 -7.5 -1.5 ], 25 );
Rotate( [0 0 Deg2Rad(90) ], 30, 5 );
MoveRel( [ 9.0 0 0 ], 50 );
Rotate( [0 0 Deg2Rad(90) ], 30, 5 );
MoveRel( [ 9.0 0 0 ], 50 );
Rotate( [0 0 Deg2Rad(90) ], 30, 5 );
MoveRel( [ 9.0 0 0 ], 50 );
Rotate( [0 0 Deg2Rad(90) ], 30, 5 );
MoveRel( [ 9.0 0 0 ], 50 );


% ... and close file since we are done!
fclose(FileID);