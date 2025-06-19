% *************************************************************************
% INITIALIZATION
% *************************************************************************

% A.0 Load robot-specific definitions for gripper open/closed position
% commands.
% -------------------------------------------------------------------------

% A.01 - Get robot identifier.
% A.02 - Select robot-specific gripper positions.
% A.03 - Set desired gripper positions as constant global variables.
% --> Double-check for both robots in practice!
% --> Do these definitions have to go into the header file? Can we change
%     C++ header files dynamically at run-time? Or should we manually
%     change the code before uploading it to a given robot?



% A.1 Load table level/height, define table as obstacle.
% -------------------------------------------------------------------------
% A.11 - Confirm actual table dimensions and position/orientation w.r.t.
%        each robot.
% A.12 - Define table obstacle as per Viviana's sample code, lines 44-54 in
%        main_motion.cpp.
z0 = 0; % Table height.


% A.2 Load tower base (centre?) coordinates.
% -------------------------------------------------------------------------
% A.21 - Use robot's camera to visually scan table.
% A.22 - Get location of QR code.
% A.23 - Set desired tower location based on QR code location.
% A.24 - Compute tower corner/brick center locations/orientations in
%        robot's coordinate frame w.r.t. desired tower location.
% --> What existing PAL/TIAGo functions can we use to retrive the QR code's
%     location?
% --> What do we get from the QR code? Its center? Its corners?




% A.3
% -------------------------------------------------------------------------
% Compute array of 100 planar brick poses within robot's coordinate system, 
% w.r.t tower base. 

% Define target brick positions and orientations.
x1 = 1; % Brick center x-position in LEFT configuration.
x2 = 2; % Brick center x-position in TOP/BOTTOM configurations.
x3 = 3; % Brick center x-position in RIGHT configuration.

y1 = 1; % Brick center y-position in BOTTOM configuration.
y2 = 2; % Brick center y-position in LEFT/RIGHT configurations.
y3 = 3; % Brick center y-position in TOP configuration.

a1 = 0;  % Brick orientation in HORIZONTAL configuration.
a2 = 90; % Brick orientation in VERTICAL configuration.

zInc = 0.25; % Brick height, used to increment z-position for each layer.

% Create array of brick positions/orientations for 100 drops, assuming the
% following format:
% [x1 y2 a2 z0] % left vertical, layer 1
% [x3 y2 a2 z0] % right vertical, layer 1
% [x2 y3 a1 (z0 + 1*zInc)] % top horizontal, layer 2
% [x2 y1 a1 (z0 + 1*zInc)] % bottom horizontal, layer 2
 
arrayX = repmat([x1; x3; x2; x2],[25, 1]);
arrayY = repmat([y2; y2; y3; y1],[25, 1]);
arrayZ = [1:50]';
arrayZ = (sort(repmat(arrayZ,[2, 1]),'ascend') * zInc) + z0;
brickLocationArray = [arrayX, arrayY, arrayA, arrayZ];

% Set brick array index to one (to identify the current target brick).
currentBrick = 1;




% A.4
% -------------------------------------------------------------------------
% Define brick hand-off location and robot's arm pose (elbow down).

 


% A.5
% -------------------------------------------------------------------------
% Initialize robot state to RECEIVE.
 



% *************************************************************************
% GENERAL WORKFLOW
% *************************************************************************

% B.0 Start keystroke event listener.

% B.1 Set loop exit condition.
% -------------------------------------------------------------------------
% --> Should we use a keystroke? 'q' for quit?



% B.2 While loop exit condition is not met...
% -------------------------------------------------------------------------
% B.21  - Set robot state to RECEIVE.

% B.22  - Run goToReceiveState():
% B.221 - Move end-effector to brick hand-off location and pose.
% B.222 - Human operator holds a brick in the appropriate position and
% location for repeatable grasping via gripper.
% B.223 - Human enters keystroke commanding gripper to close.
% B.225 - Set robot state to DEPOSIT.

% B.23  - Run goOverIntendedCoordinate():
% B.231 - Robot switches to elbow-up configuration.
% B.232 - Robot retrieves target brick-deposition location 
targetLocation = brickLocationArray(currentBrick,:);
% B.233 - Robot moves end-effector to [targetX, targetY, targetA, targetZ+0.1];

% B.24  - Run dropBrick():
% B.244 - Robot moves gripper down vertically onto tower with optional
%         fine-tuning.
% B.245 - Human enters keystroke commanding gripper to open.
% B.246 - Update brick array index.
currentBrick = currentBrick + 1;
% B.247 - Robot moves gripper up vertically so as not to collide with
%         tower.




