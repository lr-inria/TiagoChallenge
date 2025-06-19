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




% -------------------------------------------------------------------------
% A.3
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




% -------------------------------------------------------------------------
% A.4
% Initialize robot state to RECEIVE.





% Create array index counter for brick drop locations.

% Start event listener (for keystrokes).

% Get QR code location.

% Define target square location?

% Define brick hand-off location and pose.
% -> Move end-effector to brick hand-off location and pose.
% -> Beep to ask operator to open gripper via keystroke?
% -> Operator positions block in gripper.
% -> Operator closes gripper via keystroke.
% -> Switch to elbow-up configuration.
% -> Exit

% Get end-effector location for next brick drop.

% Move end-effector to brick drop location.

% Drop the brick.

% Increment the brick counter.

% Repeat until last brick location array entry has been sourced or operator
% hits the 'q' (for quit) key.

