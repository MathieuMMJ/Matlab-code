close all
% load the data, then store every frame data into a pcd file to create a 3D
% points cloud of each frame

S = load("matlab_data.mat");
C = fieldnames(S);
reference = frame1 ;
roll_list = zeros(1,530);
pitch_list = zeros(1,530);
yaw_list = zeros(1,530);
translation_list = zeros(1,530) ; 

for k = 1:numel(C)
    data = S.(C{k}) ; 
    ptCloud = pointCloud(data(:,1:3)) ; 
    pcwrite(ptCloud,"frame"+int2str(k) + ".pcd")
    %disp("frame" +int2str(k) + "=" )
    %print the matrix that compose the frame variable
    %disp(S.(C{k}))
    current = data ; 
    rotation = procrustes(reference,current) ;
    translation_matrix(reference,current,procrustes(reference,current)) ;

    % Get the angles of rotation, thanks to the rotation matrix

    pitch = (atan(rotation(3,2)/rotation(3,3)))*180/pi ; % x rotation
    yaw = asin(rotation(3,1))*180/pi    ;            % y rotation
    roll = atan(rotation(2,1)/rotation(1,1))*180/pi ;    % z rotation
    
    roll_list(1,k) = roll ;
    pitch_list(1,k) = pitch;
    yaw_list(1,k) = yaw;
    TransMtx = translation_matrix(reference,current,procrustes(reference,current))
    translation_list(1,k) = sqrt(power(TransMtx(1,1),2)+power(TransMtx(2,1),2)+power(TransMtx(3,1),2)) ; 
end

disp(translation_list)
%Data treatment to compute rotation and translation matrix between every
%frame

% We need to take a reference, it will be the first frame of the clip

figure

x1 = plot(yaw_list);
title('Yaw')
xlabel('Frames')
ylabel('Yaw angle in degrees')

figure
x2 = plot(pitch_list);
title('Pitch')
xlabel('Frames')
ylabel('Pitch angle in degrees')

figure
x3 = plot(roll_list);
title('Roll')
xlabel('Frames')
ylabel('Roll angle in degrees')

figure
x4 = plot(translation_list)
title('Translation')
xlabel('Frames')
ylabel('Translation')

% Functions
function R = procrustes(reference,current)
    x= size(current) ;
    C = transpose(current-fill(centroid(current),x(1)))*(reference-fill(centroid(reference),x(1))) ; 
    [U S V] = svd(C);
    R = U * transpose(V);
end

function T = translation_matrix(reference,current,rotation_matrix)
    T = transpose(centroid(current))-rotation_matrix*transpose(centroid(reference));
end 

function cent = centroid(matrix) 
    cent = [mean(matrix(:,1)) mean(matrix(:,2)) mean(matrix(:,3))] ; 
end

function fi = fill(vect,size) 
    fi = zeros(size,3) ; 
    for i=1:size
        fi(i,1) = vect(1); 
        fi(i,2) = vect(2);
        fi(i,3) = vect(3);
    end
end


