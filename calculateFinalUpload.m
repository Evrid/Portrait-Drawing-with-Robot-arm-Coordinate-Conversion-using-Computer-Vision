%% Tutorial for Sketching Algorithm

%First in the same folder of this program, put the image we want process
%there, then create a test.csv file (by create test.txt first then change test.txt to test.csv) 
% then we enter the origion (by go to the center of the paper then write down the x and y value) and ScaleValue and LiftPenHeight we want, then it ask us to
%ask us to select a file, we change file type to ALL FILES and select a
%image located within the same folder of the MATLAB program, then the
%output will go to test.csv

%% Initialization 
clear; close all; clc; rng('default');   %clear previous stuff

ScaleValue=300;  %scale value for x and y, default scale of 1 have x and y value around 0.3
LiftPenHeight=10;  %set value for z which is the hight we lift pen

DistanceWhichLiftPen=0.002*ScaleValue;   
%don't suggest change this one (even change only change 0.002)
%it is for if two points are further than this then we will lift pen (give z value) to the end and start point

origin = [144 134]; % origion of the graph, write as [x y]
delta = 0.001; % the distance apart we want record (the smaller the more detailed drawing is)

boundary =18;    %we can change to a different value but this works fine

ZOffset=113.215;  %we can enter the offset value for z here, if enter -100 then all z value will be 100 less

Yaw=-176.629;  %W value
Roll=-2.447;   %P value 
Pitch=112.57;  %R value

%% Acquire an image from user selection


I = imread(uigetfile);   %the image need to be in the same folder as this MATLAB program (different folder won't work)

%after it ask you to select a file, file type change to ALL FILES, then can
%then can see images and select

%I = imread('Capture-removebg-preview.png');  


%% Read an image file 

figure; imshow(I);  %show the image we selected


%% Edge detection 

[BW,thre] = edge(rgb2gray(I),'Canny',[0.0813 0.1281]); 
figure, imshow(BW);
%% Skeletonization 

BWseg = bwmorph(BW,'skel',Inf);
figure, imshow(BWseg);


%% Detect start and end points 

[B,L] = bwboundaries(BWseg,'noholes');
imshow(label2rgb(L, @jet, [.5 .5 .5]))
hold on
for k = 1:length(B)
   boundary = B{k};
   plot(boundary(:,2), boundary(:,1), 'LineWidth', 3)
end


%% Find the end point 

edgeind = find(all(circshift(boundary,1)==circshift(boundary,-1),2),1);

%% Sort the points 

boundary = circshift(boundary,-edgeind+1);
boundary = boundary(1:ceil(end/2),:);

%% Create path for each edge 

for i = 1:length(B)
    boundary = B{i};
    edgeind = find(all(circshift(boundary,1)==circshift(boundary,-1),2),1);

    if ~isempty(edgeind)
        boundary = circshift(boundary,-edgeind+1);
        boundary = boundary(1:ceil(end/2),:);
    end
    B{i} = boundary;
end
imshow(label2rgb(L, @jet, [.5 .5 .5]))
hold on
for k = 1:length(B)
   boundary = B{k};
   plot(boundary(:,2), boundary(:,1), 'LineWidth', 3)
end

%% Convert to 3D coordinate 

B2 = B;
figure;
for i = 1:length(B)
    b = B{i};
    bx = -b(:,2)*delta*ScaleValue;  
    by = b(:,1)*delta*ScaleValue;   
    bz = zeros(length(bx),1);
    B2{i} = [bx by bz];    %B2 is a matrix, i loop over and set each bx,by,bz of the matrix
    %plot3(bx,by,bz); hold on;
end
grid on;

%% Find the center point of the graph, then shift to 0,0 then shift to our center point

XYZmatrix = cell2mat(B2);   %if we don't convert then we need write using writecell and the output is a mess
%therefore we convert it to matrix then export, the result is much cleaner

XYZmatrixRowSize=size(XYZmatrix,1);   %we get matrix's first dimension (row) size used later for for loop

SumOfEachColumn=sum(XYZmatrix,1);  %S = sum(A,dim) returns the sum along dimension dim. For example, if A is a matrix, then sum(A,1) is a column vector containing the sum of each column
OldCenterPointX=SumOfEachColumn(1)/size(XYZmatrix,1);  %size(XYZmatrix,1) is number of rows of XYZ matrix
OldCenterPointY=SumOfEachColumn(2)/size(XYZmatrix,1);
%first column is sum of x values, second column is sum of y values

for iii = 1:XYZmatrixRowSize
      XYZmatrix(iii,1)=XYZmatrix(iii,1)-OldCenterPointX+origin(1);   
      XYZmatrix(iii,2)=XYZmatrix(iii,2)-OldCenterPointY+origin(2);  
end

plot3(XYZmatrix(:,1),XYZmatrix(:,2),XYZmatrix(:,3)); 

%% Edit z axis if we start draw new line (like human drawing not connected lines)

    previousX=0;   %after we store bx and by, we check it with new bx and by value, first time set to 0
    previousY=0;

for ii = 1:XYZmatrixRowSize

    ThisX =XYZmatrix(ii,1);   %extract the first folumn (which is x) of every XYZmatrix using for loop
    ThisY=XYZmatrix(ii,2);    %extract the second folumn (which is y) of every XYZmatrix using for loop


   if abs(ThisX-previousX)>DistanceWhichLiftPen || abs(ThisY-previousY)>DistanceWhichLiftPen  
       %if the move distence is too much between two points then lift pen to move robot arm there
      XYZmatrix(ii,3)=ZOffset+LiftPenHeight;   %we change the z value here for how high the pen lift
   else
      XYZmatrix(ii,3)=ZOffset+0;  %no lifting means drawing
   end
   
    previousX=ThisX;   %after we store bx and by, we set it as new previous value, which will be compared when run for loop again
    previousY=ThisY;
end


%%  Replace the previous stroke (end stroke of previous drawing)'s z axis to a higher value because we want lift at end and beginning of draw line


 %the end point of previous stroke need lift we need lift up pen already,
 %so replace previous stroke z value to 20

[NewStrokeRows,~]=find(XYZmatrix(:,3)==ZOffset+LiftPenHeight);    % A(:,3) means: all rows third column, which is z value, equal to LiftPenHeight, then we locate the row
% [row,col] = find(___) returns the row and column subscripts of each nonzero element in array X using any of the input arguments in previous syntaxes.
%we fild the third column which has value of LiftPenHeight, the position is
%stored in array NewStrokeRows

for xx =1: numel(NewStrokeRows)    %n = numel( A ) returns the number of elements, n , in array A
  IndividualRow=NewStrokeRows(xx);   %give individual NewStrokeRows elements during the loop
  EndStrokeRow=int32(IndividualRow)-1;   %must convert to int32 to use in matrix assign value later
 if EndStrokeRow>1   %only if end stroke is later than first line then we edit it (because there is no line 0)
  XYZmatrix(EndStrokeRow,3)=ZOffset+LiftPenHeight;
 end
end


%% For yaw (w) roll (P) and pitch (R)
%we need put yaw pitch roll value because if not it will say uninitialized
%when using on Fanuc robot

WPRMatrix(1:XYZmatrixRowSize,1) = Yaw;   %make a number of row same as XYZmatrix and first column equals yaw
WPRMatrix(1:XYZmatrixRowSize,2) = Roll;  %second column equals pitch
WPRMatrix(1:XYZmatrixRowSize,3) = Pitch;  %third column equals Pitch

XYZWPRMatrix = [XYZmatrix WPRMatrix];  %combine x,y,z and yaw pitch and roll to one sum matrix
%% output to csv file

writematrix(XYZWPRMatrix,'test.csv');     %write matrix to test.csv file 

%we can also use only    writematrix(XYZMatrix,'test.csv'); 
%we can also create a test.txt file in the same folder as this program then
%save it there, just change above code to writematrix(XYZWPRMatrix,'test.txt');  

%Most codes written by Kevin Li (CC BY-SA 2.0)
%Inspired by Tohru Kikawada (2022). Portrait Drawing using Computer Vision and Robot Manipulator (https://www.mathworks.com/matlabcentral/fileexchange/67926-portrait-drawing-using-computer-vision-and-robot-manipulator), MATLAB Central File Exchange. Retrieved June 20, 2022


