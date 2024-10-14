clc;
clear all;
close all;

%% SELECT VIDEO FILE
filename = "file_path.mp4";
start_time = 3;
duration = 11;
flip = false;
minpoints_quality = 0.1;
maxbidirect_error = 200;
output_fps = 60;
angle_limits = 50;

%% SELECT TRACKING PARAMETERS
video_to_track = VideoReader(filename);
videoPlayer = vision.VideoPlayer('Position',[100,100,680,520]);
objectFrame = read(video_to_track,round(start_time*video_to_track.FrameRate));
if flip==true
    objectFrame = flipdim(objectFrame,2); 
end

%% SELECT TRACKING POINTS
figure; imshow(objectFrame);
origin=round(getPosition(impoint));
objectRegion=round(getPosition(imrect));
objectImage = insertShape(objectFrame,'Rectangle',objectRegion,'Color','red');
figure;
imshow(objectImage);

%% VIDEO SETUP
title('Red box shows object region');
points = detectMinEigenFeatures(rgb2gray(objectFrame),'ROI',objectRegion,'MinQuality', minpoints_quality);
pointImage = insertMarker(objectFrame,points.Location,'+','Color','white');
figure;
imshow(pointImage);
title('Detected interest points');
tracker = vision.PointTracker('MaxBidirectionalError',maxbidirect_error);
initialize(tracker,points.Location,objectFrame);
v = VideoWriter(cd + "\Experimental\Camera Tracking\videos_out\" + extractBefore(filename,".") + " - TRACK ",'MPEG-4');
v.FrameRate = output_fps;
framekeyjump = round(video_to_track.FrameRate/v.FrameRate);
open(v);
close all;

%% TRACK THE VIDEO
i=1;j=1;
while (hasFrame(video_to_track) && i<duration*video_to_track.FrameRate)
     
      %Track the points for each frame
      frame = readFrame(video_to_track);
      if flip==true
         objectFrame = flipdim(objectFrame,2); 
      end 
      [points,validity] = tracker(frame);
      
      %Save points and angle history
      points_buffer(i,:)= [mean(points(:,1)) mean(points(:,2))];
      angle_buffer(i,:) = ComputeAngle([origin(1)+400 origin(2)],origin,points_buffer(i,:));
      dist_buffer(i,:) = ComputeDist(origin,points_buffer(i,:));
     
      %Print on screen
      framebw = rgb2gray(frame);
      out = insertMarker(framebw,points_buffer(i,:),'x','color','green','size',20);
      out = insertMarker(out,points_buffer,'o','color','white','size',1);
      out = insertShape(out,'line',[origin points_buffer(1,:); origin points_buffer(i,:)],'LineWidth',2,'Color', {'white','blue'});
      out = insertMarker(out,origin,'x','color','blue','size',5);
      out = insertMarker(out,origin,'o','color','blue','size',5); 
      
      out = insertShape(out,'line',[origin points_buffer(i,:)],'LineWidth',2,'Color', 'white');

      i=i+1;
      videoPlayer(out);
      
      %Save video file
      if(j>framekeyjump)
          j=1;
      end

      if(j==1)
        writeVideo(v,out);
        j = j + 1;
      else
        j = j + 1;
      end

end
close(v);
release(videoPlayer);

%% PLOT RESULTS
%video plot
plot_video = VideoWriter(cd + "Experimental\Camera Tracking\videos_out\" + extractBefore(filename,".") + " - PLOT",'MPEG-4');
plot_video.FrameRate = v.FrameRate;
angle_idx = 1:framekeyjump:length(angle_buffer)-1;
angle_buffer_framejump = angle_buffer(angle_idx);
t_framejump = 0:1/v.FrameRate:(length(angle_buffer_framejump)-1)/plot_video.FrameRate; t_framejump=t_framejump';
AnimatePlot(t_framejump,angle_buffer_framejump,'Tracked Angle','time [s]','angle [deg]', [0 t_framejump(end)], [-angle_limits, angle_limits],'',plot_video)
%image plot
t = 0:1/video_to_track.FrameRate:(length(angle_buffer)-1)/video_to_track.FrameRate; t=t';
plot(t,angle_buffer,'LineWidth', 2)
xlabel('time [s]')
ylabel('angle [deg]')
ylim([-angle_limits angle_limits])
grid on

save(cd + "Experimental\Camera Tracking\data_out\" + extractBefore(filename,".") + " - DATA.mat","angle_buffer","t");