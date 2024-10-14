function [angle] = ComputeAngle(A,B,C)
%    for i = 1:length(points)
%        x10 = points(1,1)-origin(1,1);
%        y10 = points(1,2)-origin(1,2);
%        x20 = points(i,1)-origin(1,1);
%        y20 =points(i,2)-origin(1,2); 
%        angle(i) = pi-atan2(abs(x10*y20-x20*y10),x10*y10+x20*y20);

AB = A-B;
CB = C-B;
ang_rad = atan2(abs(det([AB;CB])),dot(AB,CB)); % Angle in radians
%ang_rad1 = atan(tan(ang_rad) * cos(deg2rad(26)));

% if C(2)>A(2)
%     ang_rad = 2*pi - ang_rad;
% end

angle = rad2deg(ang_rad);

if C(2)>A(2)
    angle = -angle;
     if C(1)<B(1)
         angle = -180-angle;
     end
else
    if C(1)<B(1)
    angle = 180-angle;
    end
end

% if C(2)>A(2)
%     angle = -angle;
% end


end