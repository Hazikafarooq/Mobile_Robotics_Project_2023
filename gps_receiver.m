function [localOrigin,gpsFs] = gps_receiver(localOrigin,gpsFs)

gps = gpsSensor('UpdateRate', gpsFs, 'ReferenceFrame', 'ENU');
gps.ReferenceLocation = localOrigin;
gps.DecayFactor = 0.5;                % Random walk noise parameter 
gps.HorizontalPositionAccuracy = 1.0;   
gps.VerticalPositionAccuracy =  1.0;
gps.VelocityAccuracy = 0.1;