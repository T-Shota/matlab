% *takeSnapshot* captures the output for the HTML publishing report.
function I = takeSnapshot_J(frame, sensor, sensorOut)

    % Unpack the inputs
    leftEgoBoundary  = sensorOut.leftEgoBoundary;
    rightEgoBoundary = sensorOut.rightEgoBoundary;
    locations        = sensorOut.vehicleLocations;
    xVehiclePoints   = sensorOut.xVehiclePoints;
    bboxes           = sensorOut.vehicleBoxes;
      
    frameWithOverlays = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
    frameWithOverlays = insertLaneBoundary(frameWithOverlays, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');
    frameWithOverlays = insertVehicleDetections_J(frameWithOverlays, locations, bboxes);
   
    I = frameWithOverlays;

end
