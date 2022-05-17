function [ directionOut, pathOffset, statusOut ] = statesMachine( collision, parkSpotFound, offset, statusIn )
    
    persistent collisionAux;
    if isempty(collisionAux)
        collisionAux = collision;
    end
    
    switch statusIn
        case 1 % Path Following
            
            statusOut = 1;
            directionOut = 'Forward';
            pathOffset = 0;
            
            if parkSpotFound == 1
                statusOut = 2;
                directionOut = 'Backward';
                pathOffset = 0;
                return;
            end
                             
        case 2 % Parking Maneuver
            
            pathOffset = 0;
            statusOut = 2;
            directionOut = 'Backward'; 
            
            if collision == 1 
                statusOut = 3;
                directionOut = 'Fordward';
                pathOffset = 0;
                return;
            end
            
        case 3 % Aligning
            statusOut = 3;
            directionOut = 'Fordward';
            pathOffset = 0;
                
            if collision == 1 
                statusOut = 4;
                directionOut = 'Fordward';
                pathOffset = 0;
                return;
            end 
                
        case 4 % End Parking Maneuver
            statusOut = 4;
            directionOut = 'Fordward';
            pathOffset = 0; 
            
        otherwise            
    end

    collisionAux = collision;
end

