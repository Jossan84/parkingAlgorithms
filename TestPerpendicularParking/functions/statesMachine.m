function [ directionOut, pathOffset, statusOut ] = statesMachine( collision, parkSpotFound, offset, statusIn )
    
    persistent collisionAux;
    if isempty(collisionAux)
        collisionAux = collision;
    end
    
    switch statusIn
        case 1 % Path Following
            
            statusOut = 1;
            
            if collision == 1
                statusOut = 2;
                directionOut = 'Backward';
                pathOffset = 0;
                return;
            end
            
            directionOut = 'Forward';
            pathOffset = 0;
            
            if parkSpotFound == 1
               pathOffset = offset;               
            end
                 
        case 2 % Parking Maneuver
            
            pathOffset = 0;
            statusOut = 2;
            directionOut = 'Backward';
                        
        otherwise
            
    end

    collisionAux = collision;
end

