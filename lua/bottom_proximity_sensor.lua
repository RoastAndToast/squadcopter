function sysCall_init()
    -- get sensor object
end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    -- get distance from sensor
    -- do something with distance (send to py, etc.)
    -- print(distance)
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- returns a minimum found distance from a given proximity sensor.
-- returns -1 if nearest obstacle is out of sensor range.
function getDistance(sensor)
    local detected, distance
    detected, distance = sim.readProximitySensor(sensor)
    if (detected == 1) then
        return distance
    end
    return -1
end