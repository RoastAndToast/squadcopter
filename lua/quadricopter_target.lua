function sysCall_init()
    -- do some initialization here
    delta_mov = 0.0075
end

function sysCall_actuation()
    -- put your actuation code here
    message,data,data2=sim.getSimulatorMessage()
    thisHandle=sim.getObjectHandle('Quadcopter_target')
    pos=sim.getObjectPosition(thisHandle, -1)
    x=pos[1]
    y=pos[2]
    z=pos[3]
    newPos={x,y,z}
    if (message==sim.message_keypress) then
        
        if (data[1]==100) then -- rightkey
            y=y-delta_mov
        end
        if (data[1]==97) then -- left key
            y=y+delta_mov
        end
        if (data[1]==119) then -- up key
            x=x+delta_mov
        end
        if (data[1]==115) then -- down key
            x=x-delta_mov
        end
        if (data[1]==122) then -- down key
            z=z-delta_mov
        end
        if (data[1]==120) then -- down key
            z=z+delta_mov
        end

        newPos={x,y,z}
        sim.setObjectPosition(thisHandle, -1, newPos)
    end
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
