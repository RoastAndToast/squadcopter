function sysCall_init()
    -- do some initialization here
end

function sysCall_actuation()
    -- put your actuation code here
    message,data,data2=sim.getSimulatorMessage()
    thisHandle=sim.getObjectHandle('Quadricopter_target')
    pos=sim.getObjectPosition(thisHandle, -1)
    x=pos[1]
    y=pos[2]
    z=pos[3]
    newPos={x,y,z}
    if (message==sim.message_keypress) then
        
        if (data[1]==100) then -- rightkey
            y=y-0.05
        end
        if (data[1]==97) then -- left key
            y=y+0.05
        end
        if (data[1]==119) then -- up key
            x=x+0.05
        end
        if (data[1]==115) then -- down key
            x=x-0.05
        end
        if (data[1]==122) then -- down key
            z=z-0.05
        end
        if (data[1]==120) then -- down key
            z=z+0.05
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
