function sysCall_init() 
    particlesAreVisible=false
    simulateParticles=true
    fakeShadow=false
    
    particleCountPerSecond=430
    particleSize=0.005
    particleDensity=8500
    particleScatteringAngle=30
    particleLifeTime=0.5
    maxParticleCount=50
 
    -- Detatch the manipulation sphere:
    targetObj=sim.getObjectHandle('Quadcopter_target')
    sim.setObjectParent(targetObj,-1,true)
 
    -- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example
 
    propellerHandles={}
    jointHandles={}
    particleObjects={-1,-1,-1,-1}
    local ttype=sim.particle_roughspheres+sim.particle_cyclic+sim.particle_respondable1to4+sim.particle_respondable5to8+sim.particle_ignoresgravity
    if not particlesAreVisible then
        ttype=ttype+sim.particle_invisible
    end
    for i=1,4,1 do
        propellerHandles[i]=sim.getObjectHandle('Quadcopter_propeller_respondable'..i)
        jointHandles[i]=sim.getObjectHandle('Quadcopter_propeller_joint'..i)
        if simulateParticles then
            particleObjects[i]=sim.addParticleObject(ttype,particleSize,particleDensity,{2,1,0.2,3,0.4},particleLifeTime,maxParticleCount,{0.3,0.7,1})
        end
    end
    heli=sim.getObjectHandle(sim.handle_self)
    d=sim.getObjectHandle('Quadcopter_base')
 
    init_pos = sim.getObjectPosition(d, -1)
    inti_ori = sim.getObjectOrientation(d, -1)
 
    pParam=2
    iParam=0
    dParam=0
    vParam=-2
 
    cumul=0
    lastE=0
    pAlphaE=0
    pBetaE=0
    psp2=0
    psp1=0
 
    prevEuler=0
    --if true, then can be moved with keyboard after unchecking sets current pos as (0, 0)
    manualMode = false
 
    --Target XYZ. Target position can be relative, but then
    --initial pos is relative meaning 0, 0. Z pos is always absolute.
    --targetPos = {0, 0, 0.3}
    targetPos = {0, 0, 1}
    targetRot = 0
    
    --Proximity sensor
    bottom_proximity_handle = sim.getScriptHandle('bottom_proximity_sensor')
    bottom_proximity = sim.getObjectHandle('bottom_proximity_sensor')
    dist = 0
    
    -- xml for control ui
    xml = [[
        <ui closable="false" placement="relative" position="500,300" layout="hbox">
        <group layout="vbox">
        <checkbox text="Manual mode(keyboard)" on-change="ManualChange" />
        </group>
        </ui>
    ]]
    ui = simUI.create(xml)
    --
    -- Prepare 2 floating views with the camera views:
    floorCam=sim.getObjectHandle('Quadricopter_floorCamera')
    frontCam=sim.getObjectHandle('Quadricopter_frontCamera')
    floorView=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    frontView=sim.floatingViewAdd(0.7,0.9,0.2,0.2,0)
    sim.adjustView(floorView,floorCam,64)
    sim.adjustView(frontView,frontCam,64)
 
    if (fakeShadow) then
        shadowCont=sim.addDrawingObject(sim.drawing_discpoints+sim.drawing_cyclic+sim.drawing_25percenttransparency+sim.drawing_50percenttransparency+sim.drawing_itemsizes,0.2,0,-1,1)
    end
end
 
function sysCall_cleanup() 
    if (fakeShadow) then
        sim.removeDrawingObject(shadowCont)
    end
    for i=1,#particleObjects,1 do
        sim.removeParticleObject(particleObjects[i])
    end
end 
 
function sysCall_sensing()
    dist = sim.callScriptFunction('getDistance@bottom_proximity_sensor', bottom_proximity_handle, bottom_proximity)
end
 
function sysCall_actuation() 
    pos=sim.getObjectPosition(d,-1)
    if (fakeShadow) then
        itemData={pos[1],pos[2],0.002,0,0,1,0.2}
        sim.addDrawingObjectItem(shadowCont,itemData)
    end
 
    --Relative position for debugging
    pos=sim.getObjectPosition(d,-1)
    pos_relative_x = pos[1] - init_pos[1]
    pos_relative_y = pos[2] - init_pos[2]
    pos_relative_z = pos[3]
    x_str = string.format("%.2f", pos_relative_x)
    y_str = string.format(" %.2f", pos_relative_y)
    z_str = string.format(" %.2f", pos_relative_z)
    d_str = string.format(" %.2f", dist)
    -- print(x_str, y_str, z_str, d_str)
 
    --change to pos form flow thing
    currPos = {pos_relative_x, pos_relative_y, pos_relative_z}
    currOri = sim.getObjectOrientation(d, -1)[3] - inti_ori[3]
    
    if(manualMode == 2) then
    else
        targetAbsPos = {init_pos[1] + targetPos[1], init_pos[2] + targetPos[2], targetPos[3]}
        sim.setObjectPosition(targetObj, -1, targetAbsPos)
    end
 
    sp = sim.getObjectPosition(targetObj,d)
 
    -- Vertical control:
    l=sim.getVelocity(heli)
    e = (sp[3]-dist)
    cumul=cumul+e
    pv=pParam*e
    thrust=5.45+pv+iParam*cumul+dParam*(e-lastE)+l[3]*vParam
    lastE=e
    
    -- Horizontal control: 
    mH= sim.getObjectMatrix(d,-1)
    vx={1,0,0}
    vx=sim.multiplyVector(mH,vx)
    vy={0,1,0}
    vy=sim.multiplyVector(mH,vy)
    alphaE= (vy[3]-mH[12])
    alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE)
    betaE= (vx[3]-mH[12])
    betaCorr=-0.25*betaE-2.1*(betaE-pBetaE)
    pAlphaE=alphaE
    pBetaE=betaE
    alphaCorr=alphaCorr+sp[2]*0.005+1*(sp[2]-psp2)
    betaCorr=betaCorr-sp[1]*0.005-1*(sp[1]-psp1)
    psp2=sp[2]
    psp1=sp[1]
    
    -- Rotational control:
    euler=currOri-targetRot
    rotCorr=euler*0.1+2*(euler-prevEuler)
    prevEuler=euler
    
    -- Decide of the motor velocities:
    handlePropeller(1,thrust*(1-alphaCorr+betaCorr+rotCorr))
    handlePropeller(2,thrust*(1-alphaCorr-betaCorr-rotCorr))
    handlePropeller(3,thrust*(1+alphaCorr-betaCorr+rotCorr))
    handlePropeller(4,thrust*(1+alphaCorr+betaCorr-rotCorr))
end 
 
function setTagetPosition(targetPosPy)
    -- targetPos = {targetPosPy[1], targetPosPy[2], targetPosPy[3]}
    print('setTargetPosition')
    targetPos[1] = targetPosPy
end
 
function handlePropeller(index,particleVelocity)
    propellerRespondable=propellerHandles[index]
    propellerJoint=jointHandles[index]
    propeller=sim.getObjectParent(propellerRespondable)
    particleObject=particleObjects[index]
    maxParticleDeviation=math.tan(particleScatteringAngle*0.5*math.pi/180)*particleVelocity
    notFullParticles=0
 
    local t=sim.getSimulationTime()
    sim.setJointPosition(propellerJoint,t*10)
    ts=sim.getSimulationTimeStep()
    
    m=sim.getObjectMatrix(propeller,-1)
    particleCnt=0
    pos={0,0,0}
    dir={0,0,1}
    
    requiredParticleCnt=particleCountPerSecond*ts+notFullParticles
    notFullParticles=requiredParticleCnt % 1
    requiredParticleCnt=math.floor(requiredParticleCnt)
    while (particleCnt<requiredParticleCnt) do
        -- we want a uniform distribution:
        x=(math.random()-0.5)*2
        y=(math.random()-0.5)*2
        if (x*x+y*y<=1) then
            if (simulateParticles) then
                pos[1]=x*0.08
                pos[2]=y*0.08
                pos[3]=-particleSize*0.6
                dir[1]=pos[1]+(math.random()-0.5)*maxParticleDeviation*2
                dir[2]=pos[2]+(math.random()-0.5)*maxParticleDeviation*2
                dir[3]=pos[3]-particleVelocity*(1+0.2*(math.random()-0.5))
                pos=sim.multiplyVector(m,pos)
                dir=sim.multiplyVector(m,dir)
                itemData={pos[1],pos[2],pos[3],dir[1],dir[2],dir[3]}
                sim.addParticleObjectItem(particleObject,itemData)
            end
            particleCnt=particleCnt+1
        end
    end
    -- Apply a reactive force onto the body:
    totalExertedForce=particleCnt*particleDensity*particleVelocity*math.pi*particleSize*particleSize*particleSize/(6*ts)
    force={0,0,totalExertedForce}
    m[4]=0
    m[8]=0
    m[12]=0
    force=sim.multiplyVector(m,force)
    local rotDir=1-math.mod(index,2)*2
    torque={0,0,rotDir*0.002*particleVelocity}
    torque=sim.multiplyVector(m,torque)
    sim.addForceAndTorque(propellerRespondable,force,torque)
end
 
--[[functions for sliders
function XChange(uiHandle, id, newValue)
    targetPos[1] = newValue / 500
    simUI.setLabelText(ui, 2002, "X="..newValue/500)
end
 
function YChange(uiHandle, id, newValue)
    targetPos[2] = newValue / 500
    simUI.setLabelText(ui, 2003, "Y="..newValue/500)
end
 
function ZChange(uiHandle, id, newValue)
    targetPos[3] = newValue / 500
    simUI.setLabelText(ui, 2004, "Z="..newValue/500)
end
 
function RotChange(uiHandle, id, newValue)
    targetRot = newValue / 500
    simUI.setLabelText(ui, 2005, "Rot="..newValue/500)
end--]]
function ManualChange(uiHandle, id, newValue)
    manualMode = newValue
end