function sysCall_init()
    xml = [[
  <ui title="Quadricopter position" closeable="true" resizable="true" activate="false">
  <group layout="grid" flat="true">
          <!-- <button text ="Start" on-click="start" id="1" />  -->
      </group>
      <group layout="grid" flat="true">
          <button text ="First Tunnel" on-click="straightTunnel" id="2" />
          <button text ="Second Tunnel" on-click="secondTunnel" id="3"  />
          <button text ="Third Tunnel" on-click="thirdTunnel" id="4"  />
          <button text ="Fourth Tunnel" on-click="fourthTunnel" id="5"  />
      </group>
      <label text="" style="* {margin-left: 300px;}"/>
  </ui>
  ]]
          ui=simUI.create(xml)
  end
  
  function sysCall_actuation()
      -- put your actuation code here
  end
  
  function sysCall_sensing()
      -- put your sensing code here
  end
  
  function sysCall_cleanup()
      -- do some clean-up here
  end
  
  function start()
  
  end
  
  -- See the user manual or the available code snippets for additional callback functions and details
  function straightTunnel()
      copter=sim.getObjectHandle("Quadcopter")
      target=sim.getObjectHandle("Quadcopter_target")
      objs=sim.getObjectsInTree(copter,sim.handle_all,0)
      currentOrCopter=sim.getObjectOrientation(copter,-1)
      currentOrTarget=sim.getObjectOrientation(target,-1)
      newPositionCopter={-13.75,0.49,0.8} --sis ir vietas kur maina koordinates, un  new Position target ari. Pielikti klat offsetti, jo target ar dronu nedaudz offsetoti bija.
      newPositionTarget={newPositionCopter[1]-0.006,newPositionCopter[2]-0.0029,newPositionCopter[3]+0.1707}
      for i=1,#objs,1 do
          sim.resetDynamicObject(objs[i])
      end
      sim.resetDynamicObject(copter)
      sim.resetDynamicObject(target)
      sim.setObjectPosition(copter,-1,newPositionCopter)
      sim.setObjectPosition(target,-1,newPositionTarget)
  end

  function secondTunnel()
      copter=sim.getObjectHandle("Quadcopter")
      target=sim.getObjectHandle("Quadcopter_target")
      objs=sim.getObjectsInTree(copter,sim.handle_all,0)
      currentOrCopter=sim.getObjectOrientation(copter,-1)
      currentOrTarget=sim.getObjectOrientation(target,-1)
      newPositionCopter={-4.39,6.59,0.8}
      newPositionTarget={newPositionCopter[1]-0.006,newPositionCopter[2]-0.0029,newPositionCopter[3]+0.1707}
      for i=1,#objs,1 do
          sim.resetDynamicObject(objs[i])
      end
      sim.resetDynamicObject(copter)
      sim.resetDynamicObject(target)
      sim.setObjectPosition(copter,-1,newPositionCopter)
      sim.setObjectPosition(target,-1,newPositionTarget)
  end

  function thirdTunnel()
      copter=sim.getObjectHandle("Quadcopter")
      target=sim.getObjectHandle("Quadcopter_target")
      objs=sim.getObjectsInTree(copter,sim.handle_all,0)
      currentOrCopter=sim.getObjectOrientation(copter,-1)
      currentOrTarget=sim.getObjectOrientation(target,-1)
      newPositionCopter={4.40,-17.246,0.8}
      newPositionTarget={newPositionCopter[1]-0.006,newPositionCopter[2]-0.0029,newPositionCopter[3]+0.1707}
      for i=1,#objs,1 do
          sim.resetDynamicObject(objs[i])
      end
      sim.resetDynamicObject(copter)
      sim.resetDynamicObject(target)
      sim.setObjectPosition(copter,-1,newPositionCopter)
      sim.setObjectPosition(target,-1,newPositionTarget)
  end

  function fourthTunnel()
      copter=sim.getObjectHandle("Quadcopter")
      target=sim.getObjectHandle("Quadcopter_target")
      objs=sim.getObjectsInTree(copter,sim.handle_all,0)
      currentOrCopter=sim.getObjectOrientation(copter,-1)
      currentOrTarget=sim.getObjectOrientation(target,-1)
      newPositionCopter={-3.95,-18.0,0.8}
      newPositionTarget={newPositionCopter[1]-0.006,newPositionCopter[2]-0.0029,newPositionCopter[3]+0.1707}
      for i=1,#objs,1 do
          sim.resetDynamicObject(objs[i])
      end
      sim.resetDynamicObject(copter)
      sim.resetDynamicObject(target)
      sim.setObjectPosition(copter,-1,newPositionCopter)
      sim.setObjectPosition(target,-1,newPositionTarget)
  end