function sysCall_init()
        
    -- Get some handles:
    joint1=sim.getObjectHandle('UR5_joint1')
    joint2=sim.getObjectHandle('UR5_joint2')
    joint3=sim.getObjectHandle('UR5_joint3')
    joint4=sim.getObjectHandle('UR5_joint4')
    joint5=sim.getObjectHandle('UR5_joint5')
    joint6=sim.getObjectHandle('UR5_joint6')
    
    -- Enable an image publisher and subscriber:
    pub=simROS.advertise('/sim/joint_state', 'sensor_msgs/JointState')
    simROS.publisherTreatUInt8ArrayAsString(pub) -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua) 
end

function sysCall_sensing()
	joint_position={1.0,1.0,1.0,1.0,1.0,1.0}

	joint_position[1]=sim.getJointPosition(joint1)
	joint_position[2]=sim.getJointPosition(joint2)
	joint_position[3]=sim.getJointPosition(joint3)
	joint_position[4]=sim.getJointPosition(joint4)
	joint_position[5]=sim.getJointPosition(joint6)
	joint_position[6]=sim.getJointPosition(joint6)

	d={}
	d['header']={seq=0,stamp=simROS.getTime(),frame_id="a"}
	d['position']=joint_position
	simROS.publish(pub,d)
end

