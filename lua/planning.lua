function sysCall_init()
        
    -- Get some handles:
    joint1=sim.getObjectHandle('UR5_joint1')
    joint2=sim.getObjectHandle('UR5_joint2')
    joint3=sim.getObjectHandle('UR5_joint3')
    joint4=sim.getObjectHandle('UR5_joint4')
    joint5=sim.getObjectHandle('UR5_joint5')
    joint6=sim.getObjectHandle('UR5_joint6')

    jointHandles={joint1,joint2,joint3,joint4,joint5,joint6}

    ikGroup=sim.getIkGroupHandle('UR5')
    ikTip=sim.getObjectHandle('UR5_tip')
    ikTarget=sim.getObjectHandle('UR5_target')
    collisionPairs={sim.getCollectionHandle('UR5'),sim.getCollectionHandle('not_UR5')}

    minConfigsForIkPath=2
end

generateIkPath=function(goalPose)
    -- Generates (if possible) a linear, collision free path between a robot config and a target pose
    sim.setObjectMatrix(ikTarget,-1,goalPose)
    local c=sim.generateIkPath(ikGroup,jointHandles,minConfigsForIkPath,collisionPairs)
    return c
end

findIkPath=function(inInts,inFloats,inStrings,inBuffer)
    local goalPose = {}
    for i=1,12,1 do goalPose[i]=inFloats[i] end

    local path=generateIkPath(goalPose)
    if not path then
        return {0},{},{},''
    end
    return {#path},path,{},''
end

findCollisionFreeConfig=function(task)
    -- Here we search for a robot configuration..
    -- 1. ..that matches the desired pose (task.goalPose)
    -- 2. ..that does not collide in that configuration
    sim.setObjectMatrix(task.ikTarget,-1,task.goalPose)
    -- Here we check point 1 & 2:
    local c=sim.getConfigForTipPose(task.ikGroup,task.jh,0.65,20,nil,task.collisionPairs)
    return c
end

findSeveralCollisionFreeConfigs=function(task)
    -- Here we search for several robot configurations...
    -- 1. ..that matches the desired pose (task.goalPose)
    -- 2. ..that does not collide in that configuration
    sim.setObjectMatrix(task.ikTarget,-1,task.goalPose)
    local cs={}
    local l={}
    for i=1,task.maxTrialsForConfigSearch,1 do
        local c=findCollisionFreeConfig(task)
        if c then
            local dist=getConfigConfigDistance(task.currentState,c,task.metric)
            local p=0
            local same=false
            for j=1,#l,1 do
                if math.abs(l[j]-dist)<0.001 then
                    -- we might have the exact same config. Avoid that
                    same=true
                    for k=1,#task.jh,1 do
                        if math.abs(cs[j][k]-c[k])>0.01 then
                            same=false
                            break
                        end
                    end
                end
                if same then
                    break
                end
            end
            if not same then
                cs[#cs+1]=c
                l[#l+1]=dist
            end
        end
        if #l>=task.maxConfigsForDesiredPose then
            break
        end
    end
    if #cs==0 then
        cs=nil
    end
    return cs
end

-----------------------------------------------------------------------------------------------


getConfig=function(jointHandles)
    local config={}
    for i=1,#jointHandles,1 do
        config[i]=sim.getJointPosition(jointHandles[i])
    end
    return config
end

setConfig=function(jointHandles,config)
    if config then
        for i=1,#jointHandles,1 do
            sim.setJointPosition(jointHandles[i],config[i])
        end
    end
end

getConfigConfigDistance=function(config1,config2,metric)
    local d=0
    for i=1,#config1,1 do
        local dx=(config1[i]-config2[i])*metric[i]
        d=d+dx*dx
    end
    return math.sqrt(d)
end

getPathLength=function(path,metric)
    local d=0
    local l=#metric
    local pc=#path/l
    for i=1,pc-1,1 do
        local config1={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6]}
        local config2={path[i*l+1],path[i*l+2],path[i*l+3],path[i*l+4],path[i*l+5],path[i*l+6]}
        d=d+getConfigConfigDistance(config1,config2,metric)
    end
    return d
end

findCollisionFreeConfig=function(task)
    -- Here we search for a robot configuration..
    -- 1. ..that matches the desired pose (task.goalPose)
    -- 2. ..that does not collide in that configuration
    sim.setObjectMatrix(task.ikTarget,-1,task.goalPose)
    -- Here we check point 1 & 2:
    local c=sim.getConfigForTipPose(task.ikGroup,task.jh,0.65,20,nil,task.collisionPairs)
    return c
end

findSeveralCollisionFreeConfigs=function(task)
    -- Here we search for several robot configurations...
    -- 1. ..that matches the desired pose (task.goalPose)
    -- 2. ..that does not collide in that configuration
    sim.setObjectMatrix(task.ikTarget,-1,task.goalPose)
    local cs={}
    local l={}
    for i=1,task.maxTrialsForConfigSearch,1 do
        local c=findCollisionFreeConfig(task)
        if c then
            local dist=getConfigConfigDistance(task.currentState,c,task.metric)
            local p=0
            local same=false
            for j=1,#l,1 do
                if math.abs(l[j]-dist)<0.001 then
                    -- we might have the exact same config. Avoid that
                    same=true
                    for k=1,#task.jh,1 do
                        if math.abs(cs[j][k]-c[k])>0.01 then
                            same=false
                            break
                        end
                    end
                end
                if same then
                    break
                end
            end
            if not same then
                cs[#cs+1]=c
                l[#l+1]=dist
            end
        end
        if #l>=task.maxConfigsForDesiredPose then
            break
        end
    end
    if #cs==0 then
        cs=nil
    end
    return cs
end

findOnePath=function(task,goalConfigs)
    local omplTask=simOMPL.createTask('omplTask')
    simOMPL.setAlgorithm(omplTask,simOMPL.Algorithm.SBL)
    local j1_space=simOMPL.createStateSpace('j1_space',simOMPL.StateSpaceType.joint_position,task.jh[1],{-180*math.pi/180},{180*math.pi/180},1)
    local j2_space=simOMPL.createStateSpace('j2_space',simOMPL.StateSpaceType.joint_position,task.jh[2],{-90*math.pi/180},{150*math.pi/180},2)
    local j3_space=simOMPL.createStateSpace('j3_space',simOMPL.StateSpaceType.joint_position,task.jh[3],{-180*math.pi/180},{75*math.pi/180},3)
    local j4_space=simOMPL.createStateSpace('j4_space',simOMPL.StateSpaceType.joint_position,task.jh[4],{-400*math.pi/180},{400*math.pi/180},0)
    local j5_space=simOMPL.createStateSpace('j5_space',simOMPL.StateSpaceType.joint_position,task.jh[5],{-125*math.pi/180},{120*math.pi/180},0)
    local j6_space=simOMPL.createStateSpace('j6_space',simOMPL.StateSpaceType.joint_position,task.jh[6],{-400*math.pi/180},{400*math.pi/180},0)
    simOMPL.setStateSpace(omplTask,{j1_space,j2_space,j3_space,j4_space,j5_space,j6_space})
    simOMPL.setCollisionPairs(omplTask,task.collisionPairs)
    simOMPL.setStartState(omplTask,task.currentState)
    simOMPL.setGoalState(omplTask,goalConfigs[1])
    for i=2,#goalConfigs,1 do
        simOMPL.addGoalState(omplTask,goalConfigs[i])
    end
    simOMPL.setStateValidityCheckingResolution(omplTask,0.001)
    local path=nil
    local l=999999999999
    for i=1,task.searchCountPerConfig,1 do
        local res,_path=simOMPL.compute(omplTask,4,-1,task.minConfigsForPathPlanningPath)
        if res and _path then
            local _l=getPathLength(_path,task.metric)
            if _l<l then
                l=_l
                path=_path
            end
        end
    end
    simOMPL.destroyTask(omplTask)
    return path,l
end

findShortestPath=function(task,goalConfigs)
    -- This function will search for several paths between the specified start configuration,
    -- and several of the specified goal configurations. The shortest path will be returned
    local path=findOnePath(task,goalConfigs)
    return path
end

getReversedPath=function(path)
    local retPath={}
    local ptCnt=#path/6
    for i=ptCnt,1,-1 do
        for j=1,6,1 do
            retPath[#retPath+1]=path[(i-1)*6+j]
        end
    end
    return retPath
end

findPathForPoseGoal=function(inInts,inFloats,inStrings,inBuffer)
    local task={}
    task.robotHandle=inInts[1]
    local collisionChecking=inInts[2]>0
    task.minConfigsForIkPath=inInts[3]
    task.minConfigsForPathPlanningPath=inInts[4]
    task.maxConfigsForDesiredPose=inInts[5]
    task.maxTrialsForConfigSearch=inInts[6]
    task.searchCountPerConfig=inInts[7]

    local currentState={}
    for i=1,6,1 do currentState[i]=inFloats[i] end
    task.currentState=currentState
    local goalPose={}
    for i=1,12,1 do goalPose[i]=inFloats[i+6] end
    task.goalPose=goalPose

    local fullRobotName=sim.getObjectName(task.robotHandle)

    local jh={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        jh[i]=sim.getObjectHandle('UR5_joint'..i)
    end
    task.jh=jh

    task.ikGroup=sim.getIkGroupHandle('UR5')
    task.ikTip=sim.getObjectHandle('UR5_tip')
    task.ikTarget=sim.getObjectHandle('UR5_target')
    task.collisionPairs={}
    if collisionChecking then
        task.collisionPairs={sim.getCollectionHandle('UR5'),sim.getCollectionHandle('not_UR5')}
    end

    task.metric={0.5,1,1,0.15,0.1,0.1}

    local configs=findSeveralCollisionFreeConfigs(task)
    if configs then
        local path=findShortestPath(task,configs)
        print(path)
        if path then
            local fkTrajLength=#path/#jh
            return {fkTrajLength},path,{},''
        end
    end
    return {0},{},{},''
end
