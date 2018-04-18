-- Anil Ozdemir
-- 02Feb2018
-- The University of Sheffield

-- This is the Epuck principal control script. It is threaded
threadFunction=function()
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        sim.setJointTargetVelocity(leftMotor,36)
        sim.setJointTargetVelocity(rightMotor,32)
        
-- Image Processing Part.
    sim.handleVisionSensor(ePuckCam) -- the image processing camera is handled explicitely, since we do not need to execute that command at each simulation pass
    result,t0,t1=sim.readVisionSensor(ePuckCam) -- Here we read the image processing camera!
    
-- The e-puck robot has Blob Detection filter. The code provided below get useful information
-- regarding blobs detected, such as amount, size, position, etc.

    if (t1) then -- (if Detection is successful) in t1 we should have the blob information if the camera was set-up correctly  
        blobCount=t1[1]
        dataSizePerBlob=t1[2]
        lowestYofDetection=100
        -- Now we go through all blobs:
        for i=1,blobCount,1 do
            blobSize=t1[2+(i-1)*dataSizePerBlob+1]
            blobOrientation=t1[2+(i-1)*dataSizePerBlob+2]
            xPos=t1[2+(i-1)*dataSizePerBlob+3]
            yPos=t1[2+(i-1)*dataSizePerBlob+4]
            blobBoxDimensions={t1[2+(i-1)*dataSizePerBlob+5],t1[2+(i-1)*dataSizePerBlob+6]}
            -- now do something with the blob information
        end
        sim.addStatusbarMessage(xPos)
        sim.addStatusbarMessage(yPos)

        if (xPos<0.5) then
        sim.setJointTargetVelocity(leftMotor,16)
        sim.setJointTargetVelocity(rightMotor,16*(1+0.5-xPos))
        end

        if (xPos>0.5) then
        sim.setJointTargetVelocity(leftMotor,16*(1+xPos-0.5))
        sim.setJointTargetVelocity(rightMotor,16)
        end

    end

-- This part of code gets the proximity sensor readings. 
        s=sim.getObjectSizeFactor(bodyElements) -- make sure that if we scale the robot during simulation, other values are scaled too!
        noDetectionDistance=0.05*s
        proxSensDist={noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance}
        for i=1,8,1 do
                velLeft = 6.24
                velRight= 6.24
            res,dist=sim.readProximitySensor(proxSens[i])
            if (res>0) and (dist<noDetectionDistance) then
                proxSensDist[i]=dist
                if(proxSensDist[3]>noDetectionDistance or proxSensDist[4]<noDetectionDistance) then
                sim.setJointTargetVelocity(leftMotor,-velLeft)
                sim.setJointTargetVelocity(rightMotor,velRight)
                sim.switchThread()

                elseif(proxSensDist[3]<noDetectionDistance or proxSensDist[4]>noDetectionDistance) then
                sim.setJointTargetVelocity(leftMotor,velLeft)
                sim.setJointTargetVelocity(rightMotor,-velRight)
                sim.switchThread()

                elseif(proxSensDist[1]<noDetectionDistance or proxSensDist[2]>proxSensDist[1]) then
                sim.setJointTargetVelocity(leftMotor,velLeft)
                sim.setJointTargetVelocity(rightMotor,-velRight)
                sim.switchThread()

                elseif(proxSensDist[6]<noDetectionDistance or proxSensDist[6]>proxSensDist[5]) then
                sim.setJointTargetVelocity(leftMotor,-velLeft)
                sim.setJointTargetVelocity(rightMotor,velRight)
                sim.switchThread()
                end
            end
        end

-- This part of the code sends the velocity values to the motors.
-- An example were given below.
        velLeft=6.24 
        velRight=6.24 
        sim.setJointTargetVelocity(leftMotor,velLeft)
        sim.setJointTargetVelocity(rightMotor,velRight)
        sim.switchThread() -- Don't waste too much time in here (simulation time will anyway only change in next thread switch)
    end
end

-- These are handles, you do not need to change here. (If you need e.g. bluetooth, you can add it here)

sim.setThreadSwitchTiming(200) -- We will manually switch in the main loop
bodyElements=sim.getObjectHandle('ePuck_bodyElements')
leftMotor=sim.getObjectHandle('ePuck_leftJoint')
rightMotor=sim.getObjectHandle('ePuck_rightJoint')
ePuck=sim.getObjectHandle('ePuck')
ePuckCam=sim.getObjectHandle('ePuck_camera')
ePuckBase=sim.getObjectHandle('ePuck_base')
ledLight=sim.getObjectHandle('ePuck_ledLight')

proxSens={-1,-1,-1,-1,-1,-1,-1,-1}
for i=1,8,1 do
    proxSens[i]=sim.getObjectHandle('ePuck_proxSensor'..i)
end

maxVel = 6.24 -- Maximum wheel speeds in rad/s.

res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
    sim.addStatusbarMessage('Lua runtime error: '..err)
end

