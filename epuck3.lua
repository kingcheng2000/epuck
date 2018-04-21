-- Author *****
-- 02Feb2018
-- The Uni**** of Sh*****

-- This is the Epuck principal control script. It is threaded

    -- Braitenberg weights for the 4 front prox sensors (avoidance):
    braitFrontSens_leftMotor={1,2,-2,-1}
    -- Braitenberg weights for the 2 side prox sensors (following):
    braitSideSens_leftMotor={-1,0}
    -- Braitenberg weights for the 8 sensors (following):
    braitAllSensAvoid_leftMotor={0,0.5,1,-1,-0.5,-0.5,0,0}
    braitAllSensAvoid_rightMotor={-0.5,-0.5,-1,1,0.5,0,0,0}
threadFunction=function()
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do  
       
        for z =0,10,1 do 
        sim.setJointTargetVelocity(leftMotor,maxVel)
        sim.setJointTargetVelocity(rightMotor,-maxVel)
       end 
        sim.setJointTargetVelocity(leftMotor,maxVel)
        sim.setJointTargetVelocity(rightMotor,maxVel)
        
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
       local MatrixBlod ={}
        for i=1,blobCount,1 do
            blobSize=t1[2+(i-1)*dataSizePerBlob+1]
            blobOrientation=t1[2+(i-1)*dataSizePerBlob+2]
            xPos=t1[2+(i-1)*dataSizePerBlob+3]
            yPos=t1[2+(i-1)*dataSizePerBlob+4]
            blobBoxDimensions={t1[2+(i-1)*dataSizePerBlob+5],t1[2+(i-1)*dataSizePerBlob+6]}
            -- now do something with the blob information
            -- table operation
            --table.insert(MatrixBlod,yPos)
             MatrixBlod[i] = {xPos,yPos}
          --  print(MatrixBlod[i])
             
        end
-- PID control section
   -- Construct pid table
     local pid = {SetPos, ActualPos, err, err_last,Kp,Ki,Kd,integral,voltage}         
   -- Initialise the value 
        print("PID_init begin \n");
        pid.SetPos=0.5;
        pid.ActualPos=0.0;
        pid.err=0.0;
        pid.err_last=0.0;
        pid.voltage=0.0;
        pid.integral=0.0;
        pid.Kp=0.2;
        pid.Ki=0.015;
        pid.Kd=0.2;
        print("PID_init end \n")
   -- algorithm for PID control

 
        pid.err=math.abs(pid.SetPos-pid.ActualPos)
        pid.integral=pid.integral+pid.err
        ActualSpeed=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
        pid.err_last=pid.err;
    --    pid.ActualSpeed=pid.voltage*1.0;
   

    
-- Minimum vertical position of blod in camera,
            function table_min(t)
              local mn=nil;
              for k, v in pairs(t) do
                if(mn==nil) then
                  mn=v
                end
                if mn > v then
                  mn = v
                end
              end
              return mn
            end
-- get minimal yPos

 --local min = (MatrixBlod)
 -- print(min)

        --print(t1[5])
     k1= 1.3
     if (t1[5]~=nil) then 
        if (t1[5]<0.5) then
        sim.setJointTargetVelocity(leftMotor,maxVel*(1-(0.5-t1[5])))
        sim.setJointTargetVelocity(rightMotor,maxVel)
        end

        if (t1[5]>0.5) then
        sim.setJointTargetVelocity(leftMotor,maxVel)
        sim.setJointTargetVelocity(rightMotor,maxVel*(1-(t1[5]-0.5)))
        end
--[[
        if (0.45<t1[5] and t1[5]<0.55) then
        sim.setJointTargetVelocity(leftMotor,maxVel)
        sim.setJointTargetVelocity(rightMotor,maxVel)
        end
]]
     end

   end
-- No bold will turn around 
--[[
        if (blobSize < 0.01)then 
                sim.setJointTargetVelocity(leftMotor,maxVel*0.9)
                sim.setJointTargetVelocity(rightMotor,maxVel)
        end
]]
        if (blobCount == 0)then 
                sim.setJointTargetVelocity(leftMotor,-maxVel*0.5)
                sim.setJointTargetVelocity(rightMotor,maxVel)
        end


-- This part of code gets the proximity sensor readings. 
        s=sim.getObjectSizeFactor(bodyElements) -- make sure that if we scale the robot during simulation, other values are scaled too!
        noDetectionDistance=0.05*s
        proxSensDist={noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance}
        for i=1,8,1 do
            res,dist=sim.readProximitySensor(proxSens[i])
            if (res>0) and (dist<noDetectionDistance) then
                proxSensDist[i]=dist
            end
        end

                velRight=maxVel
                velLeft=maxVel
                if (proxSensDist[2]+proxSensDist[3]+proxSensDist[4]+proxSensDist[5]==noDetectionDistance*4) then
                    -- Nothing in front. Maybe we have an obstacle on the side, in which case we wanna keep a constant distance with it:
                    if (proxSensDist[1]>0.25*noDetectionDistance) then
                        velLeft=velLeft+maxVel*braitSideSens_leftMotor[1]*(1-(proxSensDist[1]/noDetectionDistance))
                        velRight=velRight+maxVel*braitSideSens_leftMotor[2]*(1-(proxSensDist[1]/noDetectionDistance))
                    end
                    if (proxSensDist[6]>0.25*noDetectionDistance) then
                        velLeft=velLeft+maxVel*braitSideSens_leftMotor[2]*(1-(proxSensDist[6]/noDetectionDistance))
                        velRight=velRight+maxVel*braitSideSens_leftMotor[1]*(1-(proxSensDist[6]/noDetectionDistance))
                    end
                else
                    -- Obstacle in front. Use Braitenberg to avoid it
                    for i=1,4,1 do
                        velLeft=velLeft+maxVel*braitFrontSens_leftMotor[i]*(1-(proxSensDist[1+i]/noDetectionDistance))
                        velRight=velRight+maxVel*braitFrontSens_leftMotor[5-i]*(1-(proxSensDist[1+i]/noDetectionDistance))
                    end
                end


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

maxVel = 30 -- Maximum wheel speeds in rad/s.

res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
    sim.addStatusbarMessage('Lua runtime error: '..err)
end
