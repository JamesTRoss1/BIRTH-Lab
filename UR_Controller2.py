import math
import UR_Move_RTDE
import XInput 

def joyStickNeedleControl(x,y,z,roll,pitch,yaw):
    #pygame.init()
    #Arduino.initialize()
    # Loop until the user clicks the close button.
    done = False
    # Initialize the joystickss
    UR_Move_RTDE.startup()
    orientScale = 1
    moveScale = 1  
    previousUp = 0 
    previousDown = 0 
    previousRight = 0
    previousLeft = 0 
    z = 0 
    previousZ = False 
    while not done:
        state = XInput.get_state(0)
        trigger = XInput.get_trigger_values(state)
        #trigger index 1 is 1 unit forward 
        #trigger index 0 is 1 unit backwards 
        #range is 0 to 1
        right = trigger.get("DPAD_RIGHT")
        left = trigger.get("DPAD_LEFT")
        if int(right) == 1 and int(left) == 1:
            moveScale = moveScale
        elif (int(right) == 1) and not previousRight: 
            moveScale += 1 
            previousRight = True 
            print("Move Scale: " + str(moveScale)) 
        elif (int(left) == 1) and not previousLeft: 
            previousLeft = True 
            moveScale -= 1 
            print("Move Scale: " + str(moveScale)) 
        if moveScale < 1: 
            moveScale = 1 
        if int(right) == 0: 
            previousRight = False 
        if int(left) == 0:
            previousLeft = False 
        if int(trigger[1]) == 1 and int(trigger[0]) == 1: 
            previousZ = False
            z = z 
        elif int(trigger[1]) == 1 and not previousZ: 
            previousZ = True
            z += 1 * moveScale  
        elif int(trigger[0]) == 1 and not previousZ:
            previousZ = True
            z -= 1 * moveScale
        else: 
            previousZ = False
            z = z 
        trigger = dict(XInput.get_button_values(state))
        done = int(trigger.get('Y'))
        up = trigger.get("DPAD_UP")
        down = trigger.get("DPAD_DOWN") 
        if int(up) == 1 and int(down) == 1:
            orientScale = orientScale
        elif (int(up) == 1) and not previousUp: 
            orientScale += 1
            previousUp = True 
            print("Orientation Scale: " + str(orientScale)) 
        elif (int(down) == 1) and not previousDown: 
            previousDown = True 
            orientScale -= 1 
            print("Orientation Scale: " + str(orientScale))
        if orientScale < 1: 
            orientScale = 1 
        if int(up) == 0: 
            previousUp = False 
        if int(down) == 0:
            previousDown = False 
        #pitch and yaw 
        #maybe not analog inputs? 
        thumb = XInput.get_thumb_values(state)
        pitch = thumb[1][1] 
        yaw = thumb[0][1]  
        print("Scale: " + str(orientScale))
        coord = [float(0),float(0),float(z),float(orientScale * pitch),float(orientScale * yaw),0.0]
        print("Coord: " + str(coord))
        coord = [float(0),float(0),float(z),float(orientScale * pitch),float(orientScale * yaw),0.0]
        if all(float(c) == 0.0 for c in coord):
            print("Not moving at all")
            #UR_Move_RTDE.move([0.0,0.0,0.0,0.0,0.0,0.0])
            UR_Move_RTDE.stopMoving()
        else: 
            for c in range(0, len(coord)): 
                coord[c] = coord[c] * -1
            print("Moving") 
            UR_Move_RTDE.move(coord)
        if(done):
            print("Stopping Robot")
            done=True
            UR_Move_RTDE.quit()
            break 

    # Close the window and quit.
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
joyStickNeedleControl(0,0,0,0,0,0)