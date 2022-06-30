import math
import XInput 

def joyStickNeedleControl():
        state = XInput.get_state(0)
        thumb = XInput.get_thumb_values(state)
        z = thumb[1][1] 
        pitch = thumb[0][0] 
        yaw = thumb[0][1]  
        vel = [0, 0, z, 0, pitch, yaw] 
        return vel 
        

    # Close the window and quit.
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
while True: 
    print(str(joyStickNeedleControl()))