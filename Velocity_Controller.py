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
