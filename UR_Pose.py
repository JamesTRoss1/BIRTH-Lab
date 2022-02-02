import msvcrt
from os import abort
import UR_Move_RTDE
string = ""
num = 1
done = False 
pose = []
value = ""
UR_Move_RTDE.startup()
print("Now Ready To Move Robot \n")
while not done:
    if msvcrt.kbhit():
        char = msvcrt.getch().decode()
        string = string + str(char)
        print("Raw String: " + str(string) + "\n")
        if(str(char) == " "): 
            print("Number of Values: " + str(num) + "(Need 6 Units [x, y, z, rx, ry, rz]) \n")
            num = int(num) + 1
        if(str(char) == "n"):
            print("Enter all new values")
            num = 1 
            string = ""
        if(str(char) == "\r"):
            num = 1  
            for st in string: 
                if st == " " or st == "," or st == "\r" or st == "\n" or st == "" or len(st) == 0:
                    try: 
                        if(len(str(value)) == 0):
                            pass
                        else:
                            pose.append(float(value))
                    except Exception as e:
                        print("Cannot append pose")
                        print(str(e.with_traceback()))
                        abort() 
                    finally: 
                        value = ""
                else:
                    value = str(value) + str(st)
            print("Position: " + str(pose))
            if len(pose) == 6:
                print("Moving Robot")
                #negative everything bc moving needle 
                UR_Move_RTDE.moveByPose(pose)
                pose.clear()
                string = ""
                value = "" 
                print("\n" + "Move Complete" + "\n")
            else: 
                print("Must send 6 values to robot. Try again\n\n")
                pose.clear() 
                string = ""
                value = ""
        if(char == "q"):
            done = True 
UR_Move_RTDE.quit()