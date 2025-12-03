
from ur_fabrication_control.direct_control import URScript
from onrobot_mixins import OnRobotMixins

class URScript_OnRobot(URScript, OnRobotMixins):
    pass


if __name__=="__main__":
    urscript = URScript_OnRobot("192.168.0.210", 30002)
    urscript.start()
    urscript.fgp_grip(70, 80, 10)
    urscript.fgp_release(155, 10)
    urscript.end()
    urscript.generate()

    urscript.send_script()
