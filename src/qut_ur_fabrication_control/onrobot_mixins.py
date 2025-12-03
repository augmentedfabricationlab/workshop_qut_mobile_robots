

class OnRobotMixins:
    def fgp_grip(self, width, force, speed, indent=1):
        '''
        Docstring for fgp20_grip
        
        :param self: Description
        :param width: Description
        :param force: Description
        :param speed: Description
        :param indent: Description
        '''
        return self.add_line(f"fgp_grip(width={width}, force={force}, speed={speed})", indent=indent)
    
    def fgp_release(self, width, speed, indent=1):
        '''
        Docstring for fgp20_release
        
        :param self: Description
        :param width: Description
        :param speed: Description
        :param indent: Description
        '''
        return self.add_line(f"fgp_release(width={width}, speed={speed})", indent=indent)