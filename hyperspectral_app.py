# -*- coding: utf-8 -*-
"""
Created on 22 Dec 2025

@authors: Martina Riva. Politecnico di Milano
"""
from ScopeFoundry import BaseMicroscopeApp

def add_path(path):
    import sys
    import os
    # add path to ospath list, assuming that the path is in a sybling folder
    from os.path import dirname
    sys.path.append(os.path.abspath(os.path.join(dirname(dirname(__file__)),path)))



class hyper_app(BaseMicroscopeApp):
    
    name = 'hyper_app'
    
    def setup(self):
        
        #Add hardware components
        print("Adding Hardware Components")

        add_path('Teledyne_ScopeFoundry') 
        from CameraHW import PVcamHW
        self.add_hardware(PVcamHW(self))
        
        add_path('IKO_ScopeFoundry')
        from IKO_Hardware import IKO_HW
        self.add_hardware(IKO_HW(self, ip = "10.0.0.100", port = 701))

        
        # Add measurement components
        print("Create Measurement objects")
        from hyperspectral_measure import hyperMeasure
        self.add_measurement(hyperMeasure(self))
        
        #For ScopeFoundry release 2.0.2 comment these lines:
        #self.ui.show()
        #self.ui.activateWindow()

if __name__ == '__main__':
    
    import sys
    import os

    app = hyper_app(sys.argv)
    
    # Load settings from ini file in Settings, within the same folder as this script
    current_dir = os.path.dirname(os.path.abspath(__file__))
    setting_dir = os.path.join(current_dir, 'Settings', 'settings.ini')
    app.settings_load_ini(setting_dir)

    for hc_name, hc in app.hardware.items():
        hc.settings['connected'] = True    # connect all the hardwares  automatically
    
    
    sys.exit(app.exec_())
