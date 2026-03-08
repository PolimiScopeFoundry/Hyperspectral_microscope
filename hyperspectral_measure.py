# -*- coding: utf-8 -*-
"""
Created on 22 Dec 2025

@authors: Martina Riva. Politecnico di Milano
"""
from ScopeFoundry import Measurement
from ScopeFoundry.helper_funcs import sibling_path, load_qt_ui_file
from ScopeFoundry import h5_io
import pyqtgraph as pg
from qtpy.QtWidgets import QFileDialog
import numpy as np
import os, time


class hyperMeasure(Measurement):
    
    name = "hyper"
    
    def setup(self):
        """
        Runs once during App initialization.
        This is the place to load a user interface file,
        define settings, and set up data structures.
        """
        
        self.ui_filename = sibling_path(__file__, "camera_with_plot.ui")
        self.ui = load_qt_ui_file(self.ui_filename) 
        
        self.settings.New('start_pos', dtype=float, unit='mm', initial=2.3, spinbox_decimals=4) 
        self.settings.New('step', dtype=float, unit='um', initial=40, spinbox_decimals=2) 
        self.settings.New('step_num', dtype=int, initial=50, vmin = 1) 
        self.settings.New('Load_positions', dtype=bool, initial=False)
        self.settings.New('motor_velocity', dtype = float, initial=0.125, unit='mm/s', spinbox_decimals=3)
        # self.add_operation('measure', self.measure)
        self.settings.New('camera_trigger', dtype=str, ro=0, choices = ['Internal', 'Edge', 'Trigger First', 'Software Trigger Edge', 'Software Trigger First'], initial = 'Internal')
    
        
        #dummy values for initialization; they are necessary for HDF5 files visualization in ImageJ 
        self.settings.New('xsampling', dtype=float, unit='um', initial=1.0) 
        self.settings.New('ysampling', dtype=float, unit='um', initial=1.0)
        self.settings.New('zsampling', dtype=float, unit='um', initial=1.0)
        
        self.auto_range = self.settings.New('auto_range', dtype=bool, initial=True)
        self.settings.New('auto_levels', dtype=bool, initial=True)
        self.settings.New('level_min', dtype=int, initial=60)
        self.settings.New('level_max', dtype=int, initial=4000)
        self.settings.New('save_h5', dtype=bool, initial=False)         
        self.settings.New('refresh_period',dtype = float, unit ='s', spinbox_decimals = 3, initial = 0.05, vmin = 0) 
        self.settings.New('posx', dtype=int, initial=800)       
        self.settings.New('posy', dtype=int, initial=600)

        self.image_gen = self.app.hardware['PVcamHW']
        self.stage = self.app.hardware['PI_HW']
        self.stage.settings['velocity'] = 5 #set high velocity for fast movement to initial position (IKO) 

        
        
    def setup_figure(self):
        """
        Runs once during App initialization, after setup()
        This is the place to make all graphical interface initializations,
        build plots, etc.
        """
        
        # connect ui widgets to measurement/hardware settings or functions
        self.ui.start_pushButton.clicked.connect(self.start)
        self.ui.interrupt_pushButton.clicked.connect(self.interrupt)
        self.settings.save_h5.connect_to_widget(self.ui.save_h5_checkBox)
        self.settings.auto_levels.connect_to_widget(self.ui.autoLevels_checkbox)
        self.auto_range.connect_to_widget(self.ui.autoRange_checkbox)
        self.settings.level_min.connect_to_widget(self.ui.min_doubleSpinBox) 
        self.settings.level_max.connect_to_widget(self.ui.max_doubleSpinBox)
        self.settings.posx.connect_to_widget(self.ui.posX)
        self.settings.posy.connect_to_widget(self.ui.posY)
        self.settings.Load_positions.add_listener(self.load_positions)
  
        # Set up pyqtgraph graph_layout in the UI
        self.imv = pg.ImageView()
        self.plot_graph = pg.plot(title='Interferogram')
        self.ui.imageLayout.addWidget(self.imv)
        colors = [(0, 0, 0),
                  (45, 5, 61),
                  (84, 42, 55),
                  (150, 87, 60),
                  (208, 171, 141),
                  (255, 255, 255)
                  ]
        cmap = pg.ColorMap(pos=np.linspace(0.0, 1.0, 6), color=colors)
        self.imv.setColorMap(cmap)
        # #Try new colormap
        # colors = [
        #     (0, 0, 0),     # black
        #     (0, 0, 255),   # blue
        #     (0, 150, 255),  # light blue
        #     (255, 0, 0)     # red
        # ]
        # cmap = pg.ColorMap(pos=[0.0, 0.25 0.5, 1.0], color=colors)
        self.ui.plotLayout.addWidget(self.plot_graph)
        self.ui.tabWidget.setCurrentIndex(0) # show the image tab by default
        self.time = []
        self.intensity = []

    def load_positions(self):
        if self.settings.Load_positions.val:
          filename, _ = QFileDialog.getOpenFileName(
            parent=self.ui,
            caption="Select position file",
            directory="",
            filter="Text files (*.txt);;CSV files (*.csv);;All files (*)"
            )

          if filename:
            print("Selected file:", filename)
            self.target_pos = np.loadtxt(filename, dtype=float, ndmin=2)[:,0] # load only the first column of the file
            print('Debugging: Loaded positions:', self.target_pos)

        else:
            # user cancelled → uncheck checkbox
            self.settings.Load_positions.update_value(False)
        
        
    def update_display(self):
        """
        Displays (plots) the numpy array self.buffer. 
        This function runs repeatedly and automatically during the measurement run.
        its update frequency is defined by self.display_update_period
        """
        
        self.stage.read_from_hardware() #without this line the stage settings are not updated during the measurement
        # self.image_gen.read_from_hardware() #with this line the camera does not acquire images
        
        self.display_update_period = self.settings['refresh_period'] 
       
        #length = self.image_gen.frame_num.val
        length = self.settings.step_num.val
        self.settings['progress'] = (self.frame_index +1) * 100/length
        
        
        
        if hasattr(self, 'img'):
            self.imv.setImage(self.img.T,
                                autoLevels = self.settings['auto_levels'],
                                autoRange = self.auto_range.val,
                                levelMode = 'mono'
                                )
            
            if self.settings['auto_levels']:
                lmin,lmax = self.imv.getHistogramWidget().getLevels()
                self.settings['level_min'] = lmin
                self.settings['level_max'] = lmax
            else:
                self.imv.setLevels( min= self.settings['level_min'],
                                    max= self.settings['level_max'])
                
            #self.plot_graph.setXRange(1, 10)
            #self.plot_graph.setYRange(20, 40)
            if self.settings['save_h5']:
                self.time.append(self.frame_index)
                self.intensity.append(self.img[self.settings.posx.val, self.settings.posy.val])
                #print(self.time)
                #print(self.intensity)
                self.plot_graph.plot(self.time, self.intensity, pen='r')
                #self.plot_graph.setData(np.array(self.time),np.array(self.intensity))
                
            
    
    def measure(self):
        print("Debugging: Starting Measurement")
        self.plot_graph.clear()
        self.time = []
        self.intensity = []
        self.image_gen.read_from_hardware()
        first_frame_acquired = False
        step_num  = self.settings.step_num.val # number of acquired frames equals the number of motor steps
        step = self.settings.step.val /1000 # step is in um
        starting_pos = self.settings.start_pos.val # starting position in mm

        self.stage.motor.set_velocity(5) # high velocity for fast movement toward initial position
        print('Debugging: Motor velocity:', self.stage.motor.get_velocity(), 'mm/s')

        if self.settings['camera_trigger'] == 'Internal':
            self.image_gen.settings['trigger_mode'] = 'Internal Trigger' 
            self.image_gen.settings['number_frames'] = 1 #1-frame sequence acquisition

            self.stage.motor.move_absolute(starting_pos)
            self.stage.motor.wait_on_target()
            print('Debugging: Initial motor position:', self.stage.motor.get_position())
            if self.settings.Load_positions.val: # if the user loaded a position file, use the positions in the file 
                target_pos = self.target_pos
                step_num = len(target_pos) # number of acquired frames equals the number of positions in the file
                self.settings.step_num.val = step_num
                starting_pos = target_pos[0] # move to the first position in the file
            else: #otherwise, calculate the target positions based on the starting position and step size
                target_pos = np.arange(starting_pos, starting_pos + step_num * step, step) 
                #final position starting_pos + step_num * step is not included in the target positions, 
                # but number of steps is equal to step_num 
        
            for frame_idx in range(step_num):

                self.stage.motor.move_absolute(target_pos[frame_idx]) 
                self.stage.motor.wait_on_target()
            
                current_pos = self.stage.motor.get_position()
                # print(f'Position at acquisition {frame_idx}:', current_pos)
                self.image_gen.cam.acq_start_seq(1)
                self.frame_index = frame_idx    
                self.img = self.image_gen.cam.get_nparray()        
                self.image_gen.cam.acq_stop()
                                
                if self.settings['save_h5']:
                    if not first_frame_acquired:
                        self.create_h5_file()
                        first_frame_acquired = True
                    self.image_h5[frame_idx,:,:] = self.img
                    self.positions_h5[frame_idx] = current_pos*1000 # convert to um
                    self.h5file.flush()
                
                if self.interrupt_measurement_called:
                    break
            

                self.stage.read_from_hardware()

        elif self.settings['camera_trigger'] == 'Edge':
            correction = 0.05 # to compensate the delay between trigger and movement start (in mm)
            ch = 1
            ch_tot = 4
            self.image_gen.settings['trigger_mode'] = 'Edge Trigger'
            self.image_gen.read_from_hardware()
            target_pos = starting_pos + (step_num-1) * step # final position

            
            if starting_pos > target_pos:
                correction = -correction
            self.stage.motor.move_absolute(starting_pos-correction) #move a bit before the starting position
            self.stage.motor.wait_on_target()
            self.set_motor_velocity() # set an appropriate velocity for the scan
            print('Debugging: Motor velocity:', self.stage.motor.get_velocity(), 'mm/s')

            #Acquisition with trigger
            self.image_gen.cam.acq_start_seq(step_num) #sequential acquisition of a fixed number of frames

            start_time=time.time()
            print('Acquisition started')
            self.stage.motor.trigger(step, starting_pos, target_pos, ch, ch_tot) 
            self.stage.motor.move_absolute(target_pos+correction) #move a bit before the starting position
            self.stage.motor.wait_on_target()
            end_time = time.time()
            print('Acquisition time:', end_time-start_time, 's')


            vettore_posizioni = np.linspace(starting_pos, target_pos, step_num)*1000 # convert to um
            
            #Create the h5 file and save the data
            if self.settings['save_h5']:
                self.create_h5_file()
                for frame_idx in range(0, step_num-1):     
                    self.image_h5[frame_idx,:,:] = self.image_gen.cam.get_nparray()
                    print('Debugging: Saving frame', frame_idx+1, 'dimensions:', self.image_h5[frame_idx,:,:].shape)
                    
                    self.frame_index = frame_idx
                    self.positions_h5[frame_idx] = vettore_posizioni[frame_idx]                   
                self.h5file.flush()
            self.image_gen.cam.acq_stop()
            print('Acquisition stopped')
            self.stage.motor.trigger_disable(ch_tot)
            self.image_gen.settings['trigger_mode'] = 'Internal Trigger'
            self.stage.settings['velocity'] = 5 #high velocity for fast movement 
            self.image_gen.read_from_hardware()
            self.stage.read_from_hardware()
        else:
            raise ValueError('Measurement trigger is not set on internal nor external')

            
    def run(self):
        """
        Runs when measurement is started. Runs in a separate thread from GUI.
        It should not update the graphical interface directly, and should only
        focus on data acquisition.
        """

        try: 

            #start the camera
            self.time = []
            self.intensity = []
        
            self.frame_index = -1
 
            self.image_gen.read_from_hardware()
            self.image_gen.settings['acquisition_mode'] == 'Continuous'
            print('Debugging: Check frame status:', self.image_gen.cam.cam.check_frame_status())
            self.image_gen.cam.acq_start() 

            while not self.interrupt_measurement_called:

                # print('Debugging: Check frame status:', self.image_gen.cam.cam.check_frame_status())
                self.img = self.image_gen.cam.get_nparray()
                if self.settings['save_h5']:
                    self.image_gen.cam.acq_stop()
                    self.measure()
                    break
                if self.interrupt_measurement_called:
                    break


        finally:
        
            self.image_gen.cam.acq_stop()
            
            if self.settings['save_h5'] and hasattr(self, 'h5file'):
                # make sure to close the data file
                self.h5file.close() 
                self.settings['save_h5'] = False
                    


    def set_motor_velocity(self):
        # %Version 1: user sets the motor velocity manually
        self.stage.settings['velocity'] = self.settings['motor_velocity']
        self.stage.read_from_hardware()
        W = self.image_gen.cam.roi[2] #width of the ROI
        H = self.image_gen.cam.roi[3] #height of the ROI
        Exp = self.image_gen.settings['exposure_time']*10**(-3) # in s

        # %Automatic calculation of the motor velocity with maximum frame rate
        if W<=3200 and W>2900:
            a = 1.029e+05
            b = -35.78
            c = -4.887

        elif W<=2900 and W>2600:
            a = 1.089e+05
            b = -42.89
            c = -0.969

        elif W<=2600 and W>=1:
            a = 1.085e+05
            b = -43.19
            c = 4.604


        readout = 1/(a/(H-b) + c) #processing time for the specified ROI
        frame_rate = 1/(Exp+readout) # maximum frame rate for internal trigger

        print('Debugging: Maximum frame rate for ROI (',self.image_gen.subarrayv.val, self.image_gen.subarrayh.val, ') and acquisition time', Exp, 'is',  frame_rate, 'Hz')

        self.stage.settings['velocity']=self.settings['step']*10**(-3)/(4*Exp) # in mm/s
        print('Debugging: Chosen motor velocity:', self.stage.settings['velocity'], 'mm/s')
        print('Debugging: required frame rate:', 1/(self.settings['step']*10**(-3)/self.stage.settings['velocity']), 'Hz')

        if 1/(self.settings['step']*10**(-3)/self.stage.settings['velocity']) > 0.85*frame_rate: # empirical threshold 
            print('Warning: the selected motor velocity is too high for the current camera settings')
            print('Maximum motor velocity for current camera settings is ', self.settings['step']*10**(-3)*frame_rate, 'mm/s')
            self.stage.settings['velocity'] = 0.75*self.settings['step']*10**(-3)*frame_rate #empirical threshold
            
        self.stage.read_from_hardware()
           

                
    def create_saving_directory(self):
        
        if not os.path.isdir(self.app.settings['save_dir']):
            os.makedirs(self.app.settings['save_dir'])
        
    
    def create_h5_file(self):                   
        self.create_saving_directory()
        # file name creation
        timestamp = time.strftime("%y%m%d_%H%M%S", time.localtime())
        sample = self.app.settings['sample']
        #sample_name = f'{timestamp}_{self.name}_{sample}.h5'
        if sample == '':
            sample_name = '_'.join([timestamp, self.name])
        else:
            sample_name = '_'.join([timestamp, self.name, sample])
        fname = os.path.join(self.app.settings['save_dir'], sample_name + '.h5')
        
        self.h5file = h5_io.h5_base_file(app=self.app, measurement=self, fname = fname)
        self.h5_group = h5_io.h5_create_measurement_group(measurement=self, h5group=self.h5file)
        
        img_size = self.img.shape
        dtype=self.img.dtype
        
        if self.settings['camera_trigger'] == 'Internal':
            length = self.settings.step_num.val
        elif self.settings['camera_trigger'] == 'Edge': #with trigger there is a frame more acquired
            length = self.settings.step_num.val+1
        
        self.image_h5 = self.h5_group.create_dataset(name  = 't0/c0/image', 
                                                  shape = [length, img_size[0], img_size[1]],
                                                  dtype = dtype)
        self.image_h5.attrs['element_size_um'] =  [self.settings['zsampling'],self.settings['ysampling'],self.settings['xsampling']]
        
        self.positions_h5 = self.h5_group.create_dataset(name  = 't0/c0/position_mm', 
                                                  shape = [length],
                                                  dtype = np.float32)
                   

    
