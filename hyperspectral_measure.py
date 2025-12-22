# -*- coding: utf-8 -*-
"""
Created on wed Jun 16 01:33:32 2021

@authors: Andrea Bassi, Martina Riva, Antonio Composto. Politecnico di Milano
"""
from ScopeFoundry import Measurement
from ScopeFoundry.helper_funcs import sibling_path, load_qt_ui_file
from ScopeFoundry import h5_io
import pyqtgraph as pg
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
        self.settings.New('motor_velocity', dtype = float, initial=0.125, unit='mm/s', spinbox_decimals=2)
        #self.add_operation('measure', self.measure)
        self.settings.New('camera_trigger', dtype=str, ro=0, choices = ["internal", "external"], initial = 'internal')
    
        
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

        self.image_gen = self.app.hardware['HamamatsuHardware']
        self.stage = self.app.hardware['PI_CG_HW']
        self.stage.settings['velocity'] = 5 

        
        
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
        self.ui.plotLayout.addWidget(self.plot_graph)
        self.time = []
        self.intensity = []

        
        
    def update_display(self):
        """
        Displays (plots) the numpy array self.buffer. 
        This function runs repeatedly and automatically during the measurement run.
        its update frequency is defined by self.display_update_period
        """
        
        self.stage.read_from_hardware()
        self.image_gen.read_from_hardware()
        
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

        self.plot_graph.clear()
        self.time = []
        self.intensity = []
        self.image_gen.read_from_hardware()
        first_frame_acquired = False
        step_num  = self.settings.step_num.val # number of acquired frames equals the number of motor steps
        step = self.settings.step.val /1000 # step is in um
        self.starting_pos = starting_pos = self.settings.start_pos.val
    
        #Move motor to the starting position
        #velocity = self.stage.motor.get_velocity()
        #self.stage.motor.set_velocity(5) # high velocity for fast movement toward initial position
        print('Debugging: Motor velocity:', self.stage.motor.get_velocity(), 'mm/s')
        self.stage.motor.move_absolute(starting_pos)
        self.stage.motor.wait_on_target()
        print('Debugging: Initial motor position:', self.stage.motor.get_position())

        if self.settings['camera_trigger'] == 'internal':
            self.image_gen.settings['acquisition_mode'] = 'fixed_length'
            self.image_gen.settings['number_frames'] = 1
            self.stage.settings['velocity'] = 5
        
            for frame_idx in range(step_num):
            
                current_pos = self.stage.motor.get_position()
                print(f'Position at acquisition {frame_idx}:', current_pos)
                self.image_gen.hamamatsu.startAcquisition()
                self.frame_index = frame_idx    
                [frame, dims] = self.image_gen.hamamatsu.getLastFrame()        
                self.np_data = frame.getData()
                self.img = np.reshape(self.np_data,(self.eff_subarrayv, self.eff_subarrayh))
                self.image_gen.hamamatsu.stopAcquisition()
                                
                if self.settings['save_h5']:
                    if not first_frame_acquired:
                        self.create_h5_file()
                        first_frame_acquired = True
                    self.image_h5[frame_idx,:,:] = self.img
                    self.positions_h5[frame_idx] = current_pos*1000 # convert to um
                    self.h5file.flush()
                
                if self.interrupt_measurement_called:
                    break
                
                if frame_idx < step_num-1: # does not make a step after the last acquisition
                    target_pos = starting_pos + (frame_idx+1) * step
                    self.stage.motor.move_absolute(target_pos) 
                    self.stage.motor.wait_on_target()
                    

                self.stage.read_from_hardware()

        elif self.settings['camera_trigger'] == 'external':
                self.image_gen.settings['trigger_source'] = 'external'
                self.image_gen.settings['acquisition_mode'] = 'run_till_abort'
                self.image_gen.settings['number_frames'] = step_num+10; #expected step_num but not always true, 10 arbitrary
                #necessary to avoid warning about buffer
                #step_num+1 because initial position is included
                self.image_gen.read_from_hardware()
                read_start_pos = self.stage.motor.get_position() #read the starting position
                print('Scan starting position:', read_start_pos)
                target_pos = starting_pos + step_num * step # final position
                self.set_motor_velocity() # set an appropriate velocity for the scan
                print('Debugging: Motor velocity:', self.stage.motor.get_velocity(), 'mm/s')

                #Acquisition with trigger
                self.image_gen.hamamatsu.startAcquisition()

                start_time=time.time()
                self.stage.motor.trigger(step, target_pos)
                self.stage.motor.wait_on_target()
                end_time = time.time()
                print('Acquisition time:', end_time-start_time, 's')
                read_final_pos = self.stage.motor.get_position() #read the final position
                print('Final position:', read_final_pos)
                
                self.image_gen.hamamatsu.stopAcquisitionNotReleasing() #stop acquisition without releasing the buffer
                [frames, dims] = self.image_gen.hamamatsu.getFrames() 
                print('Number of acquired frames ',len(frames))
                #self.step_num_eff=len(frames) #effective number of acquired frames is different from step_num (one more)
                #Create the h5 file and save the data
                if self.settings['save_h5']:
                    self.create_h5_file()
                    for frame_idx in range(0, len(frames)): #len(frames)=step_num+1: using this range I loos the last frame 
                        self.np_data=frames[frame_idx].getData()
                        self.image=np.reshape(self.np_data, (dims[0], dims[1]))
                        self.image_h5[frame_idx,:,:] = self.image
                        self.frame_index = frame_idx
                    vettore_posizioni = np.linspace(read_start_pos, read_final_pos, len(frames))*1000 # convert to um
                    self.positions_h5[0:len(frames)] = vettore_posizioni
                    print('positions_h5: ', self.positions_h5)
                    self.h5file.flush()
                
                self.image_gen.settings['trigger_source'] = 'internal'
                self.stage.settings['velocity'] = 5
                self.image_gen.read_from_hardware()
                self.stage.read_from_hardware()
        else:
            raise ValueError('Measurement trigger is not set on internal nor external')

            
    def run(self):
                   
        try:
            #start the camera
            self.time = []
            self.intensity = []
        
            self.frame_index = -1
            self.eff_subarrayh = int(self.image_gen.subarrayh.val/self.image_gen.binning.val)
            self.eff_subarrayv = int(self.image_gen.subarrayv.val/self.image_gen.binning.val)
            
            self.image_gen.read_from_hardware()
    
            self.image_gen.settings['acquisition_mode'] = 'run_till_abort'
            self.image_gen.hamamatsu.startAcquisition()
            
            # continuously get the last frame and put it in self.image, in order to 
            # show it via self.update_display()
            
            while not self.interrupt_measurement_called:
                
                [frame, dims] = self.image_gen.hamamatsu.getLastFrame()        
                self.np_data = frame.getData()
                self.img = np.reshape(self.np_data,(self.eff_subarrayv, self.eff_subarrayh))
                
                # If measurement is called, stop the acquisition, call self.measure
                # and get out of run()
                if self.settings['save_h5']:
                    self.image_gen.hamamatsu.stopAcquisition()
                    self.measure()
                    break
                
                if self.interrupt_measurement_called:
                    break
            
        finally:
            self.image_gen.hamamatsu.stopAcquisition()
            if self.settings['save_h5'] and hasattr(self, 'h5file'):
                # make sure to close the data file
                self.h5file.close() 
                self.settings['save_h5'] = False
                


    # def compute_motor_velocity(self):
    #     #compute an appropriate motor velocity for continuous scan according to step size and exposure time
    #     # User can set the motor velocity manually
    #     # self.stage.settings['velocity'] = self.settings['motor_velocity']
    #     # self.stage.read_from_hardware()

    #     if self.image_gen.settings['exposure_time']<0.010: #limitation due to camera maximum frame rate
    #         frame_time=0.010      
    #     else:
    #         frame_time=self.image_gen.settings['exposure_time']

    #     self.stage.settings['velocity']=self.settings['step']*10**(-3)/(4*frame_time) # in mm/s
    #     self.stage.read_from_hardware()

    def set_motor_velocity(self):
        self.stage.settings['velocity'] = self.settings['motor_velocity']
        self.stage.read_from_hardware()
        '''
        
        '''

                
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
        
        if self.settings['camera_trigger'] == 'internal':
            length = self.settings.step_num.val
        elif self.settings['camera_trigger'] == 'external': #with trigger there is a frame more acquired
            length = self.settings.step_num.val+1
        
        self.image_h5 = self.h5_group.create_dataset(name  = 't0/c0/image', 
                                                  shape = [length, img_size[0], img_size[1]],
                                                  dtype = dtype)
        self.image_h5.attrs['element_size_um'] =  [self.settings['zsampling'],self.settings['ysampling'],self.settings['xsampling']]
        
        self.positions_h5 = self.h5_group.create_dataset(name  = 't0/c0/position_mm', 
                                                  shape = [length],
                                                  dtype = np.float32)
                   

    
