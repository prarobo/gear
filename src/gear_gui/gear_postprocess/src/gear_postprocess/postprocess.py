import os
import rospy
import rospkg
import yaml
import pprint
import commands

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtGui import QWidget, QMainWindow, QTextCursor, QColor, QFileDialog
import itertools
from time import sleep
import shutil
from glob import glob

DEFAULT_DATA_DIR = "/mnt/md0/gear_data"
DEFAULT_COMPOSITION = [("k2", "color"), ("p1", "color"), ("p2", "color")]

class PostprocessGUI(Plugin):

    def __init__(self, context):
        super(PostprocessGUI, self).__init__(context)
        self.setObjectName('GearPostprocess')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('gear_postprocess'), 'resource', 'gear_postprocess.ui')
        
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        
        # Give QObjects reasonable names
        self._widget.setObjectName('GearPostprocessUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        
        # Configure gui
        self._configure_gui()
                
        return      
            
    def _configure_gui(self):
        '''
        Configure the gui elements on startup
        '''        
        # Just call reset callback
        self._onclicked_btnReset()

        # Register callbacks for the gui elements
        self._widget.btnFileselect.clicked[bool].connect(self._onclicked_btnFileselect);
        self._widget.txtFilepath.returnPressed.connect(self._onchanged_txtFilepath);

        self._widget.btnStart.clicked[bool].connect(self._onclicked_btnStart);
        self._widget.btnReset.clicked[bool].connect(self._onclicked_btnReset);        

        self._widget.comboSubject.currentIndexChanged.connect(self._onchanged_comboSubject);  
        self._widget.comboSession.currentIndexChanged.connect(self._onchanged_comboSession);  
        self._widget.comboActivity.currentIndexChanged.connect(self._onchanged_comboActivity);  
        self._widget.comboCondition.currentIndexChanged.connect(self._onchanged_comboCondition);  
        self._widget.comboTrial.currentIndexChanged.connect(self._onchanged_comboTrial);  
        self._widget.comboSensor.currentIndexChanged.connect(self._onchanged_comboSensor);  
        self._widget.comboVideo.currentIndexChanged.connect(self._onchanged_comboVideo);  

        self._widget.chkActivityAll.stateChanged.connect(self._onchanged_chkActivityAll);  
        self._widget.chkConditionAll.stateChanged.connect(self._onchanged_chkConditionAll);  
        self._widget.chkTrialAll.stateChanged.connect(self._onchanged_chkTrialAll);  
        self._widget.chkSensorAll.stateChanged.connect(self._onchanged_chkSensorAll);  
        self._widget.chkVideoAll.stateChanged.connect(self._onchanged_chkVideoAll);  
        return

    def _onchanged_chkActivityAll(self):
        '''
        Check box callback
        '''
        self._onchanged_comboActivity()
        if self._widget.chkActivityAll.isChecked():
            self._widget.comboActivity.setEnabled(False)
        else:
            self._widget.comboActivity.setEnabled(True)
        return

    def _onchanged_chkConditionAll(self):
        '''
        Check box callback
        '''
        self._onchanged_comboCondition()
        if self._widget.chkConditionAll.isChecked():
            self._widget.comboCondition.setEnabled(False)
        else:
            self._widget.comboCondition.setEnabled(True)
        return

    def _onchanged_chkTrialAll(self):
        '''
        Check box callback
        '''
        self._onchanged_comboTrial()
        if self._widget.chkTrialAll.isChecked():
            self._widget.comboTrial.setEnabled(False)
        else:
            self._widget.comboTrial.setEnabled(True)
        return

    def _onchanged_chkSensorAll(self):
        '''
        Check box callback
        '''
        self._onchanged_comboSensor()
        if self._widget.chkSensorAll.isChecked():
            self._widget.comboSensor.setEnabled(False)
        else:
            self._widget.comboSensor.setEnabled(True)
        return

    def _onchanged_chkVideoAll(self):
        '''
        Check box callback
        '''
        self._onchanged_comboVideo()
        if self._widget.chkVideoAll.isChecked():
            self._widget.comboVideo.setEnabled(False)
        else:
            self._widget.comboVideo.setEnabled(True)
        return
                
    def _onchanged_txtFilepath(self):
        '''
        Refresh on changing root dir
        '''        
        # Check if the input directory is valid
        if not os.path.isdir(str(self._widget.txtFilepath.text())):
            self._disable_all()
            return
            
        self.root_dir = str(self._widget.txtFilepath.text())

        subject_list = [os.path.basename(d.rstrip('/')) for d in glob(self.root_dir+"/*/")]                  
        if not subject_list:
            return

        self._widget.comboSubject.clear()        
        self._widget.comboSubject.addItems(subject_list)
        self._onchanged_comboSubject()
        return
        
    def _onclicked_btnFileselect(self):
        '''
        Select root directory
        '''
        
        if os.path.isdir(str(self._widget.txtFilepath.text())):
            start_dir = str(self._widget.txtFilepath.text())
        else:
            start_dir = DEFAULT_DATA_DIR
            
        root_dir = QFileDialog.getExistingDirectory(self._widget, "Open a folder", start_dir, QFileDialog.ShowDirsOnly)
        self._widget.txtFilepath.setText(root_dir)
        self._onchanged_txtFilepath()
        return
    
    def _onchanged_comboSubject(self):
        '''
        Refresh GUI when subject changes
        '''
        self.subject = str(self._widget.comboSubject.currentText())
        
        session_list = [os.path.basename(d.rstrip('/')) for d in glob(os.path.join(self.root_dir, self.subject)+"/*/")]
        if not session_list:
            return
        
        self._widget.comboSession.clear()
        self._widget.comboSession.addItems(session_list)
        self._onchanged_comboSession()   
        return     

    def _onchanged_comboSession(self):
        '''
        Refresh GUI when subject changes
        '''
        self.session = str(self._widget.comboSession.currentText())
        
        activity_list = self._get_activity_condition_trial_list("activity")
        if not activity_list:
            return
        
        self._widget.comboActivity.clear()
        self._widget.comboActivity.addItems(list(activity_list))
        self._onchanged_comboActivity()  
        return      

    def _onchanged_comboActivity(self):
        '''
        Refresh GUI when subject changes
        '''
        if self._widget.chkActivityAll.isChecked():
            self.activity = self._get_activity_condition_trial_list("activity")
        else:
            self.activity = set([str(self._widget.comboActivity.currentText())])
            
        condition_list = self._get_activity_condition_trial_list("condition")
        if not condition_list:
            return
        
        self._widget.comboCondition.clear()
        self._widget.comboCondition.addItems(list(condition_list))
        self._onchanged_comboCondition()        
        return

    def _onchanged_comboCondition(self):
        '''
        Refresh GUI when subject changes
        '''
        if self._widget.chkConditionAll.isChecked():
            self.condition = self._get_activity_condition_trial_list("condition")
        else:
            self.condition = set([str(self._widget.comboCondition.currentText())])
            
        trial_list = self._get_activity_condition_trial_list("trial")
        if not trial_list:
            return
        
        self._widget.comboTrial.clear()
        self._widget.comboTrial.addItems(list(trial_list))
        self._onchanged_comboTrial()        
        return
    
    def _onchanged_comboTrial(self):
        '''
        Refresh GUI when subject changes
        '''
        if self._widget.chkTrialAll.isChecked():
            self.trial = self._get_activity_condition_trial_list("trial")
        else:
            self.trial = set([str(self._widget.comboTrial.currentText())])
        
        sensor_list = self._get_sensor_video_list('sensor')                
        if not sensor_list:
            return
        
        self._widget.comboSensor.clear()
        self._widget.comboSensor.addItems(list(sensor_list))
        self._onchanged_comboSensor()        
        return
    
    def _onchanged_comboSensor(self):
        '''
        Refresh GUI when subject changes
        '''
        if self._widget.chkSensorAll.isChecked():
            self.sensor = self._get_sensor_video_list('sensor')
        else:
            self.sensor = set([str(self._widget.comboSensor.currentText())])
        
        video_list = self._get_sensor_video_list('video')        
        if not video_list:
            return
        
        self._widget.comboVideo.clear()
        self._widget.comboVideo.addItems(list(video_list))
        self._onchanged_comboVideo()        
        return
    
    def _onchanged_comboVideo(self):
        '''
        Refresh GUI when subject changes
        '''
        if self._widget.chkVideoAll.isChecked():
            self.video = self._get_sensor_video_list('video')
        else:
            self.video = set([str(self._widget.comboVideo.currentText())])
        return
            
    def _onclicked_btnReset(self):
        '''
        Reset everything to default
        '''
        self._widget.txtFilepath.setText(DEFAULT_DATA_DIR)

        if not os.path.isdir(self._widget.txtFilepath.text()):
            self._disable_all()
        else:
            self._widget.comboActivity.setEnabled(False)
            self._widget.comboCondition.setEnabled(False)
            self._widget.comboTrial.setEnabled(False)
            self._widget.comboSensor.setEnabled(False)
            self._widget.comboVideo.setEnabled(False)            
            self._widget.btnReset.setEnabled(True)
            self._widget.comboSubject.setEnabled(True)
            self._widget.comboSession.setEnabled(True)
            self._widget.chkComposition.setEnabled(True)
            self._widget.chkActivityAll.setEnabled(True)
            self._widget.chkConditionAll.setEnabled(True)
            self._widget.chkTrialAll.setEnabled(True)
            self._widget.chkSensorAll.setEnabled(True)
            self._widget.chkVideoAll.setEnabled(True)
            self._widget.chkComposition.setEnabled(True)
            self._widget.btnStart.setEnabled(True)
            self._widget.chkComposition.setChecked(True)
            self._widget.chkActivityAll.setChecked(True)
            self._widget.chkConditionAll.setChecked(True)
            self._widget.chkTrialAll.setChecked(True)
            self._widget.chkSensorAll.setChecked(True)
            self._widget.chkVideoAll.setChecked(True)
            self._widget.progressBar.setValue(0)
            
            self._onchanged_txtFilepath()
        return

    def _disable_all(self):
        '''
        Disable all controls
        '''
        self._widget.comboActivity.setEnabled(False)
        self._widget.comboCondition.setEnabled(False)
        self._widget.comboTrial.setEnabled(False)
        self._widget.comboSensor.setEnabled(False)
        self._widget.comboVideo.setEnabled(False)            
        self._widget.btnReset.setEnabled(True)
        self._widget.comboSubject.setEnabled(False)
        self._widget.comboSession.setEnabled(False)
        self._widget.chkComposition.setEnabled(False)
        self._widget.chkActivityAll.setEnabled(False)
        self._widget.chkConditionAll.setEnabled(False)
        self._widget.chkTrialAll.setEnabled(False)
        self._widget.chkSensorAll.setEnabled(False)
        self._widget.chkVideoAll.setEnabled(False)
        self._widget.chkComposition.setEnabled(False)
        self._widget.btnStart.setEnabled(False)
        self._widget.btnStop.setEnabled(False)
        return
                
    def _onclicked_btnStart(self):
        '''
        Start generating videos
        '''
        self._widget.progressBar.setValue(0)
        self._widget.btnStop.setEnabled(True)
        
        # Generate all possible tasks
        tasks = itertools.product([self.subject], [self.session], 
                                  self.activity, self.condition, self.trial, 
                                  self.sensor, self.video)
        
        # Find all valid tasks
        valid_tasks = [t for t in tasks if self._validate_task(t)]
        
        # Get all possiblities till trials       
        trials = itertools.product([self.subject], [self.session], 
                                   self.activity, self.condition, self.trial)

        # All possible compositions
        compositions = []
        for t in trials:
            temp = itertools.product([t], DEFAULT_COMPOSITION)

            compositions.append([t1[0]+t1[1] for t1 in temp])
        
        # Get valid compositions
        valid_compositions = [c for c in compositions if self._validate_composition(c)]
        
        return
    
    def _validate_task(self, task):
        '''
        Validate if a given task is possible
        '''
        path = os.path.join(self.root_dir, task[0], task[1], '_'.join(task[2:5]), "images", '_'.join(task[5:]))
        return os.path.exists(path)
    
    def _validate_composition(self, composition):
        '''
        Check if a composition is possible
        '''
        for t in composition:
            if not self._validate_task(t):
                return False
        return True
    
    def _logger(self, output_text, type="info", skip_ui=False):
        '''
        Logging module that handles both roslog and output status
        '''
        if type=="warn":
            rospy.logwarn(output_text)
            if not skip_ui:
                self._widget.txtOutput.setTextColor(QColor(255,0,0))
        else:
            rospy.loginfo(output_text)
            if not skip_ui:
                self._widget.txtOutput.setTextColor(QColor(0,0,0))
        
        if not skip_ui:     
            self._widget.txtOutput.append(output_text)
            self._widget.txtOutput.moveCursor(QTextCursor.End)
            self._widget.txtOutput.ensureCursorVisible()
        return
    
    def _get_sensor_video_list(self, attr_type):
        '''
        Get list of sensors
        '''
        out_list = set([])
        for i in itertools.product(self.activity, self.condition, self.trial):
            path = os.path.join(self.root_dir, self.subject, self.session, '_'.join(i), "images")
            if os.path.exists(path):
                path1 = [d for d in glob(path+"/*/") if len(os.listdir(d))>0]
                out_list.update(self._get_attribute_list(path1, attr_type))
        return out_list
    
    def _get_activity_condition_trial_list(self, attr_type):
        '''
        Get list of attributes
        '''
        paths = glob(os.path.join(self.root_dir, self.subject, self.session)+"/*/")
        return self._get_attribute_list(paths, attr_type)
        
    def _get_attribute_list(self, dir_list, attr_name):
        '''
        Get list of attributes
        '''
        if attr_name == 'activity' or attr_name == 'sensor':
            index = 0
        elif attr_name == 'condition' or attr_name == 'video':
            index = 1
        elif attr_name == 'trial':
            index = 2
        
        attr_list = set([])
        for d in dir_list:
            attr_list.add(os.path.basename(d.rstrip('/')).split('_')[index])
            
        return attr_list

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

if __name__ == "__main__":
    import sys
    from rqt_gui.main import Main
    
    main = Main()
    sys.exit(main.main(sys.argv, standalone='gear_postprocess.postprocess.PostprocessGUI'))