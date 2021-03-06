import os
import sys
import rospy
import rospkg
import yaml
from rqt_gui.main import Main
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtGui import QWidget, QMainWindow, QTextCursor, QColor, QFileDialog
import itertools
from time import sleep
import shutil
from glob import glob
from gear_data_handler.postprocessor import PostProcessor
import funcy

DEFAULT_DATA_DIR = "/mnt/md0/gear_data"
DEFAULT_COMPOSITION = '[["k2", "color"], ["k4", "color"], ["k5", "color"]]'
DEFAULT_VIDEO_BLACKLIST = ["depth"]
DEFAULT_VIDEO_EXTN = ".avi"
DEFAULT_FRAME_RATE = 15
DEFAULT_WORKER_THREADS = 8
DEFAULT_CROP_PARAMS = '[]'#'[["k4", "color", 0, 0, 500, 500]]'
DEFAULT_COMPOSITION_SCALING_FACTOR = "[0.1, 0.25, 0.25]"
         
class TaskThread(QtCore.QThread):
    task_finished = QtCore.pyqtSignal(int, int)
    
    def __init__(self, tid):
        super(TaskThread, self).__init__()
        self.tid = tid
     
    def initialize(self, root_dir, task, task_type, video_extn, frame_rate, composition_scale_factor):
        '''
        Constructor
        '''
        self._root_dir = root_dir
        self._task = task
        self._task_type = task_type
        self._video_extn = video_extn
        self._frame_rate = frame_rate
        self._composition_scale_factor = composition_scale_factor
        return

    @QtCore.pyqtSlot()     
    def run(self):
        '''
        Interface with the backend postprocessing
        '''
        # Create postprocessing object
        post_processor_obj = PostProcessor(self._root_dir, self._video_extn, self._frame_rate, 
                                           self._composition_scale_factor)
         
        # Generate video or composition
        if self._task_type == "video":
            post_processor_obj.create_video(self._task)
            task_size = 1
        elif self._task_type == "composition":
            post_processor_obj.create_composition(self._task)
            task_size = len(self._task)

        self.task_finished.emit(self.tid, task_size)
        return
                     
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
        self._configure_node()
        
        # Configure gui
        self._configure_gui()
                
        return      

    def _configure_node(self):
        '''
        Configure ros specific stuff
        '''
        self._data_dir = rospy.get_param("~data_dir", DEFAULT_DATA_DIR)
        self._video_extn = rospy.get_param("~video_extn", DEFAULT_VIDEO_EXTN)
        self._frame_rate = rospy.get_param("~frame_rate", DEFAULT_FRAME_RATE)
        self._worker_threads = rospy.get_param("~num_worker_threads", DEFAULT_WORKER_THREADS)
        self._video_blacklist = rospy.get_param("~video_blacklist", DEFAULT_VIDEO_BLACKLIST)
        self._composition = eval(rospy.get_param("~composition", DEFAULT_COMPOSITION))
        self._crop_params = eval(rospy.get_param("~crop_params", DEFAULT_CROP_PARAMS))
        self._composition_scale_factor = eval(rospy.get_param("~composition_scale_factor", DEFAULT_COMPOSITION_SCALING_FACTOR))

        rospy.loginfo("[GearPostprocess] Data directory: "+self._data_dir)
        rospy.loginfo("[GearPostprocess] Video extension: "+self._video_extn)
        rospy.loginfo("[GearPostprocess] Frame rate: "+str(self._frame_rate))
        rospy.loginfo("[GearPostprocess] Worker threads: "+str(self._worker_threads))
        rospy.loginfo("[GearPostprocess] Video blacklist: "+str(self._video_blacklist))
        rospy.loginfo("[GearPostprocess] Composition: "+str(self._composition))
        rospy.loginfo("[GearPostprocess] Crop params: "+str(self._crop_params))
        rospy.loginfo("[GearPostprocess] Composition scaling factor: "+str(self._composition_scale_factor))

        return

    def _configure_gui(self):
        '''
        Configure the gui elements on startup
        '''        
        # Just call reset callback
        self._onclicked_btnReset()
        self._onchanged_txtFilepath()

        # Register callbacks for the gui elements
        self._widget.btnFileselect.clicked[bool].connect(self._onclicked_btnFileselect);
        self._widget.txtFilepath.returnPressed.connect(self._onchanged_txtFilepath);

        self._widget.btnStart.clicked[bool].connect(self._onclicked_btnStart);
        self._widget.btnReset.clicked[bool].connect(self._onclicked_btnReset);        
        self._widget.btnStop.clicked[bool].connect(self._onclicked_btnStop);        

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
        
        # Create worker thread
        self.task_threads = [TaskThread(i) for i in xrange(self._worker_threads)]
        self.thread_idle = [True]*self._worker_threads
        for t in self.task_threads: 
            t.task_finished.connect(self._onfinished_task)
        
        return

    @QtCore.pyqtSlot(int, int)    
    def _onfinished_task(self, tid, task_size):
        '''
        Update when an ongoing task is finished
        '''
        self.progress_value += task_size
        self._widget.progressBar.setValue(self.progress_value)
        task_started = False
        composition_started = False
        self.thread_idle[tid] = True
        
        # Process videos
        if self.tasks:
            task_started = True
            self._output_statustext("Processing task "+PostProcessor.build_video_name(self.tasks[-1])+" ...")
            self.task_threads[tid].initialize(self.root_dir, self.tasks.pop(), "video", self._video_extn, 
                                              self._frame_rate, self._composition_scale_factor)
            self.task_threads[tid].start()
            self.thread_idle[tid] = False
        
        # Process compositions
        if not task_started and self.composition_tasks:
            composition_started = True
            self._output_statustext("Processing composition "+PostProcessor.build_composition_name(self.composition_tasks[-1])+" ...")
            self.task_threads[tid].initialize(self.root_dir, self.composition_tasks.pop(), "composition", self._video_extn, 
                                              self._frame_rate, self._composition_scale_factor)
            self.task_threads[tid].start()  
            self.thread_idle[tid] = False      
        
        if not task_started and not composition_started and all(self.thread_idle):
            self._output_statustext("Tasks complete")
            self._onclicked_btnReset()
            self._widget.progressBar.setValue(self.progress_value)
        return

    @QtCore.pyqtSlot()    
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

    @QtCore.pyqtSlot()
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
    
    @QtCore.pyqtSlot()
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
    
    @QtCore.pyqtSlot()
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

    @QtCore.pyqtSlot()
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
    
    def _update_combobox(self, q_obj, item_list):
        '''
        Update combobox without emitting signals
        '''
        q_obj.blockSignals(True)
        q_obj.clear()
        q_obj.addItems(sorted(list(item_list)))
        q_obj.blockSignals(False)
        return

    def _update_textbox(self, q_obj, text):
        '''
        Update combobox without emitting signals
        '''
        q_obj.blockSignals(True)
        q_obj.setText(text)
        q_obj.blockSignals(False)
        return

    @QtCore.pyqtSlot()                    
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
        
        self._update_combobox(self._widget.comboSubject, subject_list)
        self._onchanged_comboSubject()
        return

    @QtCore.pyqtSlot()        
    def _onclicked_btnFileselect(self):
        '''
        Select root directory
        '''
        
        if os.path.isdir(str(self._widget.txtFilepath.text())):
            start_dir = str(self._widget.txtFilepath.text())
        else:
            start_dir = self._data_dir
            
        root_dir = QFileDialog.getExistingDirectory(self._widget, "Open a folder", start_dir, QFileDialog.ShowDirsOnly)
        if root_dir == "":
            root_dir = self._data_dir
        self._update_textbox(self._widget.txtFilepath, root_dir)
        self._onchanged_txtFilepath()
        return

    @QtCore.pyqtSlot()    
    def _onchanged_comboSubject(self):
        '''
        Refresh GUI when subject changes
        '''
        self.subject = str(self._widget.comboSubject.currentText())
        
        session_list = [os.path.basename(d.rstrip('/')) for d in glob(os.path.join(self.root_dir, self.subject)+"/*/")]
        if not session_list:
            return
        
        self._update_combobox(self._widget.comboSession, session_list)
        self._onchanged_comboSession()   
        return     

    @QtCore.pyqtSlot()
    def _onchanged_comboSession(self):
        '''
        Refresh GUI when subject changes
        '''
        self.session = str(self._widget.comboSession.currentText())
        
        activity_list = self._get_activity_condition_trial_list("activity")
        if not activity_list:
            return
        
        self._update_combobox(self._widget.comboActivity, activity_list)
        self._onchanged_comboActivity()  
        return      

    @QtCore.pyqtSlot()
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
        
        self._update_combobox(self._widget.comboCondition, condition_list)
        self._onchanged_comboCondition()        
        return

    @QtCore.pyqtSlot()
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
        
        self._update_combobox(self._widget.comboTrial, trial_list)
        self._onchanged_comboTrial()        
        return
    
    @QtCore.pyqtSlot()
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

        self._update_combobox(self._widget.comboSensor, sensor_list)        
        self._onchanged_comboSensor()        
        return
    
    @QtCore.pyqtSlot()
    def _onchanged_comboSensor(self):
        '''
        Refresh GUI when subject changes
        '''
        if self._widget.chkSensorAll.isChecked():
            self.sensor = self._get_sensor_video_list('sensor')
        else:
            self.sensor = set([str(self._widget.comboSensor.currentText())])
        
        video_list = self._get_sensor_video_list('video')
        video_list.difference_update(set(self._video_blacklist))        
        if not video_list:
            return
        
        self._update_combobox(self._widget.comboVideo, video_list)
        self._onchanged_comboVideo()        
        return
    
    def _onchanged_comboVideo(self):
        '''
        Refresh GUI when subject changes
        '''
        if self._widget.chkVideoAll.isChecked():
            self.video = self._get_sensor_video_list('video')
            self.video.difference_update(set(self._video_blacklist))        
        else:
            self._update_combobox(self._widget.comboVideo, [str(self._widget.comboVideo.currentText())])
        return

    @QtCore.pyqtSlot()            
    def _onclicked_btnReset(self):
        '''
        Reset everything to default
        '''
        self._widget.txtFilepath.setText(self._data_dir)

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
            self._widget.btnStop.setEnabled(False)
            self._widget.btnFileselect.setEnabled(True)
            self._widget.txtFilepath.setEnabled(True)           
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

    @QtCore.pyqtSlot()            
    def _onclicked_btnStart(self):
        '''
        Start generating videos
        '''                        
        # Generate all tasks
        self.tasks, self.composition_tasks = self._generate_tasks()
        totalTasks = len(self.tasks)+len(funcy.cat(self.composition_tasks))

        # Configure gui elements
        self._disable_all()
        self._widget.btnFileselect.setEnabled(False)
        self._widget.txtFilepath.setEnabled(False)
        self._widget.progressBar.setValue(0)
        self._widget.progressBar.setRange(0, totalTasks)
        self._widget.btnStop.setEnabled(True)
        self._widget.btnReset.setEnabled(False)
        self.progress_value = 0
        self._widget.txtOutput.clear()
        self._output_statustext("Total videos: "+str(len(self.tasks)))
        self._output_statustext("Total compositions: "+str(len(self.composition_tasks)))
        self._output_statustext("Starting tasks on {num_workers} workers...".format(num_workers=self._worker_threads))
        
        # Process tasks
        for t in self.task_threads:
            t.task_finished.emit(t.tid, 0)
        
        return

    @QtCore.pyqtSlot()
    def _onclicked_btnStop(self):
        '''
        Stop generating videos
        '''
        self.tasks = []
        self.composition_tasks = []
        self._output_statustext("Stopping ...")
        self._widget.btnStop.setEnabled(False)
        return

    def _output_statustext(self, text):
        '''
        Show output in status box
        '''
        self._widget.txtOutput.append(text)
        self._widget.txtOutput.moveCursor(QTextCursor.End)
        self._widget.txtOutput.ensureCursorVisible()
        return
    
    def _generate_task_params(self, task):
        '''
        Generate the parameters for a task
        '''
        task_param = {}
        crop_param = []
        
        # Create crop param
        for k in self._crop_params:
            if k[0:2] == task[5:7]:
                crop_param = k[2:]
        task_param["crop"] = crop_param        
        
        return task_param
    
    def _fill_task_param(self, task):
        '''
        Add task params to a task
        '''
        task_param = self._generate_task_params(task)
        filled_task = list(task)+[task_param]
        
        return filled_task
    
    def _generate_tasks(self):
        '''
        Interface with the backend postprocessing
        '''
        # Generate all possible tasks
        tasks = itertools.product([self.subject], [self.session], 
                                  self.activity, self.condition, self.trial, 
                                  self.sensor, self.video)
        tasks = [self._fill_task_param(list(t)) for t in tasks]
        
        # Find all valid tasks
        valid_tasks = [t for t in tasks if self._validate_task(t)]
        
        # Get all possiblities till trials       
        trials = itertools.product([self.subject], [self.session], 
                                   self.activity, self.condition, self.trial)

        # All possible compositions
        compositions = []
        for t in trials:
            temp = itertools.product([t], self._composition)
            compositions.append([self._fill_task_param(list(t1[0])+list(t1[1])) for t1 in temp])
        
        # Get valid compositions
        valid_compositions = [c for c in compositions if self._validate_composition(c)]

        return valid_tasks, valid_compositions
        
    def _validate_task(self, task):
        '''
        Validate if a given task is possible
        '''
        path = os.path.join(self.root_dir, task[0], task[1], '_'.join(task[2:5]), "images", '_'.join(task[5:7]))
        return os.path.exists(path)
    
    def _validate_composition(self, composition):
        '''
        Check if a composition is possible
        '''
        for t in composition:
            if not self._validate_task(t):
                return False
        return True
                
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
    main = Main()
    sys.exit(main.main(sys.argv, standalone='gear_postprocess.postprocess.PostprocessGUI'))