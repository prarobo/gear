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
from std_srvs.srv._Trigger import Trigger
from std_srvs.srv._SetBool import SetBool, SetBoolRequest
from gear_data_handler.srv._TimeExtent import TimeExtent
from rosgraph_msgs.msg._Clock import Clock
from decimal import Decimal

DEFAULT_DATA_DIR = "/mnt/md0/gear_data"
DEFAULT_PLAYBACK_START_SERVICE_NAME = "/start_playback"
DEFAULT_PLAYBACK_STOP_SERVICE_NAME = "/stop_playback"
DEFAULT_PLAYBACK_TIME_SERVICE_NAME  = "/playback_time_extents"
DEFAULT_TF_SERVICE_NAME = "/trigger_tf"
DEFAULT_TRACKING_SESSION_SERVICE_NAME = "/track_tracking_session_info"
DEFAULT_TRACKING_TRIGGER_SERVICE_NAME = "/track_tracking_enable"

class ProgressBarHelper(QtCore.QObject):
    update_progress = QtCore.pyqtSignal(int)
    
    def __init__(self):
        super(ProgressBarHelper, self).__init__()
                                          
class PlaybackGUI(Plugin):

    def __init__(self, context):
        super(PlaybackGUI, self).__init__(context)
        self.setObjectName('GearPlayback')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('gear_playback'), 'resource', 'gear_playback.ui')
        
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        
        # Give QObjects reasonable names
        self._widget.setObjectName('GearPlaybackUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        
        # Configure node
        self._configure_node()
        
        # Configure gui
        self._configure_gui()
        
        # Setup timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._check_completion)
        self.timer.start(2000)
                
        return      
     
    def _configure_node(self):
        '''
        Configure node specific paarameters
        '''
        rospy.Subscriber("/clock", Clock, self._update_time)

        # Load prameters
        self._data_dir = rospy.get_param("~data_dir", DEFAULT_DATA_DIR)
        self._playback_start_service_name = rospy.get_param("~playback_start_service_name", DEFAULT_PLAYBACK_START_SERVICE_NAME)
        self._playback_stop_service_name = rospy.get_param("~playback_stop_service_name", DEFAULT_PLAYBACK_STOP_SERVICE_NAME)
        self._playback_time_service_name = rospy.get_param("~playback_time_service_name", DEFAULT_PLAYBACK_TIME_SERVICE_NAME)
        self._tf_service_name = rospy.get_param("~tf_service_name", DEFAULT_TF_SERVICE_NAME)
        self._tracking_session_service_name = rospy.get_param("~tracking_session_service_name", DEFAULT_TRACKING_SESSION_SERVICE_NAME)
        self._tracking_trigger_service_name = rospy.get_param("~tracking_trigger_service_name", DEFAULT_TRACKING_TRIGGER_SERVICE_NAME)        

        return
           
    def _configure_gui(self):
        '''
        Configure the gui elements on startup
        '''        
        # Just call reset callback
        self._widget.txtFilepath.setText(self._data_dir)
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
        
        self.progressBarHelper = ProgressBarHelper()
        self.progressBarHelper.update_progress.connect(self._widget.progressBar.setValue)

        return
    
    def _check_completion(self):
        '''
        Check if there are update from clock server to determine completion of playback
        '''
        if self._started:
            if self._prev_time != rospy.Time(0) and self._curr_time == self._prev_time:
                self._widget.btnStop.clicked.emit(True)                 
            self._prev_time = self._curr_time
        return
    
    def _update_time(self, clock_time):
        '''
        Update time from clock
        '''
        self._curr_time = clock_time.clock
        val  = self._compute_time_sec(clock_time.clock) - self._compute_time_sec(self._start_time)
        self.progressBarHelper.update_progress.emit(val)
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
        self.trial = set([str(self._widget.comboTrial.currentText())])
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
            self._widget.comboSubject.setEnabled(True)
            self._widget.comboSession.setEnabled(True)
            self._widget.comboActivity.setEnabled(True)
            self._widget.comboCondition.setEnabled(True)
            self._widget.comboTrial.setEnabled(True)
            self._widget.btnReset.setEnabled(True)
            self._widget.btnStart.setEnabled(True)
            self._widget.progressBar.setValue(0)
            self._widget.btnStop.setEnabled(False)
            self._widget.btnFileselect.setEnabled(True)
            self._widget.txtFilepath.setEnabled(True)   
            self._widget.chkTracking.setEnabled(True)
            self._widget.chkTracking.setChecked(False)
            
        self._curr_time = rospy.Time(0)
        self._prev_time = rospy.Time(0)
        self._start_time = rospy.Time(0)
        self._started = False          
        return

    def _disable_all(self):
        '''
        Disable all controls
        '''
        self._widget.comboActivity.setEnabled(False)
        self._widget.comboCondition.setEnabled(False)
        self._widget.comboTrial.setEnabled(False)
        self._widget.btnReset.setEnabled(True)
        self._widget.comboSubject.setEnabled(False)
        self._widget.comboSession.setEnabled(False)
        self._widget.btnStart.setEnabled(False)
        self._widget.btnStop.setEnabled(False)
        self._widget.chkTracking.setEnabled(False)
        return

    @QtCore.pyqtSlot()            
    def _onclicked_btnStart(self):
        '''
        Start generating videos
        '''                        
        # Configure gui elements
        self._disable_all()
        self._widget.btnFileselect.setEnabled(False)
        self._widget.txtFilepath.setEnabled(False)
        self._widget.btnStart.setEnabled(False)
        self._widget.btnReset.setEnabled(False)
        self.progress_value = 0
        self._widget.txtOutput.clear()
        self._output_statustext("Starting playback ...")
        
        # Setting ros parameters
        if not self._started:
            self._set_ros_parameters()
            
        # Start tf
        tf_service = rospy.ServiceProxy(self._tf_service_name, Trigger)
        resp = tf_service()        
        if not resp.success:
            rospy.logerr("[GearPlayback] Failed to trigger TF")
        
        if self._widget.chkTracking.isChecked():    
            # Setting tracking logger session information
            tracking_session_service = rospy.ServiceProxy(self._tracking_session_service_name, Trigger)
            resp = tracking_session_service()        
            if not resp.success:
                rospy.logerr("[GearPlayback] Failed to set tracking logger session information")
            
            # Starting tracking logger
            tracking_trigger_service = rospy.ServiceProxy(self._tracking_trigger_service_name, SetBool)
            req = SetBoolRequest()
            req.data = True
            resp = tracking_trigger_service(req)        
            if not resp.success:
                rospy.logerr("[GearPlayback] Failed to start tracking logger")

        # Start playback node
        playback_start_service = rospy.ServiceProxy(self._playback_start_service_name, Trigger)
        resp = playback_start_service()
        if resp.success:
            rospy.loginfo("[GearPlayback] Playback started: "+str(resp.success))
            playback_time_service = rospy.ServiceProxy(self._playback_time_service_name, TimeExtent)
            resp = playback_time_service()
            time_diff = self._compute_time_sec(resp.stop_time)-self._compute_time_sec(resp.start_time)
            self._start_time = resp.start_time
            self._widget.progressBar.setRange(0, time_diff)
            self._widget.progressBar.setValue(0)
            self._started = True
            self._output_statustext("Started playback")
            self._widget.btnStop.setEnabled(True)
        else:                                
            rospy.logerr("[GearPlayback] Playback starting failed")
            self._widget.btnReset.clicked.emit(True)
            self._output_statustext("Failed to start playback")
        return
    
    def _set_ros_parameters(self):
        '''
        Set parameters to ros param server
        '''
        subject_id = str(self._widget.comboSubject.currentText())
        session_id = str(self._widget.comboSession.currentText())
        activity_id = str(self._widget.comboActivity.currentText())
        condition_id = str(self._widget.comboCondition.currentText())
        trial_id = str(self._widget.comboTrial.currentText())
        
        rospy.set_param("subject_id", subject_id)
        rospy.set_param("session_id", session_id)
        rospy.set_param("activity_id", activity_id)
        rospy.set_param("condition_id", condition_id)
        rospy.set_param("trial_id", int(trial_id))

        rospy.loginfo("[GearPlayback] Setting parameter subject_id: "+subject_id)    
        rospy.loginfo("[GearPlayback] Setting parameter session_id: "+session_id)
        rospy.loginfo("[GearPlayback] Setting parameter activity_id: "+activity_id)
        rospy.loginfo("[GearPlayback] Setting parameter condition_id: "+condition_id)
        rospy.loginfo("[GearPlayback] Setting parameter trial_id: "+trial_id)
        return
        
    def _compute_time_sec(self, time_obj):
        '''
        Compute time in minutes
        '''
        time_sec = round(time_obj.to_sec())
        return time_sec

    @QtCore.pyqtSlot()
    def _onclicked_btnStop(self):
        '''
        Stop generating videos
        '''
        self._output_statustext("Stopping playback ...")
        self._widget.btnStop.setEnabled(False)
        self._started = False
        
        if self._widget.chkTracking.isChecked():
            # Stopping tracking logger
            tracking_trigger_service = rospy.ServiceProxy(self._tracking_trigger_service_name, SetBool)
            req = SetBoolRequest()
            req.data = False
            resp = tracking_trigger_service(req)        
            if not resp.success:
                rospy.logerr("[GearPlayback] Failed to stop tracking logger")

        # Stop playback node
        playback_stop_service = rospy.ServiceProxy(self._playback_stop_service_name, Trigger)
        resp = playback_stop_service()
        if resp.success:
            rospy.loginfo("[GearPlayback] Playback stopped: "+str(resp.success))
            self._widget.btnReset.setEnabled(True)
            self._widget.btnStart.setEnabled(True)
            self._widget.chkTracking.setChecked(False)
            self._output_statustext("Stopped playback")
        else:                                
            rospy.logerr("[GearPlayback] Playback stopping failed")
            self._widget.btnReset.clicked.emit(True)
            self._output_statustext("Failed to stop playback")
        return

    def _output_statustext(self, text):
        '''
        Show output in status box
        '''
        self._widget.txtOutput.append(text)
        self._widget.txtOutput.moveCursor(QTextCursor.End)
        self._widget.txtOutput.ensureCursorVisible()
        return
    
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
    sys.exit(main.main(sys.argv, standalone='gear_playback.playback.PlaybackGUI'))
    