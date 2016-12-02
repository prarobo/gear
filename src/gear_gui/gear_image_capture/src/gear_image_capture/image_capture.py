import os
import rospy
import rospkg
import yaml
import pprint
import commands

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtGui import QWidget, QMainWindow, QTextCursor, QColor
from std_srvs.srv._SetBool import SetBool
from std_srvs.srv._Trigger import Trigger, TriggerRequest
from std_msgs.msg import Bool
import itertools
from time import sleep
import shutil

DEFAULT_SERVICES_TO_ENABLE = ["/k1_color_enable",
                              "/k2_color_enable",
                              "/k3_color_enable",
                              "/p1_color_enable",
                              "/p2_color_enable",
                              "/k1_depth_enable",
                              "/k2_depth_enable",
                              "/k3_depth_enable",
                              "/k1_points_enable",
                              "/k2_points_enable",
                              "/k3_points_enable",
                              "/track_tracking_enable"]

DEFAULT_SERVICES_SESSION_INFO = ["/k1_color_session_info",
                                 "/k2_color_session_info",
                                 "/k3_color_session_info",
                                 "/p1_color_session_info",
                                 "/p2_color_session_info",
                                 "/k1_depth_session_info",
                                 "/k2_depth_session_info",
                                 "/k3_depth_session_info",
                                 "/k1_points_session_info",
                                 "/k2_points_session_info",
                                 "/k3_points_session_info",
                                 "/track_tracking_session_info"]

DEFAULT_DATA_DIR = "/mnt/md0/gear_data"
DEFAULT_SUBJECT = "x"
DEFAULT_DELETABLE_DIR_SIZE = "2" #Gb

class ImageCaptureGUI(Plugin):

    def __init__(self, context):
        super(ImageCaptureGUI, self).__init__(context)
        self.setObjectName('GearImageCapture')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('gear_image_capture'), 'resource', 'gear_image_capture.ui')
        
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        
        # Give QObjects reasonable names
        self._widget.setObjectName('GearImageCaptureUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        
        # Configure ros node
        self._configure_node()
         
        # Configure gui
        self._configure_gui()
                
        # Register callbacks for the gui elements
        self._widget.btnInitialize.clicked[bool].connect(self._onclicked_initialize);
        self._widget.btnStart.clicked[bool].connect(self._onclicked_start);
        self._widget.btnStop.clicked[bool].connect(self._onclicked_stop);
        
    def _configure_node(self):
        '''
        Configure the rosnode
        '''
        self.session_timer_start_pub = rospy.Publisher("/gui/start_session_timer", Bool, queue_size=1)
        self.session_timer_stop_pub = rospy.Publisher("/gui/stop_session_timer", Bool, queue_size=1)
        
        self.services_to_enable = rospy.get_param("services_to_enable", DEFAULT_SERVICES_TO_ENABLE)
        self.services_session_info = rospy.get_param("services_session_info", DEFAULT_SERVICES_SESSION_INFO)
        self.data_dir = rospy.get_param("/data_dir", DEFAULT_DATA_DIR)
            
    def _configure_gui(self):
        '''
        Configure the gui element on startup
        '''
        # Load data collection configuration values
        config_file = os.path.join(rospkg.RosPack().get_path('gear_image_capture'), 'config', 'settings.yaml')
        self._logger("[GearImageCapture] Loading parameters from file: "+config_file)
        config_data = yaml.load(open(config_file,'r'))
        
        session_list = [str(i) for i in xrange(1,config_data["MaxSessions"]+1)]
        trial_list = [str(i) for i in xrange(1,config_data["MaxTrials"]+1)]
        activity_list = config_data["Activity"]
        condition_list = config_data["Condition"]
        
        # Get list of subjects
        subject_list = os.listdir(self.data_dir)
        if not subject_list:
            subject_list = [DEFAULT_SUBJECT]
        
        # Set default values
        self._widget.comboSubject.addItems(subject_list)
        self._widget.comboSession.addItems(session_list)
        self._widget.comboActivity.addItems(activity_list)
        self._widget.comboCondition.addItems(condition_list)
        self._widget.comboTrial.addItems(trial_list)
        
        self.exists_logging_dir = False
        
        # Configure button states
        self._configure_button_states()
        
    def _configure_button_states(self):
        '''
        Configure the button appearance on startup
        '''
        self._widget.btnInitialize.setEnabled(True)
        self._widget.btnStart.setEnabled(False)
        self._widget.btnStop.setEnabled(False)
            
    def _onclicked_initialize(self):
        '''
        Initializing the image capture utility on user request
        '''
        subject_id = str(self._widget.comboSubject.currentText())
        session_id = "S"+str(self._widget.comboSession.currentText())
        activity_id = str(self._widget.comboActivity.currentText())
        condition_id = str(self._widget.comboCondition.currentText())
        trial_id = str(self._widget.comboTrial.currentText())
        
        # Checking if logging directory exists
        logging_dir = os.path.join(self.data_dir, subject_id, session_id, 
                                   '_'.join([activity_id,condition_id,trial_id]))
        exists_logging_dir = os.path.exists(logging_dir)
        if (not exists_logging_dir) or (self.exists_logging_dir and exists_logging_dir):
                       
            # Delete logging directory to clear old data
            if exists_logging_dir:

                # Check size of logging directory   
                dir_size = int(commands.getoutput('du -sh -BG '+logging_dir).split()[0][:-1])
                if dir_size > DEFAULT_DELETABLE_DIR_SIZE:
                    self._logger("[GearImageCapture] Logging directory too big (cannot delete from UI," 
                                 "please manually delete to proceed)", type="warn")
                    return                
                else:
                    shutil.rmtree(logging_dir)
                    
            self.exists_logging_dir = False
                
            self._logger("[GearImageCapture] Setting data collection parameters")
            rospy.set_param("subject_id", subject_id)
            rospy.set_param("session_id", session_id)
            rospy.set_param("activity_id", activity_id)
            rospy.set_param("condition_id", condition_id)
            rospy.set_param("trial_id", int(trial_id))

            self._logger("[GearImageCapture] Setting parameter subject_id: "+subject_id)    
            self._logger("[GearImageCapture] Setting parameter session_id: "+session_id)
            self._logger("[GearImageCapture] Setting parameter activity_id: "+activity_id)
            self._logger("[GearImageCapture] Setting parameter condition_id: "+condition_id)
            self._logger("[GearImageCapture] Setting parameter trial_id: "+trial_id)
            sleep(2)
            
            # Setting session info in loggers
            for service_name in self.services_session_info:
                try:
                    rospy.wait_for_service(service_name, 1)
                except rospy.ROSException:
                    self._logger("[GearImageCapture] Service "+service_name+" service not available", type="warn")
                    continue
                
                # Service call to trigger logging
                sensor_session_info = rospy.ServiceProxy(service_name, Trigger)
                resp = sensor_session_info(TriggerRequest())
                self._logger("[GearImageCapture] Service "+service_name+" logging started: ("
                             +str(resp.success)+","+resp.message+")", skip_ui=True)
            
            # Set button states
            self._widget.btnStart.setEnabled(True)
        else:
            self.exists_logging_dir = True
            self._logger("[GearImageCapture] Logging directory exists (initialize again if you want to overwrite)", type="warn")

    def _onclicked_start(self):
        '''
        Initializing the image capture utility on user request
        '''
        self._logger("[GearImageCapture] Starting image capture ...")
        for service_name in self.services_to_enable:
            try:
                rospy.wait_for_service(service_name, 1)
            except rospy.ROSException:
                rospy.logwarn("[GearImageCapture] Service "+service_name+" service not available")
                continue
            
            # Service call to trigger logging
            sensor_trigger = rospy.ServiceProxy(service_name, SetBool)
            resp = sensor_trigger(True)
            self._logger("[GearImageCapture] Service "+service_name+" logging started")
            self._logger("[GearImageCapture] Service "+service_name+" logging started: "+str(resp), skip_ui=True)
            
        # Start session clock
        self.session_timer_start_pub.publish(True)

        # Set button states
        self._widget.btnInitialize.setEnabled(False)
        self._widget.btnStart.setEnabled(False)     
        self._widget.btnStart.setDown(True)   
        self._widget.btnStop.setEnabled(True)
            
    def _onclicked_stop(self):
        '''
        Initializing the image capture utility on user request
        '''
        self._logger("[GearImageCapture] Stopping image capture ...")
        for service_name in self.services_to_enable:
            try:
                rospy.wait_for_service(service_name, 1)
            except rospy.ROSException:
                self._logger("[GearImageCapture] Service "+service_name+" service not available", type="warn")
                continue

            sensor_trigger = rospy.ServiceProxy(service_name, SetBool)
            resp = sensor_trigger(False)
            self._logger("[GearImageCapture] Service "+service_name+" logging stopped")
            self._logger("[GearImageCapture] Service "+service_name+" logging stopped: "+str(resp), skip_ui=True)
            
        # Stop session clock
        self.session_timer_stop_pub.publish(True)
        
        # Set button states
        self._widget.btnStart.setDown(False)
        self._configure_button_states()
        
        # Refresh list of subjects
        subject_list = os.listdir(self.data_dir)
        if not subject_list:
            subject_list = [DEFAULT_SUBJECT]
        self._widget.comboSubject.addItems(subject_list)
        
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
    gui = ImageCaptureGUI()