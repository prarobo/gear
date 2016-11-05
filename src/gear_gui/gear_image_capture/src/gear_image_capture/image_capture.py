import os
import rospy
import rospkg
import yaml
import pprint

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtGui import QWidget, QMainWindow
from std_srvs.srv._SetBool import SetBool
from std_msgs.msg import Bool

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
        self._session_timer_start_pub = rospy.Publisher("/gui/start_session_timer", Bool, queue_size=1)
        self._session_timer_stop_pub = rospy.Publisher("/gui/stop_session_timer", Bool, queue_size=1)
            
    def _configure_gui(self):
        '''
        Configure the gui element on startup
        '''
        # Load data collection configuration values
        config_file = os.path.join(rospkg.RosPack().get_path('gear_image_capture'), 'config', 'settings.yaml')
        rospy.loginfo("[GearImageCapture] Loading parameters from file: "+config_file)
        config_data = yaml.load(open(config_file,'r'))
        
        trial_list = [str(i) for i in xrange(1,config_data["MaxTrials"])]
        activity_list = config_data["Activity"]
        self._sensors = config_data["Sensors"]

        # Set default values
        self._widget.txtSession.setText("test")
        self._widget.comboActivity.addItems(activity_list)
        self._widget.comboTrial.addItems(trial_list)
            
    def _onclicked_initialize(self):
        '''
        Initializing the image capture utility on user request
        '''
        rospy.loginfo("[GearImageCapture] Setting data collection parameters")
        rospy.set_param("session_id", str(self._widget.txtSession.text()))
        rospy.set_param("activity_id", str(self._widget.comboActivity.currentText()))
        rospy.set_param("trial_id", int(self._widget.comboTrial.currentText()))

    def _onclicked_start(self):
        '''
        Initializing the image capture utility on user request
        '''
        rospy.loginfo("[GearImageCapture] Starting image capture ...")
        for s in self._sensors:
            service_name = '/'+s+'_enable'
            try:
                rospy.wait_for_service(service_name, 1)
            except rospy.ROSException:
                rospy.logwarn("[GearImageCapture] Sensor "+s+" service not available")
                continue
            
            # Service call to trigger logging
            sensor_trigger = rospy.ServiceProxy(service_name, SetBool)
            resp = sensor_trigger(True)
            rospy.loginfo("[GearImageCapture] Sensor "+s+" logging started: "+str(resp))
            
        # Start session clock
        self._session_timer_start_pub.publish(True)
            
    def _onclicked_stop(self):
        '''
        Initializing the image capture utility on user request
        '''
        rospy.loginfo("[GearImageCapture] Stopping image capture ...")
        for s in self._sensors:
            service_name = '/'+s+'_enable'
            try:
                rospy.wait_for_service(service_name, 1)
            except rospy.ROSException:
                rospy.logwarn("[GearImageCapture] Sensor "+s+" service not available")
                continue

            sensor_trigger = rospy.ServiceProxy(service_name, SetBool)
            resp = sensor_trigger(False)
            rospy.loginfo("[GearImageCapture] Sensor "+s+" logging stopped: "+str(resp))
            
        # Stop session clock
        self._session_timer_stop_pub.publish(True)

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