import os
import rospy
import rospkg
import yaml
import pprint

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtGui import QWidget, QMainWindow
from std_msgs.msg import Bool
    
class SessionDurationGUI(Plugin):

    def __init__(self, context):
        super(SessionDurationGUI, self).__init__(context)
        self.setObjectName('GearSessionDuration')

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

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('gear_session_duration'), 'resource', 'gear_session_duration.ui')

        # Create QWidget
        self._widget = QWidget() 
        
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        
        # Give QObjects reasonable names
        self._widget.setObjectName('GearSessionDurationUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        
        # Configure rosnode
        self._configure_node()
         
        # Configure gui
        self._configure_gui()
                
        # Setup timer
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.Time)
        
    def _configure_node(self):
        '''
        Configure the rosnode on startup
        '''
        rospy.Subscriber("/gui/start_session_timer", Bool, self._start_timer)
        rospy.Subscriber("/gui/stop_session_timer", Bool, self._stop_timer)
                    
    def _configure_gui(self):
        '''
        Configure the gui element on startup
        '''        
        # Set default values
        self._reset_timer()
        
    def _reset_timer(self):
        '''
        Reset the session timer
        '''
        rospy.loginfo("[GearSessionDuration] Resetting timer")
        global s,m,h
        s = 0
        m = 0
        h = 0
 
        time = "%02d:%02d:%02d"%(h,m,s)
 
        self._widget.lcdSessionDuration.setDigitCount(len(time))
        self._widget.lcdSessionDuration.display(time)
        
        self._widget.lblRecordingStatus.setStyleSheet("background-color: rgb(255, 0, 0);")

    def _start_timer(self, req):
        '''
        Reset the session timer
        '''
        self._reset_timer()
        rospy.loginfo("[GearSessionDuration] Starting timer")
        self.timer.start(1000)
        
        self._widget.lblRecordingStatus.setStyleSheet("background-color: rgb(0, 255, 0);")
        return

    def _stop_timer(self, req):
        '''
        Reset the session timer
        '''
        rospy.loginfo("[GearSessionDuration] Stopping timer")
        self.timer.stop()

        self._widget.lblRecordingStatus.setStyleSheet("background-color: rgb(255, 0, 0);")
        return
    
    def Time(self):
        global s,m,h
 
        if s < 59:
            s += 1
        else:
            if m < 59:
                s = 0
                m += 1
            elif m == 59 and h < 24:
                h += 1
                m = 0
                s = 0
            else:
                self.timer.stop()
 
        time = "%02d:%02d:%02d"%(h,m,s)
 
        self._widget.lcdSessionDuration.setDigitCount(len(time))
        self._widget.lcdSessionDuration.display(time)
                        
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