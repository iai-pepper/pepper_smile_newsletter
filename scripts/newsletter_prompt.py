#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Use executeJS Method"""

import qi
import argparse
import sys
import time
import os
import rospy

def show_prompt():    
    """
    This example uses the executeJS method.
    To Test ALTabletService, you need to run the script ON the robot.
    """
    # Get the service ALTabletService.
    try:
        # Javascript script for displaying a prompt
        # ALTabletBinding is a javascript binding inject in the web page displayed on the tablet
        script = """
            var name = prompt("Write down your email to receive our SMILE-Newsletter ", "your@email.de");
            ALTabletBinding.raiseEvent(name)
        """

        # Don't forget to disconnect the signal at the end
        signalID = 0

        # function called when the signal onJSEvent is triggered
        # by the javascript function ALTabletBinding.raiseEvent(name)
        print('Defining callback')
        def callback(event):
            if not str(event) == 'your@email.de':
                if event:
                    print(event)
                    file.write(str(event) + '\n')
                tts.say('Thank you')
            promise.setValue(True)

        promise = qi.Promise()

        print('Connecting callback')
        # attach the callback function to onJSEvent signal
        signalID = tabletService.onJSEvent.connect(callback)

        # inject and execute the javascript in the current web page displayed
        print('Executing script')
        tabletService.executeJS(script)

        try:
            promise.future().hasValue(3000000)
        except RuntimeError:
            raise RuntimeError('Timeout: no signal triggered')

    except Exception, e:
        print "Error was:", e
        file.close()

    # disconnect the signal
    tabletService.onJSEvent.disconnect(signalID)


if __name__ == "__main__":
    global session
    session = qi.Session()
    try:
        session.connect("tcp://" + str(os.environ['NAO_IP']) + ":" + str(9559))
    # this will give us some light if something breaks while starting the service.
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + str(os.environ['NAO_IP']) + "\" on port " + str(9559)
               + ".\n" + "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    global tabletService
    global tts
    global webview_hidden

    # The file to save in
    global file

    # Promise to comunicate between threads
    global promise

    # Is the webview hidden?
    webview_hidden = True

    tabletService = session.service("ALTabletService")
    tts = session.service("ALTextToSpeech")

    rospy.init_node('pepper_ros_server')

    # Save if it is the first time for loading the website
    first_time = True

    # open file with a timestamp
    ts = int(time.time())
    file = open('smile_email_list_'+str(ts)+'.txt', 'w')

    while not rospy.is_shutdown():
        # Display a website to insert the javascript into
        tabletService.showWebview("http://www.smile-smart-it.de")
        webview_hidden = False
        # First time wait for the website to load
        if first_time:
            time.sleep(3)
        show_prompt()
        rospy.sleep(0.5)

        first_time = False

    # Hide the web view
    tabletService.hideWebview()
    file.close()