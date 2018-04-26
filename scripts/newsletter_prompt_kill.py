#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Use executeJS Method"""

import qi
import argparse
import sys
import time
import os
import rospy


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

    tabletService = session.service("ALTabletService")
    # Hide the web view
    tabletService.hideWebview()