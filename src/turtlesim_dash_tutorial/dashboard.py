#!/usr/bin/env python
# Create a dash server that is also an actionlib client to turtle_actionlib

from __future__ import print_function, division

import os
import sys
import json
import signal

import numpy as np

import rospy
import rospkg
import actionlib

from actionlib_msgs.msg import GoalStatus
from turtlesim.msg import Pose
from turtle_actionlib.msg import ShapeAction, ShapeGoal

# Plotly, Dash, and Flask
import dash
import dash_core_components as dcc
import dash_html_components as html

from flask import jsonify


# The app definition

APP = dash.Dash(
    __name__,
    assets_folder=os.path.join(rospkg.RosPack().get_path('turtlesim_dash_tutorial'), 'dash_assets'),
    external_stylesheets=[
        {
            'href': 'https://stackpath.bootstrapcdn.com/bootstrap/4.3.1/css/bootstrap.min.css',
            'rel': 'stylesheet',
            'integrity': 'sha384-ggOyR0iXCbMQv3Xipma34MD+dH/1fQ784/j6cY/iJTQUOhcWr7x9JvoRxT2MZw1T',
            'crossorigin': 'anonymous',
        },
    ]
)


class Dashboard(object):
    """
    Create a Flask server to display the UI and a ROS node to send commands to
    the turtlesim
    """

    # Flask
    APP_HOST = '0.0.0.0'
    APP_PORT = 8080
    APP_STATUS_URL = '/ros_api/status'
    APP_STATUS_ENDPOINT = 'ros_status'

    # Actions, Topics, and Services
    TURTLE_SHAPE_ACTION_NAME = "turtle_shape"
    TURTLE_POSE_TOPIC = '/turtle1/pose'

    def __init__(self):
        global APP

        # The Flask application
        self._app = APP
        self._flask_server = self._app.server

        # Create the stop signal handler
        signal.signal(signal.SIGINT, self.stop)

        # TODO: Setup the subscribers, action clients, etc.

        # Initialize the application
        self._define_app()

    def start(self):
        self._app.run_server(host=Dashboard.APP_HOST,
                             port=Dashboard.APP_PORT,
                             debug=False)

    def stop(self, *args, **kwargs):
        # Give some time for rospy to shutdown (cannot use rospy now!)
        print("Shutting down Dash server")
        time.sleep(2)
        sys.exit(0)

    def _define_app(self):
        """
        Define the app layout and callbacks here
        """
        # TODO
        pass
