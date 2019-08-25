#!/usr/bin/env python
# Create a dash server that is also an actionlib client to turtle_actionlib

from __future__ import print_function, division

import os
import sys
import time
import json
import signal

import numpy as np

from threading import Lock

import rospy
import rospkg
import actionlib

from actionlib_msgs.msg import GoalStatus
from turtlesim.msg import Pose
from turtle_actionlib.msg import ShapeAction, ShapeGoal

# Plotly, Dash, and Flask
import plotly.graph_objs as go

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
    TURTLE_SHAPE_ACTION_NAME = 'turtle_shape'
    TURTLE_POSE_TOPIC = '/turtle1/pose'

    # Constants that determine the behaviour of the dashboard
    # Pose is published at ~62 Hz; so we'll see ~30 sec of history
    POSE_UPDATE_INTERVAL = 5
    POSE_MAX_TIMESTEPS = 2000
    POSE_ATTRIBUTES = ['x', 'y', 'theta', 'linear_velocity', 'angular_velocity']

    def __init__(self):
        global APP

        # The Flask application
        self._app = APP
        self._flask_server = self._app.server

        # Create the stop signal handler
        signal.signal(signal.SIGINT, self.stop)

        # Initialize the variables that we'll be using to save information
        self._server_status = GoalStatus.LOST
        self._pose_history = np.ones(
            (1+len(Dashboard.POSE_ATTRIBUTES), Dashboard.POSE_MAX_TIMESTEPS)) * np.nan
        self._history_length = 0
        self._pose_history_lock = Lock()

        # Setup the subscribers, action clients, etc.
        self._shape_client = actionlib.SimpleActionClient(Dashboard.TURTLE_SHAPE_ACTION_NAME, ShapeAction)
        self._pose_sub = rospy.Subscriber(Dashboard.TURTLE_POSE_TOPIC, Pose, self._on_pose)

        # Initialize the application
        self._define_app()

    @property
    def pose_history(self):
        return self._pose_history[:, :self._history_length]

    def start(self):
        rospy.loginfo("Connecting to turtle_shape...")
        self._shape_client.wait_for_server()
        rospy.loginfo("...turtle_shape connected.")
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
        # Define each component of the page
        pose_graph_layout = html.Div(dcc.Graph(id='pose', style={ 'width': '100%' }), className='row')

        # String them all together in a single page
        self._app.layout = html.Div(
            [
                html.Div(html.H3('Pose History', className='col'), className='row'),
                pose_graph_layout,
                dcc.Interval(id='interval-component',
                             n_intervals=0,
                             interval=(Dashboard.POSE_UPDATE_INTERVAL * 1000)),
            ],
            className="container"
        )

        # Define callbacks to update the elements on the page
        self._app.callback(
            dash.dependencies.Output('pose', 'figure'),
            [dash.dependencies.Input('interval-component', 'n_intervals')]
        )(self._define_pose_history_callback())

    def _define_pose_history_callback(self):
        """
        Define a callback that will be invoked on every update of the interval
        component. Keep in mind that we return a callback here; not a result
        """
        def pose_history_callback(n_intervals):
            # Get a view into the latest pose history
            pose_history = self.pose_history

            # Create the output graph
            data = [
                go.Scatter(
                    name=attr,
                    x=pose_history[0, :],
                    y=pose_history[idx+1, :],
                    mode='lines+markers'
                )
                for idx, attr in enumerate(Dashboard.POSE_ATTRIBUTES)
            ]
            layout = go.Layout(
                showlegend=True,
                height=500,
                yaxis=dict(
                    fixedrange=True
                ),
                margin=dict(
                    autoexpand=True
                )
            )

            return { 'data': data, 'layout': layout }

        return pose_history_callback

    def _on_pose(self, msg):
        """
        The callback for the position of the turtle on
        :const:`TURTLE_POSE_TOPIC`
        """
        if self._history_length == Dashboard.POSE_MAX_TIMESTEPS:
            self._pose_history[:, :-1] = self._pose_history[:, 1:]
        else:
            self._history_length += 1

        self._pose_history[:, self._history_length-1] = [
            rospy.Time.now().to_time() % 1000,
            msg.x,
            msg.y,
            msg.theta,
            msg.linear_velocity,
            msg.angular_velocity,
        ]
