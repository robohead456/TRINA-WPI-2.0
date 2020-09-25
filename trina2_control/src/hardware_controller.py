#!/usr/bin/env python3

'''
Project: Simple MotionV2 based drive controller for Carla

@file </carla_control.py>

@author Prajankya Sonar - <prajankya@gmail.com>

MIT License
Copyright (c) 2020 Prajankya Sonar
'''

from __future__ import print_function

from smv2_drive import DriveController

import time


class MyController():
    _isMotorConnected = False
    _positionFeedback = 0
    _feedbackAngle = 0

    # pygame.init()
    # pygame.font.init()
    # world = None

    ###################### Init #######################
    # def __init__(self, args):
    def __init__(self):
        self._controller = DriveController("/dev/ttyUSB0", 1, False)

        ##### Register Callbacks #####
        self._controller.logCallback = self._logCallback
        self._controller.errorCallback = self._errorCallback
        self._controller.readingCallback = self._readingCallback
        self._controller.connectedCallback = self._connectedCallback

        self._ready_to_operate = False

        #try:
            # # initialize carla client
            # client = carla.Client(args.host, args.port)
            # client.set_timeout(4.0)

            # self._display = pygame.display.set_mode(
            #     (args.width, args.height),
            #     pygame.HWSURFACE | pygame.DOUBLEBUF)

            # self._hud = HUD(args.width, args.height)
            # self._world = World(client.get_world(), self._hud, args.filter)
            # self._keyboardController = KeyboardControl(self._world)

            # if args.agent == "Roaming":
            #     self._agent = RoamingAgent(self._world.player)
            # else:
            #     self._agent = BasicAgent(self._world.player)
            #     spawn_point = self._world.map.get_spawn_points()[0]
            #     self._agent.set_destination((spawn_point.location.x,
            #                                  spawn_point.location.y,
            #                                  spawn_point.location.z))

            # self._clock = pygame.time.Clock()

            # Connect and start
        self._controller.connect()

        print("WAITING TO CONNECT......")
        while True:
            time.sleep(0.2)
            if self._isMotorConnected:
                print("Connected !")
                break

        # Motor is connected

        # # turn motor to home
        self._goToAngle(0, 1000)

        self._controller.setAddedConstantTorque(150)

        while True:
            time.sleep(0.5)  # keep this high to debounce
            if abs(self._feedbackAngle) < 5:
                print("Motor to Zero!")
                break

        self._ready_to_operate = True

        # time.sleep(1)
        # self._controller.disableSetpointTracking(1500, 0.001)

        # while not self._keyboardController.parse_events():
        #     self._runLoop()
        # finally:
        #     self.__del__()



    def _computeControl(self, control_steer):

        # # as soon as the server is ready continue!
        # self._world.world.wait_for_tick(10.0)

        # self._world.tick(self._clock)
        # self._world.render(self._display)
        # pygame.display.flip()

        # # skip traffic lights
        # if self._world.player.is_at_traffic_light():
        #     traffic_light = self._world.player.get_traffic_light()
        #     if traffic_light.get_state() == carla.TrafficLightState.Red:
        #         self._hud.notification("Traffic light changed!")
        #         traffic_light.set_state(carla.TrafficLightState.Green)

        # control = self._agent.run_step()
        # # control.brake = False
        # # control.throttle = 1
        # # control.hand_brake = False
        # control.manual_gear_shift = False

        # Move motor
        steering_scaler = 360 # instead of 90
        _steer = int(control_steer*steering_scaler)
        print("Steering setpoint is ", _steer)

        self._goToAngle(_steer, 5000, 2.0, 0.75, 1.0)
        #self._goToAngle(_steer, 2500, 1.5, 1.2)

        steering_output = self._feedbackSteer

        return steering_output

        # self._world.player.apply_control(control)


    def haptic_feedback(self, torque_error_val, control_steer, Kf=1, Ks=1):
        torque_val = int(Kf*torque_error_val + Ks*control_steer)

        self._controller.setTorqueSetpoint(torque_val)

        steering_output = self._feedbackSteer

        return steering_output




    def getAngleandTorqueReadings(self):
        return self._feedbackAngle, self._torqueSetpoint


    def _goToAngle(self, pos, max_torque=2000, Kp=1.6, Ki=0, Kd=0.1):
        self._controller.setAbsoluteSetpoint(
            int((pos/360)*10000), max_torque, Kp, Ki, Kd)



    ################### Destructor ###################
    def __del__(self):
        ######################
        # nullptr ALL CALLBACKS before calling desctructor
        # Very very important to mitigate deadlock while exiting
        ######################
        self._controller.logCallback = None
        self._controller.errorCallback = None
        self._controller.readingCallback = None
        self._controller.connectedCallback = None
        # try:
        #     if self._world is not None:
        #         self._world.destroy()
        # finally:
        #     pass

        # pygame.quit()
        # print("Quitting....")


    ################# Event Callbacks #################
    def _logCallback(self, obj):
        # logType
        # message
        print("LOG>", obj.message)
        pass

    def _connectedCallback(self, obj):
        # isConnected
        print("CON>", obj.isConnected)
        self._isMotorConnected = obj.isConnected

    def _errorCallback(self, obj):
        # bool trackingError,
        # bool driveFault
        print("Error>", obj.trackingError, obj.driveFault)

    def _readingCallback(self, obj):
        # int posSetpoint
        # int posFeedback
        # int torqueSetpoint
        # print("Reading: posTrackingError > ", obj.posSetpoint - obj.posFeedback)
        #       obj.posFeedback, obj.torqueSetpoint)

        self._positionFeedback = obj.posFeedback
        self._torqueSetpoint = obj.torqueSetpoint
        self._feedbackAngle = (obj.posFeedback / 10000)*360

        # For mapping steering rotation to CARLA variable:
            # use 720 i.e. two turns/revs
            # Logitech uses 900
            # Regular vehicle is ~1080
        _rotation = 720
        self._feedbackSteer = (self._feedbackAngle / _rotation)*2 

        # print("Reading>", self._feedbackSteer, self._feedbackAngle)
