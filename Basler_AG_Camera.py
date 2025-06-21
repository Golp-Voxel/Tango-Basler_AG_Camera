# -*- coding: utf-8 -*-
#
# This file is part of the Basler_AG_Camera project
#
#
#
# Distributed under the terms of the GPL license.
# See LICENSE.txt for more info.

"""
Basler AG Camera

This class is to integrate a Basler AG Camera.
After installing the Server on the jive/astor you will need to use the Wizard tool 
to difine the proparties of that particular device.
"""

# PROTECTED REGION ID(Basler_AG_Camera.system_imports) ENABLED START #
# PROTECTED REGION END #    //  Basler_AG_Camera.system_imports

# PyTango imports
import tango
from tango import DebugIt
from tango.server import run
from tango.server import Device
from tango.server import attribute, command
from tango.server import device_property
from tango import AttrQuality, DispLevel, DevState
from tango import AttrWriteType, PipeWriteType
# Additional import
# PROTECTED REGION ID(Basler_AG_Camera.additionnal_import) ENABLED START #
from pypylon import pylon
import numpy as np
import os
from threading import Thread
from tango import AttrWriteType, DevLong

stop_threads = True
# PROTECTED REGION END #    //  Basler_AG_Camera.additionnal_import

__all__ = ["Basler_AG_Camera", "main"]


class Basler_AG_Camera(Device):
    """
    This class is to integrate a Basler AG Camera.
    After installing the Server on the jive/astor you will need to use the Wizard tool 
    to difine the proparties of that particular device.

    **Properties:**

    - Device Property
        CameraID
            - Type:'str'
    """
    # PROTECTED REGION ID(Basler_AG_Camera.class_variable) ENABLED START #
    camera = None
    my_thread = None
    # PROTECTED REGION END #    //  Basler_AG_Camera.class_variable

    # -----------------
    # Device Properties
    # -----------------

    CameraID = device_property(
        dtype='str',
        mandatory=True
    )

    # ----------
    # Attributes
    # ----------

    ExposureTime = attribute(
        dtype='DevDouble',
        access=AttrWriteType.READ_WRITE,
        label="Exposure time of the camera",
        unit="ms",
        display_unit="ms",
        doc="Exposure time of the camera  in ms",
    )

    Gain = attribute(
        dtype='DevDouble',
        access=AttrWriteType.READ_WRITE,
    )

    FramesTrigger = attribute(
        dtype='DevUShort',
        access=AttrWriteType.READ_WRITE,
    )

    ROI = attribute(
        dtype=('DevLong',),
        access=AttrWriteType.READ_WRITE,
        max_dim_x=4,
        doc="This attribute is expeting to recive the [offset_x,offset_y,width,height]",
    )

    Image = attribute(
        dtype=(('DevUShort',),),
        max_dim_x=2474, max_dim_y=2474,
    )

    # ---------------
    # General methods
    # ---------------

    def init_device(self):
        """Initializes the attributes and properties of the Basler_AG_Camera."""
        Device.init_device(self)
        self._exposure_time = 0.0
        self._gain = 0.0
        self._frames_trigger = 0
        self._r_oi = (0,)
        self._image = ((0,),)
        # PROTECTED REGION ID(Basler_AG_Camera.init_device) ENABLED START #
        self.info_stream("\r Try to start the Basler_AG_Camera Driver \r")
        if self.CameraID:
            print(self.CameraID)
            tl_factory = pylon.TlFactory.GetInstance()
            devices = tl_factory.EnumerateDevices()

            for device in devices:
                if device.GetSerialNumber() == self.CameraID:
                    self.camera = pylon.InstantCamera(tl_factory.CreateDevice(device))
                    self.camera.Open()
                    print(f"Connected to camera: {self.camera.GetDeviceInfo().GetModelName()} (SN: {self.CameraID})")
                    self.set_state(DevState.ON)
                    return

            raise Exception(f"Camera with serial number {self.CameraID} not found.")
        else:
            print("You need to setup the divece")
            self.set_state(DevState.FAULT)
        # PROTECTED REGION END #    //  Basler_AG_Camera.init_device

    def always_executed_hook(self):
        """Method always executed before any TANGO command is executed."""
        # PROTECTED REGION ID(Basler_AG_Camera.always_executed_hook) ENABLED START #
        # PROTECTED REGION END #    //  Basler_AG_Camera.always_executed_hook

    def delete_device(self):
        """Hook to delete resources allocated in init_device.

        This method allows for any memory or other resources allocated in the
        init_device method to be released.  This method is called by the device
        destructor and by the device Init command.
        """
        # PROTECTED REGION ID(Basler_AG_Camera.delete_device) ENABLED START #
        if self.camera:
            if self.camera.IsGrabbing():
                self.camera.StopGrabbing()
            if self.camera.IsOpen():
                self.camera.Close()
            print("Camera connection closed.")
            self.camera = None
        # PROTECTED REGION END #    //  Basler_AG_Camera.delete_device

    # ------------------
    # Attributes methods
    # ------------------

    def read_ExposureTime(self):
        # PROTECTED REGION ID(Basler_AG_Camera.ExposureTime_read) ENABLED START #
        """Return the ExposureTime attribute."""
        return self._exposure_time
        # PROTECTED REGION END #    //  Basler_AG_Camera.ExposureTime_read
    def write_ExposureTime(self, value):
        # PROTECTED REGION ID(Basler_AG_Camera.ExposureTime_write) ENABLED START #
        """
        Set the exposure time in microseconds.
        """
        self._exposure_time =  value
        nodemap = self.camera.GetNodeMap()
        exposure_node = nodemap.GetNode("ExposureTime")

        if exposure_node:
            exposure_auto = nodemap.GetNode("ExposureAuto")
            exposure_auto.SetValue("Off")
            exposure_time_us = value * 1000.0
            exposure_node.SetValue(exposure_time_us)
            print(f"Exposure time set to: {value} ms")
        else:
            print("ExposureTime node is not writable on this camera.")
        return

        # PROTECTED REGION END #    //  Basler_AG_Camera.ExposureTime_write
    def read_Gain(self):
        # PROTECTED REGION ID(Basler_AG_Camera.Gain_read) ENABLED START #
        """Return the Gain attribute."""
        return self._gain
        # PROTECTED REGION END #    //  Basler_AG_Camera.Gain_read
    def write_Gain(self, value):
        # PROTECTED REGION ID(Basler_AG_Camera.Gain_write) ENABLED START #
        """
        Set the camera's analog gain (in dB).
        """
        nodemap = self.camera.GetNodeMap()
        gain_node = nodemap.GetNode("Gain")
        
        if gain_node:
            gain_auto = nodemap.GetNode("GainAuto")
            gain_auto.SetValue("Off")
            gain_node.SetValue(value)
            self._gain = value
            print(f"Gain set to: {value} dB")
        else:
            print("Gain node is not writable on this camera.")
        pass
        # PROTECTED REGION END #    //  Basler_AG_Camera.Gain_write
    def read_FramesTrigger(self):
        # PROTECTED REGION ID(Basler_AG_Camera.FramesTrigger_read) ENABLED START #
        """Return the FramesTrigger attribute."""
        return self._frames_trigger
        # PROTECTED REGION END #    //  Basler_AG_Camera.FramesTrigger_read
    def write_FramesTrigger(self, value):
        # PROTECTED REGION ID(Basler_AG_Camera.FramesTrigger_write) ENABLED START #
        """Set the FramesTrigger attribute."""
        pass
        # PROTECTED REGION END #    //  Basler_AG_Camera.FramesTrigger_write
    def read_ROI(self):
        # PROTECTED REGION ID(Basler_AG_Camera.ROI_read) ENABLED START #
        """Return the ROI attribute."""
        return self._r_oi
        # PROTECTED REGION END #    //  Basler_AG_Camera.ROI_read
    def write_ROI(self, value):
        # PROTECTED REGION ID(Basler_AG_Camera.ROI_write) ENABLED START #
        """Set the ROI attribute."""
        nodemap = self.camera.GetNodeMap()
        if self.camera.IsGrabbing():
            self.camera.StopGrabbing()

        offset_x, offset_y, width, height = value
        print(f"x={offset_x}, y={offset_y}, width={width}, height={height}")

        for name, val in [("Width", width), ("Height", height), ("OffsetX", offset_x), ("OffsetY", offset_y)]:
            node = nodemap.GetNode(name)
            if node is None:
                print(f"Node {name} not found")
                continue
            if hasattr(node, 'SetValue'):
                try:
                    # Cast to int explicitly
                    node.SetValue(int(val))
                    print(f"{name} set to {int(val)}")
                except Exception as e:
                    print(f"Failed to set {name}: {e}")
            else:
                print(f"Node {name} does not support SetValue()")
        self._r_oi = value
        # PROTECTED REGION END #    //  Basler_AG_Camera.ROI_write
    def read_Image(self):
        # PROTECTED REGION ID(Basler_AG_Camera.Image_read) ENABLED START #
        """Return the Image attribute."""
        return self._image
        # PROTECTED REGION END #    //  Basler_AG_Camera.Image_read
    # --------
    # Commands
    # --------


    @command(
        dtype_out='DevString',
    )
    @DebugIt()
    def StartAcqusition(self):
        # PROTECTED REGION ID(Basler_AG_Camera.StartAcqusition) ENABLED START #
        """
        This command start the loop of acquiring a image from the camera
        :rtype: PyTango.DevString
        """        
        global stop_threads               
        stop_threads = False
        self.my_thread = Thread(target = self.get_image)
        
        self.set_state(DevState.RUNNING)
        self.my_thread.start()
        return ""
        # PROTECTED REGION END #    //  Basler_AG_Camera.StartAcqusition

    @command(
        dtype_in='DevString',
        doc_in="A JSON converted in to a string with the following structure",
    )
    @DebugIt()
    def ChangeParameters(self, argin):
        # PROTECTED REGION ID(Basler_AG_Camera.ChangeParameters) ENABLED START #
        """
            This command allows the user to change multiple parameters of the camera at the same time such as:
            Exposure Time
            ROI
            Gain
        :param argin: A JSON converted in to a string with the following structure
        :type argin: PyTango.DevString

        :rtype: PyTango.DevVoid
        """

        # PROTECTED REGION END #    //  Basler_AG_Camera.ChangeParameters

    @command(
    )
    @DebugIt()
    def StopAcqusition(self):
        # PROTECTED REGION ID(Basler_AG_Camera.StopAcqusition) ENABLED START #
        """
        Stops the loop that takes images
        :rtype: PyTango.DevVoid
        """
        global stop_threads               
        stop_threads = True
        try:
            self.my_thread.join()
            self.set_state(DevState.ON)
        except:
            print("Cloud not join the theard maybe none exist.")
        # PROTECTED REGION END #    //  Basler_AG_Camera.StopAcqusition

    @command(
    )
    @DebugIt()
    def Snap(self):
        # PROTECTED REGION ID(Basler_AG_Camera.Snap) ENABLED START #
        """
        Takes a image and send it to the user
        :rtype: PyTango.DevVoid
        """
        global stop_threads
        if stop_threads:
            self.get_image()
        else:
            print("It is on the Acqusition mode")
        pass
        # PROTECTED REGION END #    //  Basler_AG_Camera.Snap

    @command(
        dtype_in='DevBoolean',
        doc_in="On/Off",
    )
    @DebugIt()
    def ToggleExpouserAuto(self, argin):
        # PROTECTED REGION ID(Basler_AG_Camera.ToggleExpouserAuto) ENABLED START #
        """
        This will allow the On/Off of the ExpouserAuto mode of the camera
        :param argin: On/Off
        :type argin: PyTango.DevBoolean

        :rtype: PyTango.DevVoid
        """
        nodemap = self.camera.GetNodeMap()
        exposure_auto = nodemap.GetNode("ExposureAuto")
        if argin:
            exposure_auto.SetValue("Continuous")
        else:
            exposure_auto.SetValue("Off")
        return
        # PROTECTED REGION END #    //  Basler_AG_Camera.ToggleExpouserAuto

    @command(
        dtype_in='DevBoolean',
        doc_in="On/Off",
    )
    @DebugIt()
    def ToggleGainrAuto(self, argin):
        # PROTECTED REGION ID(Basler_AG_Camera.ToggleGainrAuto) ENABLED START #
        """
        This will allow the On/Off of the ExpouserAuto mode of the camera
        :param argin: On/Off
        :type argin: PyTango.DevBoolean

        :rtype: PyTango.DevVoid
        """
        nodemap = self.camera.GetNodeMap()
        gain_auto = nodemap.GetNode("GainAuto")
        if argin:
            gain_auto.SetValue("Continuous")
        else:
            gain_auto.SetValue("Off")
        return
        # PROTECTED REGION END #    //  Basler_AG_Camera.ToggleGainrAuto

# ----------
# Run server
# ----------

# PROTECTED REGION ID(Basler_AG_Camera.custom_code) ENABLED START #
    def get_image(self):
        while True:
            if not self.camera or not self.camera.IsOpen():
                raise Exception("Camera not initialized.")

            self.camera.StartGrabbingMax(1)

            converter = pylon.ImageFormatConverter()
            converter.OutputPixelFormat = pylon.PixelType_RGB8packed
            converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

            while self.camera.IsGrabbing():
                grab_result = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grab_result.GrabSucceeded():
                    image = converter.Convert(grab_result)
                    print(f"Image captured: {image.GetWidth()} x {image.GetHeight()}")
                    self._image = grab_result.Array
                    # Optional: return image.GetArray() if you want to use it with OpenCV, etc.
                else:
                    print("Image grab failed:", grab_result.ErrorCode, grab_result.ErrorDescription)
                grab_result.Release()

            self.camera.StopGrabbing()
            global stop_threads
            if stop_threads:
                break
# PROTECTED REGION END #    //  Basler_AG_Camera.custom_code


def main(args=None, **kwargs):
    """Main function of the Basler_AG_Camera module."""
    # PROTECTED REGION ID(Basler_AG_Camera.main) ENABLED START #
    return run((Basler_AG_Camera,), args=args, **kwargs)
    # PROTECTED REGION END #    //  Basler_AG_Camera.main

# PROTECTED REGION ID(Basler_AG_Camera.custom_functions) ENABLED START #
# PROTECTED REGION END #    //  Basler_AG_Camera.custom_functions


if __name__ == '__main__':
    main()
