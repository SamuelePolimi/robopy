"""
How to make it work: install pyusb (noite if you are on venv, do python -m pip install pyusb).
Then allow reading the port using udev rules

SUBSYSTEM=="usb", MODE="0660", GROUP="plugdev"

After adding the rule,

sudo sdevadm control --reload
sudo udevadm trigger
"""

import usb.core
import usb.util

import math

DATA_MODE_GRAMS = 2
DATA_MODE_OUNCES = 11

VENDOR_ID = 0x0922
PRODUCT_ID = 0x8003


class ScaleNotConnectedException(Exception):

    def __init__(self):
        Exception.__init__(self, "The scale is not connected or is turned off.")


class Scale(object):

    def get_tare(self):
        raise Exception("Not Implemented. You are calling an interface.")

    def set_tare(self):
        raise Exception("Not Implemented. You are calling an interface.")

    def get_weight(self):
        raise Exception("Not Implemented. You are calling an interface.")

    def get_name(self):
        raise Exception("Not Implemented. You are calling an interface.")


class DymoScale(Scale):

    def __init__(self):

        self.device = usb.core.find(idVendor=VENDOR_ID,
                                    idProduct=PRODUCT_ID)

        if self.device is None:
            raise ScaleNotConnectedException()
        if self.device.is_kernel_driver_active(0):
            self.device.detach_kernel_driver(0)

        self.device.set_configuration()
        self.endpoint = self.device[0][(0, 0)][0]
        self._tare = 0.
        self._name = "DYMO"

    def set_tare(self):
        """
        Set the tare of the scale with the current weight.
        :return:
        """
        self._tare = 0.
        self._tare = self.get_weight_grams()

    def get_tare(self):
        """
        Retreive the current tare
        :rtype: float
        :return:
        """
        return self._tare

    def get_weight(self):
        """
        Gets the weight in grams measured from the scale.
        :return:
        :rtype: float
        """
        attempts = 10
        data = None
        grams = 0
        while data is None and attempts > 0:
            try:
                data = self.device.read(self.endpoint.bEndpointAddress,
                                        self.endpoint.wMaxPacketSize)
            except usb.core.USBError as e:
                data = None
                if e.args == ('Operation timed out',):
                    attempts -= 1
                    continue

        weight = None
        raw_weight = data[4] + data[5] * 256

        if data[2] == DATA_MODE_OUNCES:
            scaling_factor = math.pow(10, (data[3] - 256))
            ounces = raw_weight * scaling_factor
            weight = ounces
        elif data[2] == DATA_MODE_GRAMS:
            grams = raw_weight
            weight = grams

        if data[1] == 5:
            weight = weight * -1

        return weight - self._tare

    def get_name(self):
        return self._name