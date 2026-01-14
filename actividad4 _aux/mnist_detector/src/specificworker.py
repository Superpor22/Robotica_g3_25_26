#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2026 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import torch
import numpy as np
import cv2

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        self.model.load_state_dict(torch.load('../my_network.pt', map_location='cpu', weights_only=False))
        self.model.eval()

        self.current_number = -1
        self.confidence = 0.0
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""


    @QtCore.Slot()
    def compute(self):
        print('SpecificWorker.compute...')

        img = self.camera360rgb_proxy.getImage()

        # 1. Convertir a numpy
        
        frame = np.frombuffer(img.image, dtype=np.uint8)
        frame = frame.reshape(img.height, img.width, 3)

        # 2. Pasar a gris
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 3. Buscar región negra (muy simplificado)
        _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) == 0:
            self.current_number = -1
            return True

        # 4. Tomar el contorno más grande
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)

        roi = gray[y:y+h, x:x+w]
        roi = cv2.resize(roi, (28, 28))
        roi = roi.astype("float32") / 255.0

        # 5. Tensor
        roi = torch.tensor(roi).unsqueeze(0).unsqueeze(0).to(self.device)

        # 6. Inferencia
        with torch.no_grad():
            output = self.model(roi)
            probs = torch.softmax(output, dim=1)
            self.current_number = torch.argmax(probs).item()
            self.confidence = probs[0, self.current_number].item()

        return True


        # computeCODE
        # try:
        #   self.differentialrobot_proxy.setSpeedBase(100, 0)
        # except Ice.Exception as e:
        #   traceback.print_exc()
        #   print(e)

        # The API of python-innermodel is not exactly the same as the C++ version
        # self.innermodel.updateTransformValues('head_rot_tilt_pose', 0, 0, 0, 1.3, 0, 0)
        # z = librobocomp_qmat.QVec(3,0)
        # r = self.innermodel.transform('rgbd', z, 'laser')
        # r.printvector('d')
        # print(r[0], r[1], r[2])

        return True

    def startup_check(self):
        print(f"Testing RoboCompCamera360RGB.TRoi from ifaces.RoboCompCamera360RGB")
        test = ifaces.RoboCompCamera360RGB.TRoi()
        print(f"Testing RoboCompCamera360RGB.TImage from ifaces.RoboCompCamera360RGB")
        test = ifaces.RoboCompCamera360RGB.TImage()
        print(f"Testing MNIST.MNISTResult from ifaces.MNIST")
        test = ifaces.MNIST.MNISTResult()
        QTimer.singleShot(200, QApplication.instance().quit)




    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of get_number method from MNIST interface
    #
    def MNIST_get_number(self):

        result = ifaces.MNIST.MNISTResult()
        result.number = self.current_number
        result.confidence = self.confidence
        return result
    #
    # IMPLEMENTATION of process_image method from MNIST interface
    #
    def MNIST_process_image(self):

        return ret
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompCamera360RGB you can call this methods:
    # RoboCompCamera360RGB.TImage self.camera360rgb_proxy.getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)

    ######################
    # From the RoboCompCamera360RGB you can use this types:
    # ifaces.RoboCompCamera360RGB.TRoi
    # ifaces.RoboCompCamera360RGB.TImage

    ######################
    # From the MNIST you can use this types:
    # ifaces.MNIST.MNISTResult


