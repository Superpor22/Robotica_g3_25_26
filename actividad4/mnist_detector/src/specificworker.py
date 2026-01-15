#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2025 by YOUR NAME HERE
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
import numpy as np
import time
import ROBOCOMPMNIST
import traceback
import cv2
import torch
import itertools
import math
import os

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        self.last_roi = None
        self.color = None
        if startup_check:
            self.startup_check()
        else:
            # camera
            self.color = []
            started_camera = False
            while not started_camera:
                try:
                    print("Connecting to Camera360RGB...")
                    self.rgb_original = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
                    print("Camera specs:")
                    print(" width:", self.rgb_original.width)
                    print(" height:", self.rgb_original.height)
                    print(" focalx", self.rgb_original.focalx)
                    print(" focaly", self.rgb_original.focaly)
                    print(" period", self.rgb_original.period)
                    print(" ratio {:.2f}".format(self.rgb_original.width / self.rgb_original.height))
                    started_camera = True
                    print("Connected to Camera360RGB")
                except Ice.Exception as e:
                    traceback.print_exc()
                    print(e, "Trying again CAMERA...")

            # Cargar modelo TorchScript
            model_path = "/home/usuario/robotica_G3/Robotica_g3_25_26/actividad4/mnist_detector/src/my_network.pt"
            self.model = torch.jit.load(model_path)
            self.model.eval()
            print("✅ TorchScript MNIST model loaded")

            # timer
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    @QtCore.Slot()
    def compute(self):

        image = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
        self.color = np.frombuffer(image.image, dtype=np.uint8).reshape(image.height, image.width, 3)
        rect = self.detect_frame(self.color)
        color_copy = self.color.copy()
        if rect is not None:
            x1, y1, x2, y2 = rect
            self.last_roi = rect
            cv2.rectangle(color_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
            roi = self.color[y1:y2, x1:x2]
            # cv2.imshow("Detected ROI", roi)
            digit = self.MNIST_getNumber()
            print("Número detectado:", digit)
        cv2.imshow("Camera360RGB", color_copy)
        cv2.waitKey(1)

    def detect_frame(self, color):
        color_copy = color.copy()
        h_img, w_img = color.shape[:2]

        # Convertir a HSV
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

        # Definir rangos de rojo (hay dos rangos en HSV)
        # Rangos de rojo puro
        lower_red1 = np.array([0, 150, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 150, 100])
        upper_red2 = np.array([180, 255, 255])

        # Crear máscara combinando ambos rangos
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Suavizar la máscara para eliminar ruido
        mask = cv2.medianBlur(mask, 5)

        # Encontrar contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            print("No red contours found")
            return None

        best_cnt = None
        best_score = -1
        candidates = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 0.001 * w_img * h_img:  # descartar muy pequeños
                continue

            # Aproximar contorno a polígono
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.05 * peri, True)

            # Bounding box
            x, y, bw, bh = cv2.boundingRect(approx)
            aspect_ratio = bw / float(bh)

            # Comprobar "casi cuadrado"
            if 0.5 <= aspect_ratio <= 2.0:
                candidates.append((area, x, y, bw, bh))

        # Score basado en cantidad de pixeles rojos en la máscara
        for c in candidates:
            _, x, y, bw, bh = c
            roi = mask[y:y + bh, x:x + bw]
            red_pixels = cv2.countNonZero(roi)
            total_pixels = bw * bh
            red_ratio = red_pixels / total_pixels
            score = red_ratio
            if score > best_score:
                best_score = score
                best_cnt = c

        if best_cnt is not None:
            _, x, y, bw, bh = best_cnt
            margin = int(min(bw, bh) * 5 / 100)  # 5% margen
            x1 = max(0, x + margin)
            y1 = max(0, y + margin)
            x2 = min(w_img, x + bw - margin)
            y2 = min(h_img, y + bh - margin)

            if x2 <= x1 or y2 <= y1:
                return None

            return [x1, y1, x2, y2]
        else:
            return None

    ####################################################################
    def startup_check(self):
        print(f"Testing RoboCompCamera360RGB.TRoi from ifaces.RoboCompCamera360RGB")
        test = ifaces.RoboCompCamera360RGB.TRoi()
        print(f"Testing RoboCompCamera360RGB.TImage from ifaces.RoboCompCamera360RGB")
        test = ifaces.RoboCompCamera360RGB.TImage()
        QTimer.singleShot(200, QApplication.instance().quit)

    def crop_margin(self, img, margin_ratio):
        """
        Recorta un margen proporcional alrededor de la imagen.
        margin_ratio: porcentaje de ancho/alto a recortar (0.1 = 10%)
        """
        h, w = img.shape[:2]
        x_margin = int(w * margin_ratio)
        y_margin = int(h * margin_ratio)

        cropped = img[y_margin:h - y_margin, x_margin:w - x_margin]
        return cropped

    # =============== Methods the component implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getNumber method from MNIST interface
    #
    def MNIST_getNumber(self):
        try:
            if self.last_roi is None or self.color is None:
                return -1

            x1, y1, x2, y2 = self.last_roi
            roi = self.color[y1:y2, x1:x2]
            roi = self.crop_margin(roi, 0.1)
            center_x = (x1 + x2) // 2

            # 1. Escala de grises
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            # 2. Binarización SUAVE (no agresiva)
            _, bw = cv2.threshold(
                gray, 0, 255,
                cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
            )

            # 3. Bounding box del dígito
            coords = cv2.findNonZero(bw)
            if coords is None or len(coords) < 30:
                return -1

            x, y, w, h = cv2.boundingRect(coords)
            digit = bw[y:y + h, x:x + w]

            # 4. Padding tipo MNIST (moderado)
            side = int(max(w, h) * 1.25)
            canvas = np.zeros((side, side), dtype=np.uint8)

            x_offset = (side - w) // 2
            y_offset = (side - h) // 2
            canvas[y_offset:y_offset + h, x_offset:x_offset + w] = digit

            # 5. Resize final
            mnist_img = cv2.resize(canvas, (28, 28))

            # 6. EXACTAMENTE como ToTensor()
            mnist_img = mnist_img.astype(np.float32) / 255.0

            # 7. Tensor [1,1,28,28]
            input_tensor = torch.from_numpy(mnist_img).unsqueeze(0).unsqueeze(0)

            with torch.no_grad():
                output = self.model(input_tensor)
                pred = output.argmax(1).item()

            # DEBUG visual
            cv2.imshow("MNIST input", mnist_img)
            cv2.waitKey(1)

            console.print(f"🧠 Predicted digit: {pred}")
            return ROBOCOMPMNIST.MNISTResult(number=pred, center=center_x)

        except Exception as e:
            console.print("❌ Error in MNIST_getNumber:", e)
            traceback.print_exc()
            return ROBOCOMPMNIST.MNISTResult(number=-1, center=-1)

    # ===================================================================
    # ===================================================================

    ######################
    # From the RoboCompCamera360RGB you can call this methods:
    # RoboCompCamera360RGB.TImage self.camera360rgb_proxy.getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)

    ######################
    # From the RoboCompCamera360RGB you can use this types:
    # ifaces.RoboCompCamera360RGB.TRoi
    # ifaces.RoboCompCamera360RGB.TImage

