#!/bin/bash

# Aufgabe 2.1: Übertragungsfaktor zwischen dem Torque Eingang und dem Pos
# Ausgang des Servo-Motors

python plotData.py --device /dev/ttyACM0 --duration 5 --torque "-20" PYTHON_TORQUE
python plotData.py --device /dev/ttyACM0 --duration 5 --torque "-10" PYTHON_TORQUE
python plotData.py --device /dev/ttyACM0 --duration 5 --torque "-8" PYTHON_TORQUE
python plotData.py --device /dev/ttyACM0 --duration 5 --torque "-5" PYTHON_TORQUE
python plotData.py --device /dev/ttyACM0 --duration 5 --torque 5 PYTHON_TORQUE
python plotData.py --device /dev/ttyACM0 --duration 5 --torque 8 PYTHON_TORQUE
python plotData.py --device /dev/ttyACM0 --duration 5 --torque 10 PYTHON_TORQUE
python plotData.py --device /dev/ttyACM0 --duration 5 --torque 20 PYTHON_TORQUE


