#!/usr/bin/env python

import numpy as np
from tflite_runtime.interpreter import Interpreter

class PointHistoryClassifier:
    def __init__(self, model_path: str):
        self.interpreter = Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    def __call__(self, point_history_list):
        input_data = np.array([point_history_list], dtype=np.float32)
        self.interpreter.set_tensor(self.input_details[0]["index"], input_data)
        self.interpreter.invoke()
        output_data = self.interpreter.get_tensor(self.output_details[0]["index"])
        return int(np.argmax(np.squeeze(output_data)))