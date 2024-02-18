import json
import time
import numpy as np
from datetime import datetime

'''
evaluation metrics
each frame:
1. Task Duration (timestamp), 
2. fly distance (x, y, z position),
3. velocity,
4. isCollided or not?
5. u_ref (mean std)
6. u_safe
7. diff between u_ref and u_safe 

case-by-case
1. # of collisions (exclude floor)
'''


class EvaluationModule():
    def __init__(self) -> None:
        self.data_frame = []

    def frame_update(self, data):
        self.data_frame.append(data)

    def export_data(self):
        export_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        export_filename = "results/data_pilot_p5_"+export_time+".txt"
        print("export to ", export_filename)
        with open (export_filename, "w") as f:
            for data in self.data_frame:
                f.write(json.dumps(data)+"\n")
