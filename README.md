# Tec-gihan Force Sensor Firmware
author credit: [Abhinaba Bhattacharjee](https://www.linkedin.com/in/abhi-bjee)

Preprocessing algorithms for [USL-AP Force Sensor](https://tecgihan.co.jp/en/products/forcesensor-amplifier-interface/forcesensor-3-axis/usl-ap_series/)

## Quick Start
1. Flash `RoboLoadCellFirmware/Palp100N_UL220303/Palp100N_UL220303/Palp100N_UL220303.ino` on a Teensy 4.0
2. Connect USB cable to computer and read using serial via simple python script

```
import numpy as np
import time
import serial

class ForceSensor:
    def __init__(self, port="/dev/ttyACM0", baudrate="115200"):
        self.serial = serial.Serial(port=port)
        self.serial.baudrate = baudrate

        while True:
            while self.serial.inWaiting() == 1:
                pass
            bytes_data = self.serial.readline()
            print(str(bytes_data, encoding="utf-8"))
            if bytes_data.startswith(bytes("T:", "utf-8")):
                print("initialized")
                break
            else:
                self.serial.write(bytes("ACK", "utf-8"))

    def read(self):
        if self.serial.inWaiting() == 0:
            return
        else:
            while self.serial.inWaiting() == 2:
                continue
            self.data_stream = self.serial.readline()
        self.data_string = str(self.data_stream)
        self.elements = self.data_string.split(",")
        Fxyz = np.array(
            [
                (float)(self.elements[1]),
                (float)(self.elements[2]),
                (float)(self.elements[3]),
            ],
            dtype=np.double,
        )
        return Fxyz

if __name__ == "__main__":
    fs = ForceSensor()
    while True:
        Fxyz = fs.read()
        if Fxyz is not None:
            print(Fxyz)
        time.sleep(0.1)
```

If you use this codebase in your paper, please cite:
```
@article{bhattacharjee2022handheld,
  title={A handheld quantifiable soft tissue manipulation device for tracking real-time dispersive force-motion patterns to characterize manual therapy treatment},
  author={Bhattacharjee, Abhinaba and Anwar, Sohel and Chien, Stanley and Loghmani, M Terry},
  journal={IEEE Transactions on Biomedical Engineering},
  volume={70},
  number={5},
  pages={1553--1564},
  year={2022},
  publisher={IEEE}
}
```

