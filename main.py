import board
import adafruit_mlx90614
import os

TEMP_THRESHOLD = 'TEMP_THRESHOLD'
i2c = board.I2C()
mlx = adafruit_mlx90614.MLX90614(i2c)

def main():

    while True:
        temp_threshold = float(os.getenv(TEMP_THRESHOLD, '38.0').lower())

        # TODO: Detect faces from camera

        # TODO: Draw box around faces and show it

        # TODO: Print temperature on faces

        noFever = (mlx.object_temperature < temp_threshold)

        if (noFever):
            # TODO: Send signal to servo motor to unlock the door
            print("PASSED")


if __name__ == "__main__":
    main()