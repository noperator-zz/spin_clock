import subprocess
import time

def run(cmd):
    subprocess.call(cmd.split(" "))

if __name__ == "__main__":

    run("echo 21 > /sys/class/gpio/export")
    run("echo out > /sys/class/gpio/gpio21/direction")


    while 1:
        run("echo 1 > /sys/class/gpio/gpio21/value")
        time.sleep(.001)
        run("echo 0 > /sys/class/gpio/gpio21/value")
        time.sleep(.029)
