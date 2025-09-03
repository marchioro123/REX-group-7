from time import sleep

LEFTSPEED = 69
RIGHTSPEED = 64

def turnNDegrees(arlo,n):
    print(arlo.go_diff(LEFTSPEED, RIGHTSPEED, 0, 1))
    sleep(2.71*n/360)


# def aheadNMeters(arlo,n):
