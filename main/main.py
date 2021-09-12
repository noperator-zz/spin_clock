import cv2
import cv2.cv2 as cv2
import numpy as np
import os
import math

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)


map1 = np.array([0]*500*32*2).reshape((500, 32, 2)).astype('float32')
for line in range(500):
    for px in range(32):
        x, y = pol2cart(px, math.pi * 2 * line / 500)
        map1[line][px] = [int(x)+32, int(y)+32]


polar_lut = np.zeros(shape=(500, 32, 2)).astype('uint8')
offset_lut = np.zeros(shape=(500, 32, 2))

for line in range(500):
    for px in range(32):
        x, y = pol2cart(px, math.pi * 2 * line / 500)
        x += 32
        y += 32
        fx = int(x)
        fy = int(y)
        ox = x - fx
        oy = y - fy
        polar_lut[line][px] = (fx, fy)
        offset_lut[line][px] = (ox, oy)


def make_polar(img):
    op = np.zeros(shape=(500, 32, 3)).astype('uint8')
    for line in range(500):
        for px in range(32):
            ox, oy = offset_lut[line][px]
            nx, ny = polar_lut[line][px]
            tl = img[ny][nx]
            tr = img[ny][min(63, nx+1)]
            bl = img[min(63, ny+1)][nx]
            br = img[min(63, ny+1)][min(63, nx+1)]

            pl = tl * (1 - oy) + bl * oy
            pr = tr * (1 - oy) + br * oy
            p = pl * (1 - ox) + pr * ox
            op[line][px] = p
    return op

img = cv2.imread('test.bmp', 1)
# smi = os.open("/dev/smi", os.O_RDWR)

while 1:
    polar_image = cv2.linearPolar(img, (img.shape[0]/2, img.shape[1]/2), img.shape[0]/2, cv2.WARP_FILL_OUTLIERS)
    polar_image = cv2.warpPolar(img, (img.shape[0]/2, 500), (img.shape[0]/2, img.shape[1]/2), img.shape[0]/2, cv2.WARP_FILL_OUTLIERS)
    # polar_image = cv2.remap(img, map1, None, cv2.INTER_NEAREST)
    polar_image = make_polar(img)

    cv2.imshow("Polar Image", polar_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    output = [bytearray(l) for l in cv2.remap(img, map1, None, cv2.INTER_NEAREST).reshape((500, img.shape[0] * 3/2))]

    for line in output:
        os.write(smi, line)