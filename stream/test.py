import numpy as np
import cv2
import socket

UDP_IP = "0.0.0.0"
UDP_Port = 8088
img_width = 160
img_height = 120

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP,UDP_Port))

while True:
    data, addr = sock.recvfrom(img_width * img_height * 2)
    img = np.frombuffer(data, dtype=np.uint16).reshape((img_height, img_width))
    temp = img[60,80] / 100.0 - 273.15
    img = cv2.normalize(img, None, 0, 65535, cv2.NORM_MINMAX)  # Extend contrast
    img = np.right_shift(img, 8, img)  # Fit data into 8 bits
    img = np.uint8(img)
    cv2.circle(img, (80,60), 1, 0, -1)
    print(f"{temp:.2f}")
    cv2.imshow("Thermal Img", img)
    if cv2.waitKey(1) & 0xFF == ('q'):
        break

cv2.destroyAllWindows()