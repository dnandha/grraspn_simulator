import os
import sys
from io import BytesIO
from imageio import imwrite
from sshtunnel import SSHTunnelForwarder
import requests

class Client(object):
    def __init__(self):
        if not os.path.isfile("client.conf"):
            print("Please create client.conf with following line:")
            print("USERNAME:PASSWORD@HOST")
            sys.exit()
        with open("client.conf") as f:
            line = f.readline()
            remote_creds, remote_host = line.split('@')
            remote_user, remote_pass = remote_creds.split(':')
            remote_host = remote_host.strip() # WAAAAH
        self.server = self.create_server(remote_host, remote_user, remote_pass)
        self.server.start()

    def create_server(self, remote_host, remote_user, remote_pass):
        remote_port = 22
        local_host = '127.0.0.1'
        local_port = 5001

        return SSHTunnelForwarder(
            (remote_host, remote_port),
            ssh_username=remote_user,
            ssh_password=remote_pass,
            remote_bind_address=(local_host, local_port),
            local_bind_address=(local_host, local_port),
            )

    def send_file(self, filename):
        with open(filename, 'rb') as f:
            files = {
                'imgfile': f
            }
            r = requests.post('http://127.0.0.1:5001/inference', files=files)
            return eval(r.text)

    def send_array(self, imgdata):
        imgbuf = BytesIO()
        imwrite(imgbuf, imgdata, format='png')
        files = {
            'imgfile': imgbuf.getvalue()
        }
        r = requests.post('http://127.0.0.1:5001/inference_raw', files=files)
        return eval(r.text)

    def shutdown(self):
        self.server.stop()

def draw_preds(img, preds):
    import cv2
    import numpy as np

    im = img.copy()
    for p in preds:
        x, y, w, h, a = p
        theta = a * np.pi / 180.0
        cos, sin = np.cos(theta), np.sin(theta)
        rect = [(-w / 2, h / 2), (-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2)]
        rot_rect = [(int(sin * yy + cos * xx + x), int(cos * yy - sin * xx + y))
                for (xx, yy) in rect]

        color = (0, 0, 255)
        thickness = 4
        for i in range(len(rot_rect)):
            if i == len(rot_rect)-1:
                start = rot_rect[-1]
                end = rot_rect[0]
            else:
                start = rot_rect[i]
                end = rot_rect[i+1]

            im = cv2.line(im, start, end, color, thickness)
    return im

if __name__ == "__main__":
    c = Client()
    from imageio import imread
    im = imread(sys.argv[1])
    preds = c.send_array(im)
    #r = c.send_file(sys.argv[1])
    print(preds)

    c.shutdown()
