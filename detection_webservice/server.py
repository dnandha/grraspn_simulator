import os
from io import BytesIO
from imageio import imread
from werkzeug.utils import secure_filename
from flask import Flask, flash, request, redirect
app = Flask(__name__)

app.config['UPLOAD_FOLDER'] = "./upload"
app.config['MAX_CONTENT_LENGTH'] = 16 * 1024 * 1024 # 16 MB

if not os.path.isdir(app.config['UPLOAD_FOLDER']):
    os.mkdir(app.config['UPLOAD_FOLDER'])

@app.route("/")
def home():
    return "Use /inference for detection"

@app.route("/inference", methods=['POST'])
def infer():
    imgfile = request.files.get('imgfile')
    if imgfile is None or not imgfile.filename:
        return "No image selected for upload"
    else:
        imgname = secure_filename(imgfile.filename)

        # save image to folder
        imgfile.save(os.path.join(app.config['UPLOAD_FOLDER'], imgname))

        # TODO: do detection here and return detection results
        return "GRASP"

@app.route("/inference_raw", methods=['POST'])
def infer_raw():
    imgfile = request.files.get('imgfile')
    if imgfile is None:
        return "No image given"
    else:
        imgbuf = BytesIO()
        imgfile.save(imgbuf)

        imgdata = imread(imgbuf.getvalue(), format='png')

        # TODO: do detection herer and return grasp
        return "GRASP"

if __name__ == "__main__":
    app.run()
