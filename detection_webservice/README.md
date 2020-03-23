# Detection Webservice

In case you don't have CUDA on your local machine you can use this project to tunnel detection http requests through ssh and receive results from the remote server.

## Installation

* `git clone https://git.ias.informatik.tu-darmstadt.de/nandha/detection_webservice.git`
* `cd detection_webservice`

### Server

* `pip install -r server_requirements.txt`

### Client

* `pip install -r client_requirements.txt`

Create `client.conf` with following line:
`USERNAME:PASSWORD@HOST`
where USERNAME and PASSWORD is your ssh username and password and HOST is the server uri.

## Running

### Server

* `cd detection_webservice`
* `export FLASK_APP=server`
* `flask run`

### Client

* `python client.py <path to imagefile>`
