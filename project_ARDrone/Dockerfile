from ubuntu:20.04

ENV TZ="Europe/Moscow"
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install python3-dev python3-pip git wget build-essential libavcodec-dev libswscale-dev libavformat-dev -y

ADD python-ardrone /python-ardrone
RUN cd /python-ardrone && python3 -m pip install .
RUN python3 -m pip install opencv-python numpy
RUN apt install libopencv-dev -y