# FROM ubuntu:bionic

FROM jupyter/scipy-notebook

USER root

RUN apt-get update && apt-get dist-upgrade -y


ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y build-essential cmake libltdl7-dev libreadline6-dev \
    libncurses5-dev libgsl0-dev python-all-dev python-numpy python-scipy \
    python-matplotlib ipython openmpi-bin libopenmpi-dev python-nose \
    cython wget git autoconf libgsl-dev

    # /usr/local/bin', include files under `/usr/local/include'
# Please re-compile NEST using --with-libneurosim=PATH


##################
### neurosim
##################

RUN git clone https://github.com/INCF/libneurosim.git \
    && cd libneurosim && touch README && autoreconf --install --force && autoconf configure.ac && ./configure && make \
    && make check && make install && make clean distclean \
    && cd .. && rm -rf libneurosim


##################
### install nest
##################

RUN wget https://codeload.github.com/nest/nest-simulator/tar.gz/v2.16.0 && \
    tar -xzvf v2.16.0 && \
    rm -f v2.16.0


RUN mkdir /usr/bin/nest && mkdir nest-build && cd nest-build  && \
    cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/bin/nest -Dwith-gsl=ON -Dwith-libneurosim=/usr/local/bin/libneurosim /home/jovyan/nest-simulator-2.16.0

#  -Dwith-gsl=ON -Dwith-mpi=ON -Dwith-music=ON

RUN cd nest-build && make && make install

# RUN useradd john && chown -R john nest-build
# USER john
# SHELL ["/bin/bash", "-c"]

# source /usr/bin/nest/bin/nest_vars.sh &&
# RUN cd nest-build && make installcheck

##################
### pybullet
##################

# todo: nest braucht python2, glaube ich. Soll das project in python2 oder python3 gemacht werden?
RUN apt-get update && apt-get install -y python3-pip python3-dev python3

USER $NB_UID
ENV JUPYTER_PATH=<directory_for_your_module>:$JUPYTER_PATH

RUN mkdir pra
WORKDIR pra
ADD requirements.txt requirements.txt
RUN python3 -m pip install -r requirements.txt
ADD . .
# CMD ./src/run.py
