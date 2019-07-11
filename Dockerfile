FROM jupyter/scipy-notebook
ENV cache_breaker 2019_06_20
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
    && cd libneurosim \
    && touch README \
    && autoreconf --install --force \
    && autoconf configure.ac \
    && ./configure  --with-python=3  --prefix=/libneurosim.install \
    && make && make check && make install && make clean distclean \
    && cd .. && rm -rf libneurosim

# workaround for nest issue #1080
RUN cd /libneurosim.install/lib \
    && ln -s libpy3neurosim.la libpyneurosim.la \
    && ln -s libpy3neurosim.a libpyneurosim.a\
    && ln -s libpy3neurosim.so libpyneurosim.so\
    && ln -s libpy3neurosim.so.0.0.0 libpyneurosim.so.0.0.0


##################
### install nest
##################

RUN wget https://codeload.github.com/nest/nest-simulator/tar.gz/v2.16.0 && \
    tar -xzvf v2.16.0 && \
    rm -f v2.16.0


RUN mkdir /usr/bin/nest && mkdir nest-build && cd nest-build  && \
    cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/bin/nest \
     -Dwith-gsl=ON \
     -Dwith-libneurosim=/libneurosim.install \
     /home/jovyan/nest-simulator-2.16.0

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

# need to compile myself so numpy is used for getCameraImage
# todo: nest braucht python2, glaube ich. Soll das project in python2 oder python3 gemacht werden?
RUN apt-get update && apt-get install -y python3-pip python3-dev python3 && pip3 install numpy


RUN git clone --recurse-submodules https://github.com/bulletphysics/bullet3.git \
 && cd bullet3 \
 && ./build_cmake_pybullet_double.sh

USER root
RUN chown -R $NB_UID /home/jovyan

USER $NB_UID
RUN cd bullet3 && python3 -m pip install .


ENV NEST_INSTALL_DIR=/usr/bin/nest
ENV NEST_DATA_DIR=$NEST_INSTALL_DIR/share/nest
ENV NEST_DOC_DIR=$NEST_INSTALL_DIR/share/doc/nest
ENV NEST_MODULE_PATH=$NEST_INSTALL_DIR/lib/nest
ENV NEST_PYTHON_PREFIX=$NEST_INSTALL_DIR/lib/python3.7/site-packages
ENV PYTHONPATH=$NEST_PYTHON_PREFIX:$PYTHONPATH
ENV PATH=$NEST_INSTALL_DIR/bin:$PATH



# ENV JUPYTER_PATH=<directory_for_your_module>:$JUPYTER_PATH

RUN python3 -m pip install --no-binary :all:  PyNN

RUN python3 -m pip install ipympl

RUN jupyter labextension install @jupyter-widgets/jupyterlab-manager \
    && jupyter labextension install jupyter-matplotlib

# requirements already part of base-notebook
# ADD requirements.txt requirements.txt

RUN mkdir pra
WORKDIR pra
# RUN python3 -m pip install -r requirements.txt
ADD neuro_robotic.ipynb neuro_robotic_readonly.ipynb
# CMD ./src/run.py
