FROM debian:bookworm
RUN apt update -qq && \
apt install -y -qq --no-install-recommends software-properties-common gpgv2 && \
apt install -qq -y --no-install-recommends \
bc \
bison \
build-essential \
ccache \
cmake \
curl \
dfu-util \
flex \
gcc \
git \
gperf \
jq \
libffi-dev \
libncurses-dev \
libssl-dev \
libusb-1.0-0 \
make \
ninja-build \
python3 \
python3-dev \
python3-pip \
python3-venv \
srecord \
unzip \
wget \
xz-utils \
&& apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

#
# Install Python components
#
RUN curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
RUN python3 get-platformio.py
RUN python3 -m pip install --break-system-packages pipx setuptools virtualenv
RUN python3 -m pipx ensurepath
RUN python3 -V
RUN mkdir -p /usr/local/bin
RUN ln -s /root/.platformio/penv/bin/platformio /usr/local/bin/platformio
RUN ln -s /root/.platformio/penv/bin/pio /usr/local/bin/pio
RUN ln -s /root/.platformio/penv/bin/piodebuggdb /usr/local/bin/piodebuggdb
RUN pio platform install raspberrypi
CMD /bin/bash

