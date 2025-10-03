FROM osrf/ros:humble-desktop

# Install dropbear SSH server and other useful tools
RUN apt-get update && apt-get install -y \
    dropbear \
    sudo \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Set root password for SSH access
RUN echo "root:password" | chpasswd

# Create workdir
RUN mkdir -p /workdir
WORKDIR /workdir

# Set up dropbear SSH configuration
RUN mkdir -p /etc/dropbear && \
    rm -f /etc/dropbear/dropbear_*_host_key && \
    dropbearkey -t rsa -f /etc/dropbear/dropbear_rsa_host_key && \
    dropbearkey -t dss -f /etc/dropbear/dropbear_dss_host_key && \
    dropbearkey -t ecdsa -f /etc/dropbear/dropbear_ecdsa_host_key && \
    dropbearkey -t ed25519 -f /etc/dropbear/dropbear_ed25519_host_key

# Source ROS2 environment in bashrc for all users
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Expose SSH port
EXPOSE 22

# Start dropbear SSH server and then bash
CMD ["bash", "-c", "dropbear -F -E -p 22 && tail -f /dev/null"]