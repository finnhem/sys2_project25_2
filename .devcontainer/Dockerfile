FROM mcr.microsoft.com/devcontainers/base:ubuntu-24.04

# Ensure the default devcontainer user is in the dialout group for serial port access
# This allows access to USB serial devices like /dev/ttyACM0, /dev/ttyUSB0, etc.
RUN if id -u vscode > /dev/null 2>&1; then \
        usermod -aG dialout vscode; \
    elif [ -n "$(getent passwd 1000)" ]; then \
        usermod -aG dialout $(getent passwd 1000 | cut -d: -f1); \
    fi

