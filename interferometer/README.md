# Interferometer

## Preparation

This project requires opencv 3.4.0 installed. Here are the steps to execute in the Raspbian command line:

    sudo apt install ffmpeg
    sudo apt-get install libavcodec-dev libavformat-dev libavdevice-dev
    sudo apt install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
    cd; wget https://github.com/opencv/opencv/archive/3.4.0.zip
    unzip 3.4.0.zip
    mkdir opencv-3.4.0/build
    cd opencv-3.4.0/build
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DWITH_QT=OFF -DWITH_GTK=OFF ..
    make
    sudo make install

_Note:_ If you try to execute the interferometer program and get an error like `Unable to stop the stream: Inappropriate ioctl for device` you may not have installed ffmpeg and its corrosponding libraries prior to running `cmake`. Run through the complete list of instructions above to fix the problem.

The environment must also be set up. All files from the `env` subdirectory in this repository must be copied to their corrosponding locations in the Raspberry Pi filesystem. The files in the `boot` subdirectory replace the file which is already on the Raspberry Pi image. Backing up the original is recommended.
