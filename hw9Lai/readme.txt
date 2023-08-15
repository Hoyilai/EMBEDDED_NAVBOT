Instructions to set up the project:
[1] enable the camera by running "sudo raspi-config" and selecting "Interfacing Options" --> "Camera" and enable the camera
[2] run "sudo apt-get install cmake"
[3] in the project directory, "cd raspicam-0.1.5"
[4] "cmake CMakeLists.txt"
[5] "make"
[6] "sudo make install"
[7] "sudo ldconfig"
[8] cd back to the project directory and run "make"
