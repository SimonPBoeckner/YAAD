#Download Stuff

1. apriltag C library from aprilrobotics github
2. opencv and addition modules C++ library
3. WPILibC from the AllWPILib repo
4. Eigen library
5. JSON library
basically whatever is in the cmake.

Clone this repo somewhere, change the paths for stuff and might work.

Ignore commit messages.

The executable can be passed a config path otherwise it will use the default one.

The dashboard currently is viewable at localhost:8080 but can be changed.

Same goes for the network tables, server ip would be the ip of the roborio/other thing thats out now.

I would suggest a server distro like ubuntu, debian or fedora.

after initially setup you can just ssh into it, I would advise setting up systemd service to run the program when the coprocessor is turned on.