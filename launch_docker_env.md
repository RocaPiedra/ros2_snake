Build the image from the ros2snake folder:
```
docker build -t ros2snake/foxy:1.0 .
```

*If you have any issue with the build at this point, delete the build/ install/ and log/ folders to avoid issues:*

*From dev_ws:* ```rm -r build/ install/ log/```

then run the container in the background to later open multiple terminals (or run it directly without the -d flag):
```
docker run -d -it --rm \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ros2snake/foxy:1.0
```
now check the id or name of the container:
```
docker ps
```
and use the image to launch an interactive shell:
```
docker exec -it <container_id/container_name> bash
```
you can launch as many shells as you want with that command and GUI should work. If not, check that your DISPLAY is set, exporting the variable:
```
export DISPLAY=:0.0
```
or adding to your ~/.bashrc and reloading the terminal:
```
echo 'export DISPLAY=:0.0' >> ~/.bashrc 
```

#### Known issues:

- If you have any issue regarding qt when trying to use the GUI in the container, it may be a permissions issue. Try this before running it:
```
xhost +local:docker
```
More info:
https://github.com/ericspod/DicomBrowser/issues/3

- If entrypoint does not work, source manually the setup files in the container:
```
. /opt/ros/foxy/setup.bash
```
```
. /dev_ws/install/setup.bash
```