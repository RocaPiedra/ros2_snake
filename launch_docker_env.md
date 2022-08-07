first build the image from the ros2snake folder:
```
docker build -t ros2snake/foxy:1.0 .
```
then run the container in the background to later open multiple terminals:
```
docker run -d -it ros2snake/foxy:1.0
```
now check the id of the container:
```
docker ps
```
and use the image to launch an interactive shell:
```
docker exec -it <container_id> bash
```
you can launch as many shells as you want with that command
