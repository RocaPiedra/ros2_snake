first build the image from the acciona folder:
```
docker build -t acciona/foxy:1.0 .
```
then run the container in the background to later open multiple terminals:
```
docker run -d -it acciona/foxy:1.0
```
now check the id of the container:
```
docker ps
```
and use the image to launch an interactive shell:
```
docker exec -it <container_id> bash
```
