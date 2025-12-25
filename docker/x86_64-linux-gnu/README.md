```sh
docker build -f ./docker/x86_64-linux-gnu/Dockerfile -t cloud-compare .
docker run -d --name cloud-compare cloud-compare
docker exec -it cloud-compare bash
```

```sh
docker rm cloud-compare -f
docker rmi cloud-compare
```
