FROM ubuntu:22.04

RUN apt-get update && apt-get install -y curl
RUN curl -fsSL https://pixi.sh/install.sh | bash

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

WORKDIR /app
COPY . .

CMD ["sleep", "infinity"]
