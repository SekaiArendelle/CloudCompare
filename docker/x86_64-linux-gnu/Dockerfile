FROM ubuntu:22.04

RUN apt-get update && apt-get install -y curl

WORKDIR /app
COPY . .
RUN curl -fsSL https://pixi.sh/install.sh | bash

CMD ["sleep", "infinity"]
