FROM ubuntu:22.04

RUN apt-get update && apt-get install -y curl
RUN curl -fsSL https://pixi.sh/install.sh | bash

WORKDIR /app
COPY . .

CMD ["sleep", "infinity"]
