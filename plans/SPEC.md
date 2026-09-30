# 概要

RTL-SDRでrawIQを受け取り、ISDB-Tのワンセグでデコードしてtsをhttp streamで流す

# cli

## command

ts-stream


## sub-command

list-devices

```sh
ts-stream list-devices
```

server
```sh
ts-stream server --host 0.0.0.0 --port 3000 --device 0
```

# http api

stream(chunked transfer encoding)
```
curl http://localhost:3000/stream?ch=19
```



version
```
curl http://localhost:3000/version
```



# 実装方針

- rustで実装する
- simdを活用する
  - intelやamdなどのamd64やsmcのarm64などに対応させる
  - 少なくとも、n100(intel)やOrangePiPC2で動かす
