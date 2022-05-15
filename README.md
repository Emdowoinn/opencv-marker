# opencv-marker

## Requirements

- OpenCV
- C++ 11
- Cmake 3.0 +

## Display Marker(２次元コードの生成)

```bash
# build
cd DisplayMarker
cmake .
make

#execution
./DisplayMarker "marker-id-1.png" -id=1 -bb=1 -ms=200
```

For M1(arm64) env, use the following commands

```bash
cmake -DCMAKE_SYSTEM_PROCESSOR=arm64 -DCMAKE_OSX_ARCHITECTURES=arm64 .
```

- Marker Dictionary は、DICT_4X4_50 を利用しています。

## Detect Marker(２次元コードの検出)

```bash
#build
cd DetectMarker
cmake .
make

#execution
./DetectMarker
```
