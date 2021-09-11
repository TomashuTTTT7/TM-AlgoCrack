# Speedslide

Speedslide is not a bug, but rather an oversight.

While sliding, your car gets constant acceleration force of 10 N no matter the speed.

While normal driving, your acceleration force is dependent on the speed:
  - 0-100 km/h - 16 N
  - 101-200 km/h - 11 N
  - 201-400 km/h - 7 N
  - 401-800 km/h - 5,5 N
  - 801+ km/h - 1 N

Above 401 km/h resultant force of sliding acceleration and sliding deceleration (with small angle) is bigger than normal driving acceleration. This is why speedslide gives you speed.

Resultant acceleration is a combination of sliding and normal acceleration based on sliding factor. 

`speedslide_quality = (sidefriction - maxsidefriction) / maxsidefriction `

sliding factor is a `speedslide_quality` limited to range [0, 1]

It is the best to have sliding factor of 1, because you utilize entire speedslide potential. Speedslide quality bigger than 1 will give you bigger penalty from drifting and gives you no more advantage.

- `speedslide_quality < 1`: you don't utilize entire speedslide potential, **steer more**.
- `speedslide_quality == 1`: you utilize entire speedslide potential. perfect speedslide.
- `speedslide_quality > 1`: you utilize entire speedslide potential, but you start losing some speed from drifting, **steer less**.

Hope you understand.