# 3d_interact

## Introduction
신입생 때 수행한 손동작 HCI 관련 프로젝트입니다.  
대학교에 들어와서 진행한 첫 프로젝트라 코드에서 미흡하지만, Depth Camera와 PointCloudLibrary, K-means clustering을 활용하여 구현에 성공했다는 점에서 의의를 뒀습니다.  
Depth Camera를 통해 얻은 손의 PointCloudData에 K-means clustering을 활용하여 양 손의 군집을 추출하고, 두 군집 사이의 거리 변화를 통해 마우스 스크롤 입력으로 변환하는 것으로 응용했습니다.  
  
본 프로젝트는 손동작을 마우스 혹은 키보드 입력으로 변환하는 프로젝트입니다.  
  
Transform hand movements into keyboard or mouse values and pass input to the computer.   

## Files
~~~
./src 소스 파일 저장 공간   
./include 헤더 파일 저장 공간   
./test_field 테스트 공간   
./test_results_log 테스트 결과 저장 공간(mp4, txt)   
~~~~

## Requirements
~~~
xdotool
PointCloudLibrary
~~~

## Test results
- Scrolling the mouse using hand movement   
![zoom](https://user-images.githubusercontent.com/49184890/124844992-b3f06580-dfd0-11eb-9ba3-8097fc26fcc4.gif)   

- Dragging the mouse using hand movement   
![pick_hold](https://user-images.githubusercontent.com/49184890/124845015-beaafa80-dfd0-11eb-8458-deb93bebbd30.gif)   

- Interworking with games   
![unity3d](https://user-images.githubusercontent.com/49184890/124845186-23feeb80-dfd1-11eb-9015-4d6ae649308c.gif)   
