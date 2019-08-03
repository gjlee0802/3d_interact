# 3d_interactive_proj
Smart System Competition Proj

7월 28일 GITHUB 시작

8월 목표: 제스쳐 인식 완성, 프레임 완성

<디렉토리 정보>
./src 소스 파일 저장 공간
./include 헤더 파일 저장 공간
./test_field 테스트 공간
./test_results_log 테스트 결과 저장 공간(mp4, txt)
./backups 프로젝트 백업 저장 공간


<사전 설치>
Point Cloud Library
Xdotool

<주의 사항>
[1]. 	터치 영역이 변형될 경우, ./include/etc.hpp의 Struct Screen_data 안에 있는 값들을 실측값으로 변경해야한다.
	#define Estimate_MIN_MAX의 주석을 해제하여 활성화하고 측정하여 MIN MAX 실측값을 얻을 수 있다.

[2]. 
