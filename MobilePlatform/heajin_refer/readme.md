cpp_sensor_talker - cpp를 통한 아두이노와의 serial 통신
mini45 - mini45와의 serial 통신, initialization, talker.py 코드 사용
msgpkg - 모든 msg 모아둔 패키지
sensor_talker - python을 통한 아두이노와의 serial 통신
ethercat_test - maxon motor 실행코드, xenomai 필수, 실행 코드 : ethercat_control.cpp
 	> ether_cpp 내에 여러 code 존재 ethercat_control.cpp만 바꿔주면 사용가능
		> final_code - 현재 적용된 알고리즘 ( adaptive backlash inverse)
		> HJ_ADAPTIVE_RLS - LMS 수렴아닌 RLS 수렴으로 한 코드
		> motor - motor 위치 제어기
		> pid - output쪽 deflection의 pid 위치 제어기
