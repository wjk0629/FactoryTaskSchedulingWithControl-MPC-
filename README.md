# FactoryTaskSchedulingWithControl-MPC-
작은 공장환경에서 모바일로봇이 이송 임무를 수행하는 환경을 묘사하고자 하였다. 
동시에 모바일로봇의 컨트롤도 고려할 수 있게 하려 했는데, 사용된 컨트롤은 PID와 MPC이다. 
다만 미완성이라 잘 안된다...언젠가 완성하겠다. 
MATLAB R2019a를 사용하였으며, 기본적인 시나리오는 생성된 모바일로봇이 랜덤한 위치에 생성된 Spot에서 Call한다면 그곳으로 이동했다가 빨간점으로 이송을 반복한다. 

This MATLAB SW is intended to describe an environment in which mobile robots perform transport missions in a small factory environment. At the same time, it was intended to allow the control of the mobile robot to be considered, and the controls used were PID and MPC. 
But it's not working well because it's incomplete.
I'll complete it someday...
MATLABR2019a was used, 
and the basic scenario is that if the generated spot created at a random location calls mobile robot, mobile robot moves there and go to the red dot. 
mobile robot repeats the transfer until call finished 
