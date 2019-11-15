# 길찾기 벨트
이 앱은 시각장애인을 위한 길 안내 기능을 하는 벨트입니다.
시각을 대신하여 진동으로 직관적인 길 안내를 하는 네비게이션입니다.

### 개발 계기
 다음과 같은 계기로 개발 된 어플리케이션 입니다.</br>
 </br>
  " 시각장애인이 가봤던 장소가 아닌 새로운 장소를 찾아서 가게 된다면 어떻게 찾아 갈 수 있을까? " </br>
  
### 아이디어
  1. 시각장애인을 위한 네비게이션이 있으면 좋을 것 같다.</br>
  2. 비 장애인용 네비게이션은 직관을 시각으로 표현하였지만, 시각 장애인은 촉감을 이용하여 직관적인 길안내를 하자.</br>

### 구현 방법
  1. 라즈베리파이를 활용하여 GPS에서 위치정보를 가져온다.</br>
  2. 10축 (3축 자이로+3축 가속도+3축 자기장+압력) 센서모듈을 활용하여 벨트를 찬 상태에서 바라보고 있는 방향을 계산한다.(상보필터 사용)</br>
  3. 맵에서 가야하는 다음 경로의 위치를 계산하고 방향을 삼각함수를 활용하여 계산한다.
  4. 네비게이션 경로에서 바라보고 있는 방향이 맞지 않다면 진동모터를 활용하여 방향을 지시한다</br>
  5. 오른쪽 모터가 작동 : 몸을 오른쪽으로 회전 / 왼쪽 모터 작동 : 몸을 왼쪽으로 회전 / 모든 모터 작동 : 방향 교정 완료</br>
  
### 사용 재료
  라즈베리파이, 10축 (3축 자이로+3축 가속도+3축 자기장+압력) 센서모듈 GY-87, GPS리시버 l80-m39, 모터(시각적 테스트를 위해 LED도 사용)
  

### 전체 구성
![meterial](https://github.com/chuuuul/Wayfinding-Belt/blob/master/gitData/material.png)

### 플로우차트
![flowChart](https://github.com/chuuuul/Wayfinding-Belt/blob/master/gitData/flowChart.png)

### 회로도
![circuitDiagram](https://github.com/chuuuul/Wayfinding-Belt/blob/master/gitData/circuit_diagram.png)

### 환경, 라이브러리, 사용 지식
C, Python2, 라즈베리파이, 라즈비안, 상보필터, I2C통신(wiringPiI2C, wiringPi), 시리얼통신, GPS, 삼각함수
  

### 동작 방법
1. 라즈베리파이에서 gcc -o find_map find_map.c -lm -lwiringPi -lpthread </br>
2. gps.py 실행</br>
3. find_map 실행</br>
  
  
  
  
