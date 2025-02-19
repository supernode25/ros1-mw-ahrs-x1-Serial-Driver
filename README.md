1. MW-AHRS-x1 IMU 센서  ROS1 드라이버 개발
2. Serial 통신기반 ros loop rate 1000hz이상까지 퍼블리쉬 테스트 완료 (멀티스레딩 최적화)
3. 115200, 921600 baudrate 정상 작동 완료




# IMU Quaternion 설명

IMU에서 출력하는 `imu.orientation.w, x, y, z` 값은 **Z-Y-X (yaw-pitch-roll) 순서의 회전을 반영한 quaternion 값**입니다.  
아래 표는 각 요소의 의미를 정리한 것입니다.

| 요소 | 의미 | 관련된 회전축 | 설명 |
|------|------|--------------|------|
| `imu.orientation.w` | 스칼라 값 | - | 전체 회전 변환의 정도를 나타냄. 1에 가까울수록 회전이 적음. |
| `imu.orientation.x` | X축 회전 성분 | X축 (1, 0, 0) | Roll (구르기, 좌우 기울기)에 대한 기여도. |
| `imu.orientation.y` | Y축 회전 성분 | Y축 (0, 1, 0) | Pitch (앞뒤 기울기)에 대한 기여도. |
| `imu.orientation.z` | Z축 회전 성분 | Z축 (0, 0, 1) | Yaw (좌우 방향 회전)에 대한 기여도. |

## Quaternion 계산 공식

다음 수식은 Z-Y-X 회전 순서에 따라 오일러 각을 quaternion으로 변환하는 공식입니다.

\[
w = \cos(z) \cos(y) \cos(x) + \sin(z) \sin(y) \sin(x)
\]

\[
x = \cos(z) \cos(y) \sin(x) - \sin(z) \sin(y) \cos(x)
\]

\[
y = \cos(z) \sin(y) \cos(x) + \sin(z) \cos(y) \sin(x)
\]

\[
z = \sin(z) \cos(y) \cos(x) - \cos(z) \sin(y) \sin(x)
\]

여기서:
- `x = ang_x (roll)` : X축을 기준으로 회전
- `y = ang_y (pitch)` : Y축을 기준으로 회전
- `z = ang_z (yaw)` : Z축을 기준으로 회전

## SW에서 2바퀴 회전하는 원인 및 해결 방법

| 원인 | 설명 | 해결 방법 |
|------|------|----------|
| 오일러 각 범위 초과 | ±180°를 넘어가면 값이 갑자기 반대 방향으로 튀는 문제 발생 | 각도를 ±180° 범위로 제한 |
| Quaternion 변환 오류 | 오일러 각 → quaternion 변환 과정에서 ±180°를 넘어갈 때 불연속적 변환이 발생 | quaternion 정규화 적용 |
| 센서 wrap-around 문제 | IMU 센서가 360°를 넘으면 0°로 초기화되면서 갑자기 반대 방향으로 바뀜 | 오일러 각 변환 시 연속성을 유지하는 보정 적용 |

### ✅ **오일러 각 범위 제한 코드**
```cpp
if (ang_x > 180.0) ang_x -= 360.0;
if (ang_x < -180.0) ang_x += 360.0;
if (ang_y > 180.0) ang_y -= 360.0;
if (ang_y < -180.0) ang_y += 360.0;
if (ang_z > 180.0) ang_z -= 360.0;
if (ang_z < -180.0) ang_z += 360.0;
