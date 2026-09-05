# UBP 모션 쇼 — 재생목록, 얼굴 추적 인사, 모니터링 페이지

`tools/demo_show.py`는 panel 위에서 도는 상위 프로그램이다. 모션 재생목록을 무한 반복하고,
머리 카메라로 사람을 찾아 인사하고, e-stop이 걸리면 panel을 다시 띄워 살아남고, 자기
상태를 웹 페이지로 보여준다. 버스는 만지지 않는다 — 모든 setpoint가 panel의 한계·e-stop·
속도 상한을 지난다.

## 실행

```
# 비전 스택이 있는 venv로 (pyrealsense2, opencv, mediapipe)
C:\pj_depthcam_teleop\imitation_upper\.venv\Scripts\python.exe tools\demo_show.py ^
    --launch configs\ubp15.json --face

# 얼굴 없이, 시스템 Python으로
python tools\demo_show.py --launch configs\ubp15.json

# 로봇 없이 관절 범위만
python tools\demo_show.py --dry-run
```

모니터: http://127.0.0.1:8086 (`--monitor-addr off`로 끔). 얼굴 모델은
`tools/models/face_landmarker.task`(git 밖, `face_tracker.py --download-model`).

`--launch`가 없으면 8085에 이미 떠 있는 panel에 붙는다. 그 경우 복구는 `/api/reset`뿐이라
통신 유실(링크 teardown)에서는 못 돌아온다 — 아래 "복구" 참고.

## 재생목록

`--playlist eight*3,wave,rest` 기본. `eight`는 양팔이 각자 어깨 앞 forward-up 평면에
세로 8자(Gerono 리사주, 반폭 0.05 m·반높이 0.08 m, 중심 0.28/0.28, 12 s/회). 원과 같은
작업공간이고 팔꿈치 한계 여유 0.08 rad. `wave`는 한 팔을 앞으로 들어(sh_p -0.75, sh_r
-0.35) 앞팔을 접고(elb_p -1.4) 팔꿈치 yaw ±0.4 rad 0.5 Hz로 흔들며 그리퍼를 한 번 접는다.
등장할 때마다 좌우 교대. `rest`는 base 2 s.

act 사이는 전부 `/api/move` 프로파일 이동이고 act 안은 20 Hz `/api/pose` 스트림이다.
프로파일 이동의 도착은 `moving_to_rad`가 null이 되고 오차가 `--settle-tol` 안일 때로 본다
(`target_rad`가 프로파일을 따라가므로 위치 오차만으로는 이동 중에 바로 돌아온다 — 실측).

실기(2026-09-05): 8자 팔 오차 ≤ 0.12 rad, 손 흔들기 처짐 0.30~0.36 rad(중력, 위치제어
한계), 우측 전압 최저 10.8 V(MX 하한 10.0).

## 얼굴 추적과 인사 (`--face`)

`tools/face_tracker.py`가 D435i(640×480@30, depth를 color에 정렬)를 읽고 MediaPipe
FaceLandmarker(VIDEO, 최대 3명)로 얼굴마다 bbox·코 위치의 depth 거리·머리 자세(yaw/pitch,
transformation matrix)를 낸다. 검출 7 ms, 30 fps 유지.

- **트랙**: 최근접 중심 배정. 1 s 안 보이면 삭제. 같은 프레임의 겹치는 박스(IoU > 0.4)는
  하나로 본다(landmarker가 한 얼굴을 두 번 내는 일이 있었다).
- **시선**: |yaw| ≤ 20°, |pitch| ≤ 20°가 3 s 이어지면(0.5 s 끊김 허용) 인사 대상. 카메라가
  머리에 있고 머리가 얼굴을 따라가므로 "카메라 정면 = 로봇 정면".
- **후보**: 인사 안 한 살아있는 트랙 중 가장 가까운 얼굴. 머리(오버레이)가 매 setpoint마다
  후보 쪽으로 돈다(오프셋의 0.6배, 틱당 ≤ 0.04 rad). 후보가 없으면 좌우 ±0.5 rad 8 s
  삼각파로 탐색.
- **인사**: 8자·rest 중 후보가 3 s 응시 → act를 끊고 base로 프로파일 복귀 → 머리는 계속 그
  사람을 향한 채 가까운 팔(머리 yaw 방향)로 wave → 그 트랙을 greeted로 표시 → 끊긴 act를
  처음부터. greeted 트랙은 후보에서 빠지고, 화면에서 사라진 자리는 10 s 동안 기억해서 그
  근처에 새로 생긴 트랙(또는 greeted 트랙 옆의 새 박스)은 greeted를 물려받는다. 10 s 뒤에
  돌아오면 새 사람이다.

실기(2026-09-05 10:27, 로비): 얼굴 1.10 m에서 3.0 s 응시 → 오른팔 인사 → 8자 재개. 직후
두 번째 트랙(1.07 m)에 왼팔 인사 — 같은 사람의 중복 검출로 추정되어 위의 IoU 제거와
인접 상속을 넣었다(수정 후 미검증).

### 머리 관절: 이름과 실제가 다르다

`--calibrate-head`는 머리를 +0.15 rad 돌리고 전후 프레임의 phaseCorrelate로 화면이 어느
쪽으로 미끄러지는지 잰다. 이 로봇에서 두 번 잰 결과:

| 관절 | 화면 이동 | 실제 역할 |
|---|---|---|
| head_y (id 16) | dy +64 / +66 px | **위아래(tilt)** |
| head_p (id 15) | dx +55 / +61 px | **좌우(pan)** |

즉 `configs/ubp15.json`의 이름이 실제와 바뀌어 있다(설정은 안 고쳤다 — 학습된 한계와 base
포즈가 id에 묶여 있어서). 쇼는 `--head-yaw-joint head_p --head-pitch-joint head_y`를
기본값으로 쓰고, 부호는 둘 다 -1(양수 = 왼쪽 / 위). 카메라 자체는 정상 가로 방향(사진 확인).

## 복구

관절이 e-stop되면(통신 유실·전압·panel 버튼) `--launch` 모드에서는 panel을 끝내고 다시
띄운다(5 → 60 s 백오프). 통신 유실 뒤엔 세션이 링크를 닫고 재연결 API가 없기 때문이다
(`/api/reset`은 통과하지만 15축이 `blocked: Not connected`로 남는다, 확인함). 재기동 후
base로 프로파일 복귀 → 끊긴 act 재시작. 강제 `/api/estop`으로 9 s 복구 확인.

## 종료

Ctrl+C 또는 모니터의 Quit 버튼(`POST /api/quit`): base 복귀 → `/api/estop` → armed 0 확인
→ 1 s → panel 종료. `/api/stop`은 쓰지 않는다: panel이 auto-arm이라 다음 사이클에 다시
arm되어 토크가 안 풀린다(실측). e-stop은 latch되어 stop burst가 토크 off를 실어 나간다
(종료 후 dump에서 torque 0 확인). panel 자체는 Ctrl+C 핸들러가 없어 그냥 죽으므로 이
순서가 유일한 깨끗한 해제다. 강제로 죽였다면 `dxl_diag COM7 3000000 release <ids>`.

## 모니터 (`tools/show_monitor.py`)

stdlib만으로 8086에 페이지를 띄운다. 쇼와 별도 스레드가 200 ms마다 `/api/state`를 읽어
10분 링버퍼에 넣으므로 panel이 재기동되는 동안에도 페이지는 남고 그 구간은 빨갛게 칠해진다.

- 상단: panel 연결·loop Hz, 쇼 cycle/act/t·복구·인사 횟수, 추적 얼굴 수·후보·시선 타이머
- 15축 표: 위치·목표·오차·전압·주기·상태·사유(오차 > 0.2 rad·전압 < 11.0 V 주황, e-stop 빨강)
- 그래프: 관절별 전압, 위치 오차(최근 10분)
- 이벤트: e-stop 사유, 복구 시도/성공, 인사, act 전환
- 카메라: 오버레이(박스·거리·yaw/pitch·응시 바) JPEG 5 Hz
- Quit 버튼

JSON: `/api/snapshot`, `/api/history?seconds=600`, `/camera.jpg`. 페이지 렌더링은 브라우저
확장이 안 붙어 이 세션에서 눈으로 확인하지 못했다(엔드포인트는 확인함).
