# 과제 — Moving Average Filter

## 목표
들어오는 측정값의 **최근 N 개** 만 평균해 신호를 평활화하는 슬라이딩 윈도우 필터를 구현한다.
앞선 Average Filter (전체 누적 평균) 와 달리 오래된 값은 잊는다.

## 인터페이스 계약
**이 시그니처는 변경하지 마세요.** 채점/테스트가 이 형태에 의존합니다.

```python
class MovingAverageFilter:
    def __init__(self, window: int = 10): ...
    def step(self, x: float) -> float
```

- `step(x)` — 새 측정 `x` 를 추가하고 현재까지 보관 중인 표본들의 평균을 반환
- `window` — 평균에 사용할 최근 표본의 개수
- `__init__` 은 이미 작성되어 있음 — `self.buffer` (deque, maxlen=window) 가 준비됨

## 구현 위치
`01_Python_project_refactored/release/01_filters/02_moving_average_filter/moving_average_filter.py` 의 `step` 메소드 안 `# TODO:` 블록.

## 실행

> 환경 셋업 (1회) 과 명령 실행 규칙은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트 (합격 검증):
```bash
uv run pytest 01_Python_project/release/01_filters/02_moving_average_filter/ -v
```

데모 (시각 확인, 선택 — 구현 후):
```bash
uv run python 01_Python_project/release/01_filters/02_moving_average_filter/demo.py
```
→ N(5, 1) 노이즈 200 표본을 window=15 로 평활한 결과 plotly 그래프.

## 합격 기준 (`pytest` 통과)
학생이 푼 알고리즘 형태 (deque sliding / cumulative subtract / 다른 방식) 는 제약 X — **behavioral spec** 만 본다.

1. **상수 입력 안정성** — `step(2.0)` 50 회 반복 시 출력 `2.0` 유지
2. **노이즈 추적 RMS** — `window=20`, 상수 truth=5 + N(0, 1) 노이즈 1만 표본, warm-up 이후 RMS 오차 `< 0.4`

> RMS = √(bias² + variance). `return 0` / `return x` 등 trivial 구현은 두 임계값 모두 초과로 차단.

## 힌트
- `self.buffer.append(x)` 로 새 값 추가 (deque maxlen 이 자동으로 오래된 값 밀어냄)
- 단순 합산: `sum(self.buffer) / len(self.buffer)`
- 효율적 형태 (선택): `y_n = y_{n-1} + (x_n - x_{n-window}) / window` — buffer 가득 찬 후만 사용 가능

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
(공통 제약은 [`../../README.md`](../../README.md) 의 "AI 도구 사용 가이드" 참조 — 시그니처 유지, `# TODO:` 위 안내문 유지 등)

이 문제에 한정한 추가 사항은 없습니다.
