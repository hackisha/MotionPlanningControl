# 과제 — Low-Pass Filter (1차 IIR)

## 목표
이전 출력과 현재 입력의 가중 평균으로 신호를 평활화하는 1차 저역 필터 (EMA, exponential moving average) 를 구현한다.
하나의 파라미터 α 로 평활 강도를 조절한다.

## 인터페이스 계약
**이 시그니처는 변경하지 마세요.** 채점/테스트가 이 형태에 의존합니다.

```python
class LowPassFilter:
    def __init__(self, alpha: float = 0.9): ...
    def step(self, x: float) -> float
```

- `step(x)` — 새 측정 `x` 를 받아 평활된 출력 반환
- `alpha` ∈ [0, 1] — 0 은 즉시 통과(no filter), 1 은 첫 값 고정(완전 평탄)
- 첫 호출에서는 이전 출력이 없으므로 `x` 자체를 그대로 반환 (초기값 보호)

## 구현 위치
`01_Python_project_refactored/release/01_filters/03_low_pass_filter/low_pass_filter.py` 의 `step` 메소드 안 `# TODO:` 블록.

## 실행

> 환경 셋업 (1회) 과 명령 실행 규칙은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트 (합격 검증):
```bash
uv run pytest 01_Python_project/release/01_filters/03_low_pass_filter/ -v
```

데모 (시각 확인, 선택 — 구현 후):
```bash
uv run python 01_Python_project/release/01_filters/03_low_pass_filter/demo.py
```
→ sin(t) + N(0, 0.5) 노이즈를 α=0.9 로 평활한 결과 plotly 그래프.

## 합격 기준 (`pytest` 통과)
학생이 푼 알고리즘 형태 (EMA / cumulative / moving avg 등) 는 제약 X — **behavioral spec** 만 본다.

1. **상수 입력 안정성** — `step(2.0)` 200 회 반복 시 출력 `2.0` 유지
2. **노이즈 추적 RMS** — `alpha=0.9`, 상수 truth=5 + N(0, 1) 노이즈 1만 표본, warm-up 이후 RMS 오차 `< 0.3`

> RMS = √(bias² + variance). `return 0` (bias 임계값 초과) / `return x` (variance 임계값 초과) 모두 차단.

## 힌트
- 일반 식: `y = α · y_prev + (1 - α) · x`
- α 와 (1-α) 위치 주의 — 부호/순서 바꾸면 측정 무게가 잘못 적용됨
- `self.y` 가 `None` 인 첫 호출 분기 필요 (`if self.y is None:`)
- α 직관: 크면 부드럽고 느림, 작으면 거칠고 빠름

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의 — kp/kd/ki, window_size, R/Q, lookahead 등은 매개변수로만 받는다. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` / `demo.py` (시나리오 여럿이면 `record_gen_<scenario>.py`) 안의 게인/파라미터가 0 으로 초기화되어 있음 → **학생이 직접 채워야 응답이 나옴**. 0 인 채로 실행하면 controller 출력 0, 응답 없음 (또는 NaN/division 에러). 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다 — 임의로 바꾸면 검증 의미가 사라짐.

즉 "다른 게인은 어떻게 동작하지?" 는 producer 만 바꾸고, "내 구현이 spec 을 통과하는가?" 는 test 그대로 두고 `pytest` 만 돌리면 된다.

## 문제별 추가 제약
(공통 제약은 [`../../README.md`](../../README.md) 의 "AI 도구 사용 가이드" 참조)

이 문제에 한정한 추가 사항은 없습니다.
