# 과제 — Both Lane Path Planning (양쪽 차선 평균)

## 목표
좌·우 차선이 모두 valid 한 sinusoidal 도로에서, 두 차선의 polynomial 계수로부터 **중앙선 path** 의 계수를 생성한다.
ego 차량은 chapter 3 의 `PurePursuit` 컨트롤러로 이 path 를 따라 주행.

## 인터페이스 계약
**이 시그니처는 변경하지 마세요.** 채점/테스트가 이 형태에 의존합니다.

```python
def both_lane_to_path(coeff_L: np.ndarray, coeff_R: np.ndarray) -> np.ndarray
    # coeff_L, coeff_R: shape (degree+1, 1) column, 계수 순서 고차→저차
    # 반환: 같은 shape
```

`coeff_L` / `coeff_R` 은 chapter 3 의 `PolynomialFitting.fit()` 출력으로, local frame 의 좌/우 차선 다항식 계수.

## 구현 위치
`01_Python_project_refactored/release/04_path_planning/01_both_lane/both_lane_planner.py` 의 함수 본문 `# TODO:` 블록.

## 실행

> 환경 셋업은 [`../../README.md`](../../README.md) 참조. **git root 에서 실행.**

테스트 (합격 검증):
```bash
uv run pytest 01_Python_project_refactored/release/04_path_planning/01_both_lane/ -v
```

시나리오 실행 → `record.json` 생성 + Rerun viewer 자동 띄움:
```bash
uv run python 01_Python_project_refactored/release/04_path_planning/01_both_lane/record_gen.py
```
→ 파란 차량이 sinusoidal 도로의 차로 중앙선을 따라 주행. 양쪽 흰색 차선 + dashed 중앙선 + 매 step 의 planner path (노란 곡선) 가 ego 앞 5 m 까지 뻗음.

> JSON 만 만들고 viewer 안 띄우려면 record_gen 명령에 `--no-viewer` 옵션 추가.

Rerun viewer 로 재생:
```bash
uv run python 01_Python_project_refactored/release/04_path_planning/simulator_path_planning.py 01_Python_project_refactored/release/04_path_planning/01_both_lane/
```

> **시뮬레이터는 챕터 전체용** — 인자 없이 실행하면 `04_path_planning/` 하위 모든 시나리오를 한 viewer 에 별도 recording 으로 멀티 로드, viewer 좌측 Recordings 패널에서 클릭 전환. `--camera follow|fixed` 로 초기 카메라 (기본 `follow`).

## 합격 기준 (`pytest` 통과)
알고리즘 형태 (정통 좌·우 평균 / 다른 결합 방식) 는 제약 X — **behavioral spec** 만 본다.

1. **인터페이스 sanity** — `both_lane_to_path` 반환 shape 이 입력과 동일 column
2. **폐루프 추적 오차** — sinusoidal both-lane 도로 30 s 추종, tail 평균 `|lateral err| < 0.3 m`, peak `< 1.0 m`

> trivial 구현 (coeff_path = 0) 은 도로 곡선 못 따라가 임계값 초과로 차단.

## 힌트
- 일반 형태: 같은 차수의 계수끼리 평균 — `(coeff_L + coeff_R) / 2`
- numpy 배열 산술은 element-wise 라 직관적으로 작성 가능

## 게인/파라미터 튜닝 위치

라이브러리 코드 (`.py` 안의 클래스·함수) 는 **시그니처만** 정의. 실제 *값* 은 두 곳에서 명시:

- **시각화/실행 (자유롭게 변경 OK, **release 기본값은 모두 0**)**: 같은 폴더의 `record_gen.py` 안의 게인/파라미터가 0 으로 초기화 → **학생이 직접 채워야 응답이 나옴**. 값을 바꿔 다시 실행하며 응답 변화 비교.
- **합격 기준 검증 (변경 금지)**: `test_*.py` 안에 박혀 있음. pytest 가 이 값으로 통과 여부를 본다.

## 문제별 추가 제약
- **`lane_both.py`, `vehicle_lat_both.py`, `record_gen.py` 수정 금지** — 검증 환경(fixture).
- chapter 3 의 `frame_transform`, `pure_pursuit` 는 sys.path 로 import 만 — 수정 X.
