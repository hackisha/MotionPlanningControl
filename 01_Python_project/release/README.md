# Release — 학생 과제 배포본

이 폴더는 IVS 강의의 **학생 과제 배포본** 입니다. 각 하위 폴더 (`01_filters/...`, `02_pid/...`) 가 자기충족적 과제 1개.

> **사용자 흐름:** 환경을 한 번 셋업 → 각 과제 폴더의 README 보고 풀이 → 같은 폴더의 테스트 통과시키기.

---

## 환경 셋업 (1회)

> **실행 위치 (중요):** 이 저장소의 **git root** (`.git/` 폴더가 있는 최상위) 에서 모든 명령을 실행합니다.
> 어느 폴더가 git root 인지 모르겠다면 `git rev-parse --show-toplevel` 으로 확인.

요구사항: **[uv](https://docs.astral.sh/uv/) 만 있으면 됩니다.** Python 3.14 는 uv 가 자동으로 다운로드/관리합니다 — 시스템에 Python 미설치여도 OK.

uv 설치 (1회):
- **Windows:** `winget install --id=astral-sh.uv` (or PowerShell: `irm https://astral.sh/uv/install.ps1 | iex`)
- **macOS / Linux:** `curl -LsSf https://astral.sh/uv/install.sh | sh`

설치 후 git root 에서:

```bash
uv sync
```

이 한 줄이 자동으로:
- `.python-version` 에 명시된 Python 3.14 다운로드 (없으면)
- `.venv/` 가상환경 생성
- `uv.lock` 기반 정확한 버전의 라이브러리 설치

`uv.lock` 은 강사가 `pyproject.toml` 로부터 생성한 **고정 버전 스냅샷** — 모든 학생이 동일한 라이브러리 버전을 사용하게 보장. 직접 편집 금지.

### 셋업 트러블슈팅

| 증상 | 대응 |
| --- | --- |
| `uv: command not found` | uv 미설치 — 위 "uv 설치 (1회)" 단계 다시 |
| `uv` 실행이 즉시 차단됨 / `WinError 4551: 애플리케이션 제어 정책...` | Windows 11 **Smart App Control (SAC)** 이 `uv.exe` 차단 중. **SAC 끄기:** `설정` → `개인정보 및 보안` → `Windows 보안` → `앱 및 브라우저 컨트롤` → `Smart App Control 설정` → **끄기** → 재기동. 강의용 임시 PC 는 반납 시 초기화되니 부담 없음. (개인 PC 면 한 번 끄면 Windows 클린 설치 전 다시 못 켜니 신중히.) |
| `ImportError: DLL load failed ... 애플리케이션 제어 정책에서 차단` | 같은 SAC 원인 — 위와 동일 대응 |
| `requires a different Python: 3.X.x not in '==3.14.*'` | `uv` 가 다른 Python 으로 venv 생성 — `rm -rf .venv && uv sync` 로 재시도 (uv 가 자동으로 3.14 다운로드) |

---

## 과제 실행 / 테스트 패턴

각 과제 README의 "실행" 섹션에 정확한 path가 있습니다. 일반 형태:

**테스트 (합격 검증)** — 합격 기준은 이 명령으로:
```bash
uv run pytest 01_Python_project/release/<area>/<problem>/ -v
```
테스트가 전부 통과 (`PASS`) 하면 합격.

**데모 (시각 확인, 선택)** — 테스트 통과 후 "내가 짠 필터/제어기가 실제로 어떻게 움직이는지" 눈으로 확인하고 싶을 때:
```bash
uv run python 01_Python_project/release/<area>/<problem>/demo.py
```
→ 기본 브라우저 새 탭에 plotly 인터랙티브 그래프 (확대·팬·호버 가능). `# TODO` 구현 전에 실행하면 `NotImplementedError`.

**비교 데모 (선택, 01_filters/01-04 모두 완료 후)**:
```bash
uv run python 01_Python_project/release/01_filters/comparison_demo.py
```
→ Average / Moving Average / Low-Pass / Kalman 4 개 필터를 같은 신호에 동시 적용한 결과 한 그래프 비교.

### 명령 실행 규칙 (반드시 따를 것)

`activate` / `deactivate` 쓰지 말고 항상 **`uv run <tool>`** 형태로 호출합니다.

- `uv run` 은 실행 전에 venv 자동 동기화까지 해주므로 "패키지 깔았는데 안 보임" 류 디버깅 0
- 셸 (bash / PowerShell / cmd / zsh) 무관, 한 형태로 통일
- 활성화 망각 시 시스템 Python 폴백 같은 함정 없음

---

## AI 도구 사용 가이드 (공통)

어떤 도구든 OK — Claude / Cursor / ChatGPT / Gemini / Copilot 등.

### 지킬 것
- 각 과제 README의 **인터페이스 계약 (시그니처)** 은 그대로 유지
- 코드 파일 안의 `# TODO:` 위 안내문은 그대로 유지 (지우지 말 것)
- 과제별 추가 제약은 각 README 의 마지막 섹션 참조 (예: 특정 보조 파일 수정 금지)

### 권장 질문 패턴 (정답 직답 회피)
1. **개념 질문** — "이 알고리즘의 일반 식이 뭐야?"
2. **변환 질문** — "이 식을 Python 메소드로 옮기면 어떻게 돼?"
3. **검증 질문** — "내가 짠 코드 보여줄게 — 의도와 일치해?"

### 피할 것
- "정답 코드 줘" 식 직접 답 요청 → 학습 효과 ↓
- AI가 준 코드 그대로 복붙 → 한 번 손으로 타이핑하면서 의미 파악
