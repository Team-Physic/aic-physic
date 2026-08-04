# Git 협업 규칙

이 문서는 프로젝트의 커밋 메시지와 브랜치 이름 작성 규칙을 정의한다.

## 커밋 메시지

커밋 메시지는 다음 형식을 사용한다.

```text
<type>: <변경 내용>
```

사용 가능한 `type`은 다음과 같다.

| 타입 | 용도 |
| --- | --- |
| `feat` | 새로운 기능 추가 |
| `fix` | 버그 수정 |
| `docs` | 문서 추가 또는 수정 |
| `style` | 동작에 영향을 주지 않는 코드 스타일 수정 |
| `refactor` | 기능 변경 없이 코드 구조 개선 |
| `test` | 테스트 코드 추가 또는 수정 |
| `chore` | 빌드 설정, 의존성 관리 등 기타 작업 |

작성할 때 다음 원칙을 지킨다.

- 변경 내용을 짧고 명확하게 설명한다.
- 변경 내용은 영어로 작성한다.
- 하나의 커밋에는 하나의 논리적 작업만 담는다.
- `update code`, `fix bug`처럼 변경 내용을 알 수 없는 표현은 사용하지 않는다.

예시:

```text
feat: add object detection visualization
fix: handle camera disconnection
docs: add development environment guide
refactor: separate robot command generation
```

## 브랜치 이름

브랜치 이름은 다음 형식을 사용한다.

```text
<type>/<short-description>
```

| 타입 | 용도 | 예시 |
| --- | --- | --- |
| `feature` | 새로운 기능 개발 | `feature/object-detection` |
| `bugfix` | 일반적인 버그 수정 | `bugfix/camera-timeout` |
| `hotfix` | 배포된 버전의 긴급 수정 | `hotfix/security-patch` |
| `release` | 릴리스 준비 | `release/v1.2.0` |
| `experiment` | 실험적인 기능이나 접근 검증 | `experiment/new-policy` |
| `wip` | 개인 작업 또는 진행 중인 실험 | `wip/refactor-controller` |

브랜치 이름에는 다음 원칙을 적용한다.

- 영문 소문자만 사용한다.
- 단어 구분에는 하이픈(`-`)을 사용한다.
- 타입과 설명은 슬래시(`/`)로 구분한다.
- 작업 목적을 알 수 있도록 짧고 구체적으로 작성한다.
- 기본 브랜치는 `main`, 개발 통합 브랜치를 운영하는 경우에는 `develop`을 사용한다.

예시:

```bash
git switch -c feature/user-authentication
git switch -c bugfix/fix-login-error
git switch -c hotfix/security-patch
git switch -c release/v1.2.0
```

## 참고 자료

- [Commit 메시지 규칙](https://wikidocs.net/332862)
- [Git branch 네이밍 규칙](https://itlab.tistory.com/153)
