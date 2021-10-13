## IoT제어 프로젝트

* 시뮬레이터 및 프로젝트 관련 파일 다운로드
  - https://drive.google.com/drive/folders/1rp54qL31ZIoHet7A9BlvpoDCCdGVsvLK?usp=sharing

(위 경로에 위치한 프로그램 및 문서는 SSAFY 과정 내에서만 사용할 수 있으며 무단 복제 및 반출, 배포를 금합니다.)

# 홈팜티 (HomeFarmT)

> 농업인을 위한 작은 스마트팜

농촌 고령화, 일손 부족 문제의 해결 방안으로 스마트팜이 제안됐습니다. 하지만, 스마트팜의 초기 투자비용, 제어기기 조작의 어려움, 우리나라 현실에 맞지 않는 빅데이터 등을 이유로 도입이 망설여지고 있습니다. 홈팜티는 이러한 문제를 해결할 수 있는, 부담없이 사용할 수 있는 작은 스마트팜입니다.

 - 서비스 명 : HomeFarmT (홈팜티)
 - 개발 기간 : 2021.08.30 ~ 2021.10.08 (6주)
 - 팀명 : 으라차차

|  이름  | 역할 |         담당 업무         | 깃허브 |
| :----: | :--: | :-----------------------: | :----: |
| 신동윤 | 팀장 | 자율주행, Frontend, 배포  |        |
| 김담영 | 팀원 |      자율주행, Unity      |        |
| 김우석 | 팀원 |  물체인식, Backend, 배포  |        |
| 이다은 | 팀원 | 자율주행, Frontend, UI/UX |        |
| 이두호 | 팀원 |      물체인식, MLOps      |        |



# 주요 기능
 - 기기 제어
    - 연결된 IoT 기기를 켜거나 끄기
 - 감시
    - 터틀봇에 달린 카메라를 통해 캠 화면을 보여줌
    - 현재 터틀봇의 위치를 미니맵에 표시함. 미니맵을 눌러 두 좌표를 특정하면 두 좌표 사이를 이동하며 감시
    - 터틀봇이 침입자를 발견할 시, 최근 감시 내역에 침입자 사진과 날짜, 시간이 저장됨
    - 감시하기 위해 임의의 경로를 최대 3개 기록 가능하며, on/off 가능함 
 - 일정
    - 일별로 일정(감시하기 or IoT 기기 제어)을 시간을 특정하여 생성할 수 있음. 
    - 메인 페이지의 오늘의 일정에 오늘의 일정을 출력
    - 특정된 시간에 해당 일정을 실행 (미구현)
 - 터틀봇 조작
    - 터틀봇에 달린 카메라를 통해 캠 화면을 보여줌
    - 현재 터틀봇의 위치를 미니맵에 표시함. 미니맵을 눌러 좌표를 특정하면 해당 좌표로 이동
    - 화살표 버튼을 클릭해 터틀봇을 제어
    - 물건에 가까이 이동한 후, 물건 들기/놓기 실행 가능
 - 기타
    - 사용자 지역의 날씨, 온도, 습도, 풍속 출력
    - 이동하다가 잡초를 발견하면 제거하여 작물 재배에 도움
    - STT 기능 (미구현)

# 구현

- 맵 생성

![침입자 감시](https://alluring-mall-81e.notion.site/image/https%3A%2F%2Fs3-us-west-2.amazonaws.com%2Fsecure.notion-static.com%2F9b00edd8-2a9d-4edf-b7a3-350fdeff0939%2F%EB%A7%B5_%EC%83%9D%EC%84%B1.gif?table=block&id=62457345-3e62-479e-a1c6-f9065158b5b9&spaceId=51eb7c59-8acd-44f0-8b14-3f8fe7b8b19c&userId=&cache=v2)

- 물체 회피

![IoT 기기 제어](https://alluring-mall-81e.notion.site/image/https%3A%2F%2Fs3-us-west-2.amazonaws.com%2Fsecure.notion-static.com%2Fcfaed099-d454-42a4-bc55-e772650c6158%2F%EB%AC%BC%EC%B2%B4_%ED%9A%8C%ED%94%BC.gif?table=block&id=4bb2776c-ce66-4655-84bf-cac81bf26e49&spaceId=51eb7c59-8acd-44f0-8b14-3f8fe7b8b19c&userId=&cache=v2)

- 침입자 감시

![터틀봇 조작](https://alluring-mall-81e.notion.site/image/https%3A%2F%2Fs3-us-west-2.amazonaws.com%2Fsecure.notion-static.com%2Faafcb8e6-e2cb-4148-871a-85fd153bb268%2F%EC%B9%A8%EC%9E%85%EC%9E%90_%EB%B0%9C%EA%B2%AC.gif?table=block&id=a8532547-8ecf-4c2b-b40c-b8be9636c3ae&spaceId=51eb7c59-8acd-44f0-8b14-3f8fe7b8b19c&userId=&cache=v2)

- 스케줄 관리

![스케줄 관리](https://alluring-mall-81e.notion.site/image/https%3A%2F%2Fs3-us-west-2.amazonaws.com%2Fsecure.notion-static.com%2F4d49948e-f182-4532-9e0c-de1c2e7bd18d%2FIoT_%EC%A0%9C%EC%96%B4.gif?table=block&id=cab36c62-23a3-4b25-b483-f268392a44a1&spaceId=51eb7c59-8acd-44f0-8b14-3f8fe7b8b19c&userId=&cache=v2)

- 작물 재배 도움

![작물 재배 도움](https://alluring-mall-81e.notion.site/image/https%3A%2F%2Fs3-us-west-2.amazonaws.com%2Fsecure.notion-static.com%2Fba912b6a-3dc0-463b-846c-056e12ba9c2d%2F%EC%9E%91%EB%AC%BC_%EC%9E%AC%EB%B0%B0.gif?table=block&id=319af1ad-1b6f-4192-90a7-259b3dfae2da&spaceId=51eb7c59-8acd-44f0-8b14-3f8fe7b8b19c&userId=&cache=v2)





# 기술 스택

 - 
 - 

# 산출물
