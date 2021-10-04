<template>
  <div class="card-box shadows">

    <div class="content-box">
      <div class="content">
        <span class="title">{{ title }}</span>
        <span class="description">{{ description }}</span>
      </div>
      <img src="@/assets/icon/switch.svg" alt="">
    </div>

    <div class="time-box">
      <img src="@/assets/icon/time.svg" alt="time" class="icon">
      <span class="time">{{ time }}</span>
      <img src="@/assets/icon/calender.svg" alt="date" class="icon">
      <span class="date">{{ date }}</span>
    </div>

  </div>
</template>

<script>
export default {
  name: 'ScheduleCard',
  data() {
    return {
      title: '감시하기',
      description: '1번 경로로 감시를 실행합니다',
      time: '오전 9:00',
      date: [
        '월', '화', '수',
      ],
      status: true
    }
  }
}
</script>

<style lang="scss" scoped>
  @import '@/components/Schedule/ScheduleCard.scss';
</style>
