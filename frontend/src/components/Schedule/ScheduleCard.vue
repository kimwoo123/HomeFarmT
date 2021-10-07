<template>
  <div class="card-box shadows">

    <div class="content-box">
      <div class="content">
        <span class="title">{{ title }}</span>
        <span class="description font-300">{{ description }}</span>
      </div>
      <SwitchBtn class="switch-align"/>
    </div>

    <div class="time-box">
      <img src="@/assets/icons/calender.svg" alt="date" class="icon">
      <span class="date">{{ date }}</span>
      <img src="@/assets/icons/time.svg" alt="time" class="icon">
      <span class="time">{{ time }}</span>
    </div>

  </div>
</template>

<script>
import SwitchBtn from '@/components/common/SwitchBtn.vue'

export default {
  name: 'ScheduleCard',
  components: {
    SwitchBtn
  },
  data() {
    return {
      title: '감시하기',
      description: '1번 경로로 감시를 실행합니다',
      time: '오전 9:00',
      date: '2021년 10월 7일',
      status: true
    }
  }
}
</script>

<style lang="scss" scoped>
  @import '@/components/Schedule/ScheduleCard.scss';
</style>
