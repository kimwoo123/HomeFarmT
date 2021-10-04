<template>
  <div class="home-schedule shadows">
    <div>
      <span>오늘의 일정</span>
      <span>icon</span>
    </div>
    <div class="schedule-box">
      <font-awesome-icon icon="chevron-left" />
      <div class="schedule-content">
        감시하기
      </div>
      <font-awesome-icon icon="chevron-right"/>
    </div>
  </div>
</template>

<script>
export default {
  name: 'HomeSchedule',
}
</script>

<style lang="scss" scoped>
  @import '@/components/Home/HomeSchedule.scss';
</style>