<template>
  <div>
    <div class="schedule-header">
      <span class="schedule-text">Schedule</span>
      <button><font-awesome-icon icon="plus" /><span> 추가</span></button>
    </div>

    <div class="schedule">
      <div v-for="time in 25" :key=time>
        <div v-if="time <= 10" class="time-box">
          0{{ time-1 }}:00
        </div>
        <div v-else class="time-box">
          {{ time-1 }}:00
        </div>
      </div>
      <div class="item">

      </div>
    </div>
  </div>
</template>

<style lang="scss" scoped>
  @import './SchedulePlan.scss';
</style>

<script>
export default {
  name: 'SchedulePlan',
    data() {
    return {
      today: {
        year: 2021,
        month: 9,
        day:14
      },
    }
  }

}
</script>