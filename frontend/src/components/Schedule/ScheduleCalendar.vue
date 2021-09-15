<template>
  <div class="calendar">
      <div class="year-container">
        <small class="year year-active">2021</small>
        <small class="year">2022</small>
      </div>
      <div class="month-container">
        <div v-for="month_cal in 5" :key=month_cal>
          <div class="month" :class="{'month-active' : month_cal == 1}">
            9월
          </div>
        </div>
      </div>
      <div class="day-container">
        <div v-for="day_cal in 5" :key=day_cal>
          <div class="day" :class="{'day-active' : day_cal == 1}">
            9
          </div>
        </div>
      </div>
    </div>
</template>

<style lang="scss" scoped>
  @import './ScheduleCalendar.scss';
</style>

<script>
export default {
  name: 'ScheduleCalendar',
    data() {
    return {
      schedules: [
        {
          start: '09:00',
          end: '12:00',
          content: 'carrot 물 주기'
        },
        {
          start: '14:00',
          end: '17:00',
          content: 'carrot asdaadad'
        },
      ],
    }
  }

}
</script>