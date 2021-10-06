<template>
  <div class="calendar">
    <div class="day-container">
      <div v-for="(day, index) in 31" :key="index" @click="selectDay=day">
        <div class="day font-400" :class="{'day-active' : selectDay == day}">
          {{ day }}
        </div>
      </div>
    </div>
  </div>
  <!-- <SchedulePlan v-bind:date="selectDay"/> -->
</template>

<script>

let today = new Date()
let todayGetDate = today.getDate()
export default {
  name: 'ScheduleCalendar',
  data() {
    return {
      todayDay: todayGetDate,
      selectDay: todayGetDate
    }
  },
  methods: {
    setDate(date) {
      this.selectDay = date
    }
  },
}
</script>

<style lang="scss" scoped>
  @import './ScheduleCalendar.scss';
</style>