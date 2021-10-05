<template>
  <div class="calendar">
    <div class="day-container">
      <div v-for="(day, index) in 31" :key="index">
        <div class="day font-400" :class="{'day-active' : index == 0}">
          {{ day }}
        </div>
      </div>
    </div>
  </div>
  <!-- <SchedulePlan v-bind:date="selectDay"/> -->
</template>

<style lang="scss" scoped>
  @import './ScheduleCalendar.scss';
</style>

<script>
// import SchedulePlan from '../Schedule/SchedulePlan.vue'

let today = new Date()
let secondDay = new Date(today.getFullYear(), today.getMonth(), today.getDate() + 1)
let thirdDay = new Date(today.getFullYear(), today.getMonth(), today.getDate() + 2)
let fourthDay = new Date(today.getFullYear(), today.getMonth(), today.getDate() + 3)
let fifthDay = new Date(today.getFullYear(), today.getMonth(), today.getDate() + 4)

export default {
  data() {
  return {
    weekend: ['월요일', '화요일', '수요일', '목요일', '금요일', '토요일', '일요일',],
    startMonth: today,
    lastMonth: fifthDay,
    selectDay: today,
    day: [
      today, 
      secondDay, 
      thirdDay, 
      fourthDay, 
      fifthDay,
    ],
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
  },
  name: 'ScheduleCalendar',
  components: {
    // SchedulePlan,
  },
  methods: {
    setDate(date) {
      this.selectDay = date
    }
  },
}
</script>