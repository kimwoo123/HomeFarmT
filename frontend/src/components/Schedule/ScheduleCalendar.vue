<template>
  <div>
    <div class="calendar">
      <div class="year-container">
        <small class="year year-active">{{ startMonth.getFullYear() }}</small>
        <small class="year">{{ lastMonth.getFullYear() }}</small>
      </div>
      <div class="month-container">
        <div v-for="(date, index) in day" :key="index">
          <div class="month" :class="{'month-active' : index == 0}">
            {{ date.getMonth() + 1 }}월
            {{ date.getDate() }}일
          </div>
        </div>
      </div>
      <div class="day-container">
        <div v-for="(date, index) in day" :key="index">
          <div 
          class="day" 
          :class="{'day-active' : index == 0}"
          v-on:click="setDate(date)"
          >
            {{ weekend[date.getDay()] }}
          </div>
        </div>
      </div>
    </div>
    <SchedulePlan v-bind:date="selectDay"/>
  </div>
</template>

<style lang="scss" scoped>
  @import './ScheduleCalendar.scss';
</style>

<script>
import SchedulePlan from '../Schedule/SchedulePlan.vue'

// let weekend = ['월요일', '화요일', '수요일', '목요일', '금요일', '토요일', '일요일',]
// let date = new Date()
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
    SchedulePlan,
  },
  methods: {
    setDate(date) {
      this.selectDay = date
    }
  },
}
</script>