<template>
  <div class="calendar-container">
    <font-awesome-icon icon="chevron-left" class="icon" @click="scrollLeft"/>
    <div class="calendar">
      <div class="day-container" id="days">
        <div v-for="(day, index) in 31" :key="index" @click="setDate(day)">
          <div class="day font-400" :class="{'day-active' : selectDay === day}">
            {{ day }}
          </div>
          <div v-if="todayDay === day" class="today-mark"></div>
        </div>
      </div>
    </div>
    <font-awesome-icon icon="chevron-right" class="icon" @click="scrollRight"/>
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
  mounted() {
    let dayContainer = document.querySelector('#days')
    dayContainer.scrollBy((todayGetDate-1) * 37 - 37, 0)
  },
  methods: {
    setDate(date) {
      this.selectDay = date
      this.$emit('set-date', date)
    },
    scrollLeft() {
      let dayContainer = document.querySelector('#days')
      dayContainer.scrollBy(-150, 0)
    },
    scrollRight() {
      let dayContainer = document.querySelector('#days')
      dayContainer.scrollBy(150, 0)
    }
  },
}
</script>

<style lang="scss" scoped>
  @import './ScheduleCalendar.scss';
</style>