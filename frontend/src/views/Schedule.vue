<template>
  <div>

    <div class="calendar">
      <div class="year-container">
        2021
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

    <span class="schedule-text">Schedule</span>

    <div class="schedule">
      <div v-for="time in 25" :key=time>
        <div v-if="time <= 10" class="time-box">
          0{{ time-1 }}:00
        </div>
        <div v-else class="time-box">
          {{ time-1 }}:00
        </div>
      </div>
    </div>

  </div>
</template>


<script>
export default {
  name: 'Schedule',
  data() {
    return {
      today: {
        year: 2021,
        month: 9,
        day:14
      },
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

<style lang="scss" scoped>
  @import '@/views/Schedule.scss';
</style>